import math
import cereal.messaging as messaging
from cereal import log
import numpy as np
from numpy import clip, interp
from collections import deque
from common.filter_simple import FirstOrderFilter
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.lateral import ISO_LATERAL_ACCEL, apply_std_steer_angle_limits
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX
from selfdrive.modeld.constants import ModelConstants  # for calculations
from common.pid import PIDController # PID control of lateral
from opendbc.car.ford.helpers import compute_dm_msg_values
from openpilot.common.params import Params
from opendbc.sunnypilot.car.ford.icbm import IntelligentCruiseButtonManagementInterface

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

def index_function(idx, max_val=192, max_idx=32):
  return (max_val) * ((idx/max_idx)**2)

# ISO 11270
ISO_LATERAL_ACCEL = 3.0  # m/s^2  # TODO: import from test lateral limits file?

# Limit to average banked road since safety doesn't have the roll
EARTH_G = 9.81
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation
MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL - (EARTH_G * AVERAGE_ROAD_ROLL)  # ~2.4 m/s^2


def anti_overshoot(apply_curvature, apply_curvature_last, v_ego):
  diff = 0.1
  tau = 5  # 5s smooths over the overshoot
  dt = DT_CTRL * CarControllerParams.STEER_STEP
  alpha = 1 - np.exp(-dt / tau)

  lataccel = apply_curvature * (v_ego ** 2)
  last_lataccel = apply_curvature_last * (v_ego ** 2)
  last_lataccel = apply_hysteresis(lataccel, last_lataccel, diff)
  last_lataccel = alpha * lataccel + (1 - alpha) * last_lataccel

  output_curvature = last_lataccel / (max(v_ego, 1) ** 2)

  return float(np.interp(v_ego, [5, 10], [apply_curvature, output_curvature]))

def apply_ford_curvature_limits(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw, steering_angle, lat_active, CP):
  max_curvature = 1 # large initial value
  # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
  if v_ego_raw > 9:
    apply_curvature = np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                              current_curvature + CarControllerParams.CURVATURE_ERROR)
    max_curvature = abs(current_curvature) + CarControllerParams.CURVATURE_ERROR

  # Curvature rate limit after driver torque limit
  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, steering_angle, lat_active, CarControllerParams.ANGLE_LIMITS)

  steer_up = apply_curvature_last * apply_curvature >= 0. and abs(apply_curvature) > abs(apply_curvature_last)
  rate_limits = CarControllerParams.ANGLE_LIMITS.ANGLE_RATE_LIMIT_UP if steer_up else CarControllerParams.ANGLE_LIMITS.ANGLE_RATE_LIMIT_DOWN
  std_steer_angle_rate_limit = np.interp(v_ego_raw, rate_limits[0], rate_limits[1])
  std_steer_angle_limit = abs(apply_curvature_last) + abs(std_steer_angle_rate_limit)
  max_curvature = np.minimum(max_curvature, std_steer_angle_limit)

  # Ford Q4/CAN FD has more torque available compared to Q3/CAN so we limit it based on lateral acceleration.
  # Safety is not aware of the road roll so we subtract a conservative amount at all times
  if CP.flags & FordFlags.CANFD:
    # Limit curvature to conservative max lateral acceleration
    curvature_accel_limit = MAX_LATERAL_ACCEL / (max(v_ego_raw, 1) ** 2)
    apply_curvature = float(np.clip(apply_curvature, -curvature_accel_limit, curvature_accel_limit))
    max_curvature = np.minimum(max_curvature, abs(curvature_accel_limit))

  return apply_curvature, max_curvature


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
# class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    # IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)

    self.params = Params()

    self.packer = CANPacker(dbc_names[Bus.pt])
    self.CAN = fordcan.CanBus(CP)

    # Initialize control variables
    self.apply_curvature_last = 0
    self.accel = 0.0
    self.gas = 0.0
    self.accel_pred = -5.0  # AccPrpl_A_Pred: safe inactive until we send; avoids cruise fault on crank
    self.main_on_last = False
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.fcw_alert_last = False  # previous status of collision alert
    self.send_ui_last = False  # previous state of ui elements
    self.send_bars_ts_last = 0  # previous state of ui elements
    self.send_bars_last = False  # previous state of ACC Gap elements
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0
    self.last_button_frame = 0  # Track last ICBM button press frame
    self.lateralUncertainty = 0.0

    self.target_speed_multiplier = 1.0

    ################################## lateral control parameters ##############################################

    # Toggles
    self.enable_human_turn_detection = True
    self.enable_lane_positioning = True

    # Variables to initialize (these get updated every scan as part of the control code)
    self.precision_type = 1  # precise or comfort
    self.human_turn = False  # have we detected a human override in a turn
    self.post_reset_ramp_active = False  # track if we're ramping after a steering reset
    self.reset_steering_last = False  # track previous reset_steering state
    self.enable_lane_positioning = False # Updated from UI: enable Advanced Lane Positioning
    self.custom_profile = 0 # updated from UI
    self.pc_blend_ratio = 0.5
    self.disable_BP_lat_UI = False   # updated from UI: disable BP lateral control
    self.disable_BP_long_UI = False  # updated from UI: bypass BP longitudinal (use stock logic)
    self.anti_overshoot_curvature_last = 0.0 # initialize anti_overshoot_curvature_last
    self._bp_long_active_last = False  # True if we sent BP long values last frame (for clean transition off BP long)
    self.bp_gas_last = 0.0
    self.bp_accel_last = 0.0
    self.bpSpeedAllow = False # initialize to false

    # Long Control Variables
    self.MAX_URBAN_SPEED_MPH = 45.0
    self.following_accel_ROC = 0.002  # max accel change per scan when in following mode
    self.brake_actuate_target = -0.14 # at what accel value do we engage brakes
    self.brake_actuate_release = -0.06 # at what accel value do we release brakes
    self.precharge_actuate_target = -0.12 # at what accel value do we engage precharge
    self.precharge_actuate_release = -0.06 # at what accel value do we release precharge
    self.op_brake_actuate_last = False # init the value for our hysteresis
    self.disable_downhill_comp_UI = True #flag to disable downhill pitch compensation
    self.ford_stock_acc_stop_go_v12 = False
    self.ford_v12_resume_lead_move_t = 0.0
    self.ford_v12_hold_accel = -0.45
    self.ford_v12_touchdown_accel_bp = [0.0, 1.2]
    self.ford_v12_touchdown_accel_v = [self.ford_v12_hold_accel, -0.75]
    self.ford_v12_phase = "disabled"
    self.ford_v12_reason = "init"
    self.ford_v12_last_stop_request = False
    self.ford_v12_last_resume_enable = False
    self.ford_v12_last_target_speed = V_CRUISE_MAX
    self.ford_v12_last_accel = 0.0
    self.ford_v12_last_gas = CarControllerParams.INACTIVE_GAS
    self.ford_v12_last_brake_actuate = False
    self.ford_v12_last_precharge_actuate = False
    self.ford_v12_last_original_accel = 0.0
    self.ford_v12_last_original_gas = CarControllerParams.INACTIVE_GAS
    self.ford_v12_last_original_brake_actuate = False
    self.ford_v12_last_original_precharge_actuate = False
    self.ford_v12_last_lead_d_rel = 0.0
    self.ford_v12_last_lead_v_rel = 0.0
    self.ford_v12_last_lead_v_lead = 0.0
    self.ford_v12_last_ttc = 120.0
    self.ford_v12_last_resume_age = 0.0
    self.ford_v12_last_hold_accel = self.ford_v12_hold_accel
    self.ford_v12_last_touchdown_floor = self.ford_v12_hold_accel
    self.ford_v13_prev_lead_valid = False
    self.ford_v13_prev_d_rel = 0.0
    self.ford_v13_prev_v_rel = 0.0
    self.ford_v13_lead_stable_since = 0.0
    self.ford_v13_lead_lost_since = 0.0
    self.ford_v13_last_lead_stable = False
    self.ford_v13_last_lead_stability_age = 0.0
    self.ford_v13_last_lead_dropout_age = 0.0
    self.ford_v13_last_cut_in = False
    self.ford_v13_last_cut_out = False
    self.ford_v13_last_gate_reason = "init"
    self.ford_stock_acc_go_release_v14 = False
    self.ford_v14_lead_move_t = 0.0
    self.ford_v14_prev_t = 0.0
    self.ford_v14_prev_jerk_limited_accel = 0.0
    self.ford_v14_last_lead_moved = False
    self.ford_v14_last_lead_stable = False
    self.ford_v14_last_lead_stable_age = 0.0
    self.ford_v14_last_time_since_lead_move = 0.0
    self.ford_v14_last_ego_lag = 0.0
    self.ford_v14_last_candidate = False
    self.ford_v14_last_blocked_reason = "init"
    self.ford_v14_last_desired_accel = 0.0
    self.ford_v14_last_jerk_limited_accel = 0.0
    self.ford_v14_last_release_phase = "init"
    self.ford_v14_last_control_enabled = False
    self.ford_stock_acc_soft_crawl_observe = True
    self.ford_stock_acc_soft_crawl_control = False
    self.ford_stock_acc_soft_crawl_v16 = False
    self.ford_stock_acc_soft_crawl_v17 = False
    self.ford_stock_acc_soft_crawl_v171 = False
    self.ford_stock_acc_soft_crawl_v172 = False
    self.ford_soft_crawl_last_available = False
    self.ford_soft_crawl_last_reason = "init"
    self.ford_soft_crawl_last_fallback_reason = "init"
    self.ford_soft_crawl_last_phase = "init"
    self.ford_soft_crawl_last_control_stage = "init"
    self.ford_soft_crawl_last_time_to_stop_est = 0.0
    self.ford_soft_crawl_last_main_control_distance = 0.0
    self.ford_soft_crawl_last_close_control_distance = 0.0
    self.ford_soft_crawl_last_main_control_allowed = False
    self.ford_soft_crawl_last_target_accel = 0.0
    self.ford_soft_crawl_last_raw_target_accel = 0.0
    self.ford_soft_crawl_last_jerk_limited_accel = 0.0
    self.ford_soft_crawl_last_accel_after_control = 0.0
    self.ford_soft_crawl_last_stop_gap_target = 0.0
    self.ford_soft_crawl_last_distance_to_stop = 0.0
    self.ford_soft_crawl_last_needed_distance = 0.0
    self.ford_soft_crawl_last_distance_margin = 0.0
    self.ford_soft_crawl_last_current_stop_distance = 0.0
    self.ford_soft_crawl_last_lead_d_rel = 0.0
    self.ford_soft_crawl_last_lead_v_rel = 0.0
    self.ford_soft_crawl_last_lead_v_lead = 0.0
    self.ford_soft_crawl_last_ttc = 120.0
    self.ford_soft_crawl_last_v_ego = 0.0
    self.ford_soft_crawl_last_original_accel = 0.0
    self.ford_soft_crawl_last_planner_stopping = False
    self.ford_soft_crawl_last_control_active = False
    self.ford_soft_crawl_prev_t = 0.0
    self.ford_soft_crawl_prev_accel = 0.0

    # # Curvature variables
    self.curvature_lookup_time = 0.42 # from lagd (how far into the future we pull curvature)
    self.lane_change_factor_bp = [4.4, 40.23] # what speeds to adjust lane_change_factor
    self.lane_change_factor_low = 0.95 # lane_change_factor at 4.4 m/s
    self.lane_change_factor_high = 0.85 # updated from UI: lane_change_factor at 40.23 m/s
    self.pc_blend_ratio_low = 0.40 # Default %-Predicted Curvature on straights
    self.pc_blend_ratio_high = 0.40 # Default %-Predicted Curvature in curves
    self.pc_blend_ratio_low_C = 0.40   # used in pc_blend_ratio_v (from UI when custom_profile == 1)
    self.pc_blend_ratio_high_C = 0.40 # used in pc_blend_ratio_v (from UI when custom_profile == 1)
    self.pc_blend_ratio_bp = [0.0, 0.001] # curvature breakpoints in 1/m

    # Curvature rate variables
    self.curvature_rate_delta_t = 0.3  # [s] used in denominator for curvature rate calculation
    self.curvature_rate_deque = deque(maxlen=int(round(self.curvature_rate_delta_t / 0.05)))  # 0.3 seconds at 20Hz
    self.curvature_rate_speed_bp = [0.0, 14.5, 15.5]  # speed breakpoints in m/s
    self.curvature_rate_speed_v = [1.0, 1.0, 0.0]  # corresponding k_p values
    self.curvature_rate_PC_bp = [0.0, 0.008,0.01] # curvature breakpoints in 1/m
    self.curvature_rate_PC_v = [0.0, 0.0, 1.0] # corresponding k_p values
    self.large_curve_factor_low = 1.0 # factor to reduce curvature for small curves
    self.large_curve_factor_high = 0.80 # factor to reduce curvature for large curves
    self.large_curve_factor_bp = [0.001, 0.02] # curvature breakpoints in 1/m
    self.large_curve_factor_v = [self.large_curve_factor_low, self.large_curve_factor_high]



    # path offset variables
    self.custom_path_offset = 0.0 # updated from UI: applies a custom offset to help with in-lane positioning
    self.path_offset_lookup_time = 0.2 # in seconds (from bp-2.1)
    self.min_laneline_confidence_bp = [0.6, 0.8]
    self.enable_lanefull_mode = True

    #path angle shared variables
    self.path_angle_filter_samples = 3 # number of samples to use for the moving average filter
    self.path_angle_deque = deque(maxlen=self.path_angle_filter_samples) # deque to hold the samples

    # path angle low curvature variables
    self.LC_PID_gain_UI = 0.0  # gain for UI tuning (from params)
    self.LC_PID_gain = 3.0  # effective gain (LC_PID_GAIN or from UI when custom_profile == 1)
    self.LC_PID_k_p = 0.25
    self.LC_PID_k_i = 0.05
    self.LC_PID_controller = PIDController(k_p=self.LC_PID_k_p, k_i=self.LC_PID_k_i, rate=20)
    self.LC_PID_speed_bp = [0.0, 9.0, 15.0]  # speed breakpoints in m/s
    self.LC_PID_speed_v = [0.0, 0.0, 1.0]  # corresponding k_p values
    self.LC_path_angle_ROC_bp = [5, 15, 25]  # speed breakpoints in m/s
    self.LC_path_angle_ROC_v = [0.003, 0.0015, 0.002]  # match panda limits
    self.LC_path_angle_reset_counter = 0
    self.LC_path_angle_reset_duration = 1.5 # in seconds

    # max absolute values for all four signals
    self.path_angle_max = 0.5  # from dbc files
    self.path_offset_max = 2.0  # too much path offset causes issues
    self.curvature_max = 0.02  # 0.02 is max from dbc files
    self.curvature_rate_max = 0.001023  # from dbc files

    # values from previous frame
    self.curvature_rate_last = 0.0
    self.path_offset_last = 0.0
    self.path_angle_last = 0.0
    self.curvature_rate = 0  # initialize curvature_rate

    # Logging variables
    #debug(f'Car Fingerprint (CarController): {CP.carFingerprint}', True)

    # Lane change transition tracking
    self.post_lane_change_timer = 0
    self.post_lane_change_active = False
    self.lane_change = False  # True when model indicates lane change active
    self.lane_change_last = False  # Track previous lane change state
    self.pre_lane_change_values = {
        'path_angle': 0.0,
        'path_offset': 0.0,
        'desired_curvature_rate': 0.0
    }

    # Maximum allowed changes per frame
    self.max_path_angle_change = 0.00125
    self.max_path_offset_change = 0.00125
    self.max_curvature_rate_change = 0.0001

    self.sm = messaging.SubMaster(['modelV2', 'liveParameters', 'selfdriveState', 'radarState'])
    self.VM = VehicleModel(self.CP)
    self.curvature_lookup_time = 0.2

    self.model = None
    self.lp = None
    self.ss = None
    self.send_driver_monitor_can_msg = False
    self.send_lane_depart_can_msg = False
    self.tja_msg = 0
    self.tja_warn = 0
    self.hands = 0
    self._update_params()

  def _update_params(self):
    self.send_hands_free_cluster_msg = self.params.get_bool("send_hands_free_cluster_msg")
    # Block hands-free UI on CAN vehicles - only CAN-FD supports the cluster message
    if not (self.CP.flags & FordFlags.CANFD):
      self.send_hands_free_cluster_msg = False
    self.enable_human_turn_detection = self.params.get_bool("enable_human_turn_detection")
    # updated from UI: lane_change_factor at 40.23 m/s
    self.lane_change_factor_high = float(self.params.get("lane_change_factor_high", return_default=True))
    self.pc_blend_ratio_high_C_UI = float(self.params.get("pc_blend_ratio_high_C_UI", return_default=True))
    self.pc_blend_ratio_low_C_UI = float(self.params.get("pc_blend_ratio_low_C_UI", return_default=True))
    self.enable_lane_positioning = self.params.get_bool("enable_lane_positioning")
    # updated from UI: applies a custom offset to help with in-lane positioning
    self.custom_path_offset = float(self.params.get("custom_path_offset", return_default=True))
    self.enable_lanefull_mode = self.params.get_bool("enable_lane_full_mode")
    self.custom_profile = int(self.params.get("custom_profile", return_default=True))
    self.LC_PID_gain_UI = float(self.params.get("LC_PID_gain_UI", return_default=True))
    # Ford long: bypass BP longitudinal toggle (gas/accel ROC use __init__ defaults only)
    self.disable_BP_long_UI = self.params.get_bool("disable_BP_long_UI")
    self.disable_downhill_comp_UI = self.params.get_bool("disable_downhill_comp_UI")
    self.ford_stock_acc_stop_go_v12 = self.params.get_bool("FordStockAccStopGoV12")
    self.ford_stock_acc_go_release_v14 = self.params.get_bool("FordStockAccGoReleaseV14")
    self.ford_stock_acc_soft_crawl_observe = bool(self.params.get("FordStockAccSoftCrawlObserve", return_default=True))
    self.ford_stock_acc_soft_crawl_control = bool(self.params.get("FordStockAccSoftCrawlControl", return_default=True))
    self.ford_stock_acc_soft_crawl_v16 = bool(self.params.get("FordStockAccSoftCrawlV16", return_default=True))
    self.ford_stock_acc_soft_crawl_v17 = bool(self.params.get("FordStockAccSoftCrawlV17", return_default=True))
    self.ford_stock_acc_soft_crawl_v171 = bool(self.params.get("FordStockAccSoftCrawlV171", return_default=True))
    self.ford_stock_acc_soft_crawl_v172 = bool(self.params.get("FordStockAccSoftCrawlV172", return_default=True))

  def _ford_stock_acc_lead(self, v_ego):
    lead = None
    if self.sm.valid.get('radarState', False):
      rs = self.sm['radarState']
      lead = getattr(rs, 'leadOne', None)
      if lead is not None and getattr(lead, 'status', 0) != 1:
        lead = None

    d_rel = float(getattr(lead, 'dRel', 0.0)) if lead is not None else 0.0
    v_rel = float(getattr(lead, 'vRel', 0.0)) if lead is not None else 0.0
    v_lead = float(getattr(lead, 'vLead', 0.0)) if lead is not None else 0.0
    lead_valid = lead is not None and d_rel > 0.0
    lead_time = d_rel / max(v_ego, 0.5) if lead_valid else 999.0
    ttc = d_rel / (-v_rel) if lead_valid and v_rel < -0.05 else 60.0
    return lead_valid, d_rel, v_rel, v_lead, float(clip(lead_time, 0.0, 999.0)), float(clip(ttc, 0.2, 120.0))

  def _ford_stock_acc_lead_gate_v13(self, lead_data, now):
    lead_valid, d_rel, v_rel, _, _, _ = lead_data
    prev_valid = self.ford_v13_prev_lead_valid
    prev_d_rel = self.ford_v13_prev_d_rel

    cut_in = False
    cut_out = False
    if lead_valid and prev_valid:
      cut_in = d_rel < prev_d_rel - max(4.0, 0.18 * max(prev_d_rel, 1.0)) or (v_rel < -4.0 and d_rel < 25.0)
      cut_out = d_rel > prev_d_rel + max(6.0, 0.25 * max(prev_d_rel, 1.0))
    elif prev_valid and not lead_valid:
      cut_out = True

    if lead_valid:
      self.ford_v13_lead_lost_since = 0.0
      if not prev_valid or cut_in or cut_out or self.ford_v13_lead_stable_since <= 0.0:
        self.ford_v13_lead_stable_since = now
    else:
      if prev_valid or self.ford_v13_lead_lost_since <= 0.0:
        self.ford_v13_lead_lost_since = now
      self.ford_v13_lead_stable_since = 0.0

    lead_stability_age = now - self.ford_v13_lead_stable_since if self.ford_v13_lead_stable_since > 0.0 else 0.0
    lead_dropout_age = now - self.ford_v13_lead_lost_since if self.ford_v13_lead_lost_since > 0.0 else 0.0
    lead_stable = bool(lead_valid and lead_stability_age >= 0.5 and abs(v_rel) < 6.0 and not cut_in and not cut_out)

    if not lead_valid:
      gate_reason = "lead_missing"
    elif cut_in:
      gate_reason = "cut_in"
    elif cut_out:
      gate_reason = "cut_out"
    elif lead_stability_age < 0.5:
      gate_reason = "lead_warmup"
    elif abs(v_rel) >= 6.0:
      gate_reason = "vrel_jump"
    else:
      gate_reason = "stable"

    self.ford_v13_prev_lead_valid = bool(lead_valid)
    self.ford_v13_prev_d_rel = float(d_rel)
    self.ford_v13_prev_v_rel = float(v_rel)
    self.ford_v13_last_lead_stable = lead_stable
    self.ford_v13_last_lead_stability_age = float(lead_stability_age)
    self.ford_v13_last_lead_dropout_age = float(lead_dropout_age)
    self.ford_v13_last_cut_in = bool(cut_in)
    self.ford_v13_last_cut_out = bool(cut_out)
    self.ford_v13_last_gate_reason = gate_reason

    return lead_stable, lead_stability_age, lead_dropout_age, cut_in, cut_out, gate_reason

  def _ford_stock_acc_go_release_v14(self, CC, CS, lead_data, lead_stable, lead_stability_age,
                                     gate_reason, standstill, human_override, stopping, now, current_accel):
    lead_valid, d_rel, v_rel, v_lead, _, ttc = lead_data
    v_ego = max(float(CS.out.vEgo), 0.0)
    lead_moved = bool(lead_valid and lead_stable and (v_lead > 0.35 or v_rel > 0.35))

    if lead_moved and self.ford_v14_lead_move_t <= 0.0:
      self.ford_v14_lead_move_t = now
    elif not standstill or human_override or not lead_valid:
      self.ford_v14_lead_move_t = 0.0

    time_since_lead_move = now - self.ford_v14_lead_move_t if self.ford_v14_lead_move_t > 0.0 else 0.0
    ego_lag = time_since_lead_move if time_since_lead_move > 0.0 and v_ego < 0.28 else 0.0

    blocked_reason = "none"
    if not CC.longActive:
      blocked_reason = "long_inactive"
    elif human_override:
      blocked_reason = "human_override"
    elif not standstill:
      blocked_reason = "not_standstill"
    elif not lead_valid:
      blocked_reason = "lead_missing"
    elif not lead_stable:
      blocked_reason = f"lead_gate_{gate_reason}"
    elif not lead_moved:
      blocked_reason = "lead_not_moving"
    elif stopping:
      blocked_reason = "planner_stopping"
    elif d_rel < 2.5:
      blocked_reason = "lead_too_close"
    elif ttc < 1.5:
      blocked_reason = "short_ttc"

    candidate = blocked_reason == "none"
    if candidate:
      if v_ego > 0.83:
        release_phase = "handoff"
      elif time_since_lead_move < 0.25:
        release_phase = "warmup"
      elif time_since_lead_move < 1.5:
        release_phase = "ramp"
      else:
        release_phase = "assertive"
    else:
      release_phase = "blocked"

    desired_accel = 0.0
    if candidate:
      desired_accel = float(interp(time_since_lead_move, [0.0, 0.5, 1.5, 2.0], [0.8, 1.0, 1.6, 1.7]))

    dt = now - self.ford_v14_prev_t if self.ford_v14_prev_t > 0.0 else DT_CTRL
    dt = float(clip(dt, DT_CTRL, 0.2))
    jerk_limit = 1.8
    if candidate:
      start_accel = self.ford_v14_prev_jerk_limited_accel if self.ford_v14_prev_t > 0.0 else max(current_accel, 0.0)
      jerk_limited_accel = float(clip(desired_accel, start_accel - jerk_limit * dt, start_accel + jerk_limit * dt))
    else:
      jerk_limited_accel = desired_accel

    self.ford_v14_prev_t = now
    self.ford_v14_prev_jerk_limited_accel = jerk_limited_accel if candidate else 0.0
    self.ford_v14_last_lead_moved = lead_moved
    self.ford_v14_last_lead_stable = lead_stable
    self.ford_v14_last_lead_stable_age = float(lead_stability_age)
    self.ford_v14_last_time_since_lead_move = float(time_since_lead_move)
    self.ford_v14_last_ego_lag = float(ego_lag)
    self.ford_v14_last_candidate = candidate
    self.ford_v14_last_blocked_reason = blocked_reason
    self.ford_v14_last_desired_accel = float(desired_accel)
    self.ford_v14_last_jerk_limited_accel = float(jerk_limited_accel)
    self.ford_v14_last_release_phase = release_phase
    self.ford_v14_last_control_enabled = bool(self.ford_stock_acc_go_release_v14 and candidate and release_phase != "warmup")

    return candidate, release_phase, jerk_limited_accel

  def _ford_stock_acc_soft_crawl_observe_v1(self, CC, CS, lead_data, lead_stable, lead_stability_age,
                                            cut_in, cut_out, stopping, original_accel, now):
    lead_valid, d_rel, v_rel, v_lead, lead_time, ttc = lead_data
    v_ego = max(float(CS.out.vEgo), 0.0)
    human_override = bool(CS.out.gasPressed or CS.out.brakePressed)
    standstill = bool(CS.out.standstill or CS.out.cruiseState.standstill or v_ego < 0.05)
    original_accel = float(original_accel)

    # v1.6+ have two stock-like stop phases:
    # - early_soften: start reducing OP's heavier braking while there is still room to recover.
    # - final_crawl: very low-speed glide toward the stock stop gap.
    stock_soft_crawl_v172 = bool(self.ford_stock_acc_soft_crawl_v172)
    stock_soft_crawl_v171_gates = bool(self.ford_stock_acc_soft_crawl_v171 or stock_soft_crawl_v172)
    stock_soft_crawl_v17 = bool(self.ford_stock_acc_soft_crawl_v17 or stock_soft_crawl_v171_gates)
    early_decel = float(interp(v_ego, [0.0, 1.0, 3.0, 6.0, 8.0], [0.24, 0.34, 0.50, 0.74, 0.88]))
    final_decel = float(interp(v_ego, [0.0, 0.8, 1.8, 3.0], [0.18, 0.23, 0.32, 0.45]))
    early_decel = max(early_decel, 0.22)
    final_decel = max(final_decel, 0.18)
    stop_gap_target = float(interp(v_ego, [0.0, 2.0, 6.0, 12.0], [3.8, 4.1, 5.8, 9.0]))
    current_decel = max(-original_accel, 0.25)
    distance_to_stop = max(d_rel - stop_gap_target, 0.0) if lead_valid else 0.0
    required_decel = (v_ego * v_ego) / (2.0 * max(distance_to_stop, 0.8)) if lead_valid else early_decel
    if stock_soft_crawl_v17:
      # v1.7 moves the early soften window forward, but keeps the target decel
      # tied to the distance actually available behind the lead.
      recoverable_decel = max(current_decel - 0.12, early_decel)
      early_decel = float(clip(max(early_decel * 0.90, required_decel * 0.92), 0.22, recoverable_decel))

    early_needed_distance = (v_ego * v_ego) / (2.0 * early_decel) + stop_gap_target
    final_needed_distance = (v_ego * v_ego) / (2.0 * final_decel) + stop_gap_target
    needed_distance = early_needed_distance

    current_stop_distance = (v_ego * v_ego) / (2.0 * current_decel) + stop_gap_target
    distance_margin = d_rel - needed_distance if lead_valid else 0.0
    final_distance_margin = d_rel - final_needed_distance if lead_valid else 0.0
    phase = "none"
    control_stage = "none"
    target_accel = -early_decel

    if stock_soft_crawl_v17:
      approach_context = bool(stopping or (
        lead_valid and v_ego < 8.5 and d_rel < max(45.0, (v_ego * 6.0) + 14.0) and v_rel < 0.65
      ))
      early_margin_floor = float(interp(v_ego, [0.0, 2.0, 5.0, 8.5], [-0.8, -1.2, -2.6, -4.0]))
      early_recoverable = bool(required_decel <= current_decel - 0.08 or current_decel <= early_decel + 0.18)
      early_braking_context = bool(stopping or original_accel < -0.18 or v_rel < 0.15 or current_decel > early_decel + 0.18)
      early_window = bool(v_ego < 8.5 and distance_margin >= early_margin_floor and
                          early_braking_context and early_recoverable)
    else:
      approach_context = bool(stopping or (lead_valid and d_rel < 45.0 and v_rel < 0.25 and v_ego < 8.0))
      early_window = bool(v_ego < 6.8 and distance_margin >= -0.8 and
                          (stopping or original_accel < -0.35 or v_rel < -0.05))
    final_window = bool(v_ego < 2.2 and final_distance_margin >= 0.0 and
                        distance_to_stop <= max(5.0, (v_ego * 3.0) + 1.2))
    fallback_reason = "none"
    if not self.ford_stock_acc_soft_crawl_observe:
      fallback_reason = "observe_disabled"
    elif not CC.longActive:
      fallback_reason = "long_inactive"
    elif human_override:
      fallback_reason = "human_override"
    elif standstill:
      fallback_reason = "standstill"
    elif v_ego > 8.0:
      fallback_reason = "speed_too_high"
    elif not lead_valid:
      fallback_reason = "lead_missing"
    elif not lead_stable:
      fallback_reason = "lead_unstable"
    elif lead_stability_age < 0.6:
      fallback_reason = "lead_warmup"
    elif cut_in:
      fallback_reason = "lead_cut_in"
    elif cut_out:
      fallback_reason = "lead_cut_out"
    elif not approach_context:
      fallback_reason = "not_approach_context"
    elif v_rel > 0.35 and not stopping:
      fallback_reason = "lead_pulling_away"
    elif ttc < 2.0:
      fallback_reason = "short_ttc"
    elif lead_time < 0.7:
      fallback_reason = "short_lead_time"
    elif not early_window and not final_window:
      fallback_reason = "not_soft_crawl_window"

    if fallback_reason == "none":
      if final_window:
        phase = "final_crawl"
        control_stage = "observe_only"
        target_accel = -final_decel
        needed_distance = final_needed_distance
        distance_margin = final_distance_margin
        if distance_margin < 0.2:
          fallback_reason = "insufficient_final_distance"
      elif early_window:
        phase = "early_soften"
        control_stage = "main"
        target_accel = -early_decel
        if distance_margin < -0.8:
          fallback_reason = "insufficient_distance"
      else:
        fallback_reason = "not_soft_crawl_window"

    time_to_stop_est = v_ego / max(current_decel, 0.25)
    main_control_distance = max(7.5, (v_ego * 1.6) + 2.0)
    close_control_distance = max(5.5, (v_ego * 1.15) + 1.0)
    v171_main_control = True
    if fallback_reason == "none" and stock_soft_crawl_v171_gates and phase == "early_soften":
      time_to_stop_est = v_ego / max(current_decel, 0.25)
      v171_main_control = bool(v_ego < 2.35 and (
        (time_to_stop_est <= 5.0 and distance_to_stop <= main_control_distance) or
        distance_to_stop <= close_control_distance
      ))
      if stock_soft_crawl_v172:
        control_stage = "observe_main" if v171_main_control else "observe_probe"
        target_accel = min(target_accel, original_accel + (0.18 if v171_main_control else 0.04))
      elif not v171_main_control:
        control_stage = "probe"
        target_accel = min(target_accel, original_accel + 0.04)
      else:
        target_accel = min(target_accel, original_accel + 0.18)

    if fallback_reason == "none" and target_accel <= original_accel + 0.03:
      fallback_reason = "already_soft"

    if fallback_reason == "none" and distance_margin < -0.8:
      fallback_reason = "insufficient_distance"

    available = fallback_reason == "none"
    dt = now - self.ford_soft_crawl_prev_t if self.ford_soft_crawl_prev_t > 0.0 else DT_CTRL
    dt = float(clip(dt, DT_CTRL, 0.2))
    release_jerk = 0.65 if phase == "early_soften" else 0.45
    if stock_soft_crawl_v171_gates and phase == "early_soften":
      release_jerk = 0.55 if v171_main_control else 0.20
    start_accel = self.ford_soft_crawl_prev_accel if self.ford_soft_crawl_prev_t > 0.0 and available else original_accel
    jerk_limited_accel = original_accel
    if available:
      jerk_limited_accel = min(target_accel, start_accel + release_jerk * dt)
      jerk_limited_accel = max(jerk_limited_accel, original_accel)

    control_active = bool(self.ford_stock_acc_soft_crawl_control and
                          (self.ford_stock_acc_soft_crawl_v16 or self.ford_stock_acc_soft_crawl_v17 or
                           self.ford_stock_acc_soft_crawl_v171) and
                          not stock_soft_crawl_v172 and
                          phase == "early_soften" and available and
                          (not self.ford_stock_acc_soft_crawl_v171 or control_stage == "main") and
                          jerk_limited_accel > original_accel + 0.01)
    if available and phase == "early_soften":
      self.ford_soft_crawl_prev_t = now
      self.ford_soft_crawl_prev_accel = jerk_limited_accel
    else:
      self.ford_soft_crawl_prev_t = 0.0
      self.ford_soft_crawl_prev_accel = original_accel

    if available:
      reason = f"{phase}_available"
    elif fallback_reason in ("observe_disabled", "long_inactive", "human_override", "standstill"):
      reason = "not_evaluating"
    else:
      reason = "fallback"

    self.ford_soft_crawl_last_available = bool(available)
    self.ford_soft_crawl_last_reason = reason
    self.ford_soft_crawl_last_fallback_reason = fallback_reason
    self.ford_soft_crawl_last_phase = phase
    self.ford_soft_crawl_last_control_stage = control_stage
    self.ford_soft_crawl_last_time_to_stop_est = float(time_to_stop_est)
    self.ford_soft_crawl_last_main_control_distance = float(main_control_distance)
    self.ford_soft_crawl_last_close_control_distance = float(close_control_distance)
    self.ford_soft_crawl_last_main_control_allowed = bool(v171_main_control)
    self.ford_soft_crawl_last_target_accel = float(target_accel)
    self.ford_soft_crawl_last_raw_target_accel = float(target_accel)
    self.ford_soft_crawl_last_jerk_limited_accel = float(jerk_limited_accel)
    self.ford_soft_crawl_last_accel_after_control = float(jerk_limited_accel if control_active else original_accel)
    self.ford_soft_crawl_last_stop_gap_target = float(stop_gap_target)
    self.ford_soft_crawl_last_distance_to_stop = float(distance_to_stop)
    self.ford_soft_crawl_last_needed_distance = float(needed_distance)
    self.ford_soft_crawl_last_distance_margin = float(distance_margin)
    self.ford_soft_crawl_last_current_stop_distance = float(current_stop_distance)
    self.ford_soft_crawl_last_lead_d_rel = float(d_rel)
    self.ford_soft_crawl_last_lead_v_rel = float(v_rel)
    self.ford_soft_crawl_last_lead_v_lead = float(v_lead)
    self.ford_soft_crawl_last_ttc = float(ttc)
    self.ford_soft_crawl_last_v_ego = float(v_ego)
    self.ford_soft_crawl_last_original_accel = float(original_accel)
    self.ford_soft_crawl_last_planner_stopping = bool(stopping)
    self.ford_soft_crawl_last_control_active = control_active

    return available, phase, jerk_limited_accel

  def _record_ford_stock_acc_v12(self, phase, reason, accel, gas, brake_actuate, precharge_actuate,
                                 stop_request, resume_enable, target_speed, original_accel, original_gas,
                                 original_brake_actuate, original_precharge_actuate, lead_data, resume_age,
                                 touchdown_floor=None):
    lead_valid, d_rel, v_rel, v_lead, _, ttc = lead_data
    self.ford_v12_phase = phase
    self.ford_v12_reason = reason
    self.ford_v12_last_stop_request = bool(stop_request)
    self.ford_v12_last_resume_enable = bool(resume_enable)
    self.ford_v12_last_target_speed = float(target_speed)
    self.ford_v12_last_accel = float(accel)
    self.ford_v12_last_gas = float(gas)
    self.ford_v12_last_brake_actuate = bool(brake_actuate)
    self.ford_v12_last_precharge_actuate = bool(precharge_actuate)
    self.ford_v12_last_original_accel = float(original_accel)
    self.ford_v12_last_original_gas = float(original_gas)
    self.ford_v12_last_original_brake_actuate = bool(original_brake_actuate)
    self.ford_v12_last_original_precharge_actuate = bool(original_precharge_actuate)
    self.ford_v12_last_lead_valid = bool(lead_valid)
    self.ford_v12_last_lead_d_rel = float(d_rel)
    self.ford_v12_last_lead_v_rel = float(v_rel)
    self.ford_v12_last_lead_v_lead = float(v_lead)
    self.ford_v12_last_ttc = float(ttc)
    self.ford_v12_last_resume_age = float(resume_age or 0.0)
    self.ford_v12_last_hold_accel = float(self.ford_v12_hold_accel)
    self.ford_v12_last_touchdown_floor = float(touchdown_floor if touchdown_floor is not None else self.ford_v12_hold_accel)

  def _apply_ford_stock_acc_stop_go_v12(self, CC, CS, accel, gas, brake_actuate, precharge_actuate,
                                        stopping, target_speed, now_nanos):
    original_accel = accel
    original_gas = gas
    original_brake_actuate = brake_actuate
    original_precharge_actuate = precharge_actuate
    stop_request = bool(stopping)
    resume_enable = bool(CC.longActive)
    v_ego = max(float(CS.out.vEgo), 0.0)
    now = float(now_nanos) * 1e-9
    standstill = bool(CS.out.standstill or CS.out.cruiseState.standstill or v_ego < 0.05)
    human_override = bool(CS.out.gasPressed or CS.out.brakePressed)
    lead_data = self._ford_stock_acc_lead(v_ego)
    lead_valid, d_rel, v_rel, v_lead, lead_time, ttc = lead_data
    lead_stable, lead_stability_age, _, cut_in, cut_out, gate_reason = self._ford_stock_acc_lead_gate_v13(lead_data, now)
    go_candidate, go_release_phase, go_accel = self._ford_stock_acc_go_release_v14(
      CC, CS, lead_data, lead_stable, lead_stability_age, gate_reason, standstill, human_override, stopping, now, accel
    )
    soft_crawl_available, soft_crawl_phase, soft_crawl_accel = self._ford_stock_acc_soft_crawl_observe_v1(
      CC, CS, lead_data, lead_stable, lead_stability_age, cut_in, cut_out, stopping, original_accel, now
    )
    if self.ford_soft_crawl_last_control_active:
      accel = float(max(accel, soft_crawl_accel))
      gas = CarControllerParams.INACTIVE_GAS
      brake_actuate = accel < self.brake_actuate_target
      precharge_actuate = accel < self.precharge_actuate_target
      self.ford_soft_crawl_last_accel_after_control = float(accel)
    resume_age = 0.0

    phase = "pass_through"
    reason = "disabled"
    if not self.ford_stock_acc_stop_go_v12:
      self.ford_v12_resume_lead_move_t = 0.0
      if self.ford_soft_crawl_last_control_active:
        phase = soft_crawl_phase
        reason = "soft_crawl_v16"
      elif soft_crawl_available:
        phase = soft_crawl_phase
        reason = "soft_crawl_observe"
      self._record_ford_stock_acc_v12(phase, reason, accel, gas, brake_actuate, precharge_actuate,
                                      stop_request, resume_enable, target_speed, original_accel, original_gas,
                                      original_brake_actuate, original_precharge_actuate, lead_data, resume_age)
      return accel, gas, brake_actuate, precharge_actuate, stop_request, resume_enable, target_speed

    if not CC.longActive:
      self.ford_v12_resume_lead_move_t = 0.0
      phase = "inactive"
      reason = "long_inactive"
    elif human_override:
      self.ford_v12_resume_lead_move_t = 0.0
      phase = "pass_through"
      reason = "human_override"
    elif v_ego > 12.0 and not stopping:
      self.ford_v12_resume_lead_move_t = 0.0
      phase = "pass_through"
      reason = "speed_too_high"
    elif lead_valid and ttc < 1.2:
      phase = "guard"
      reason = "short_ttc"
    elif cut_in:
      phase = "guard"
      reason = "lead_cut_in"
    else:
      low_speed_context = v_ego < 8.0 or stopping or standstill or (lead_valid and d_rel < 35.0)
      if not low_speed_context:
        self.ford_v12_resume_lead_move_t = 0.0
        phase = "pass_through"
        reason = "not_low_speed"
      else:
        resume_candidate = bool(standstill and lead_stable and (v_lead > 0.35 or v_rel > 0.35))
        if resume_candidate and self.ford_v12_resume_lead_move_t <= 0.0:
          self.ford_v12_resume_lead_move_t = now
        elif not resume_candidate:
          self.ford_v12_resume_lead_move_t = 0.0
        resume_age = now - self.ford_v12_resume_lead_move_t if self.ford_v12_resume_lead_move_t > 0.0 else 0.0

        if standstill:
          if resume_candidate and resume_age >= 0.25 and not stopping:
            phase = "resume_release"
            reason = "lead_moving"
            if self.ford_stock_acc_go_release_v14 and go_candidate and go_release_phase != "warmup":
              phase = "go_release"
              reason = f"stock_like_go_{go_release_phase}"
              stop_request = False
              resume_enable = bool(CC.longActive)
              brake_actuate = False
              precharge_actuate = False
              accel = max(accel, 0.0)
              gas = max(gas, go_accel)
          else:
            phase = "hold"
            reason = "standstill_wait"
            stop_request = True
            resume_enable = bool(CC.longActive)
            accel = self.ford_v12_hold_accel
        elif stopping:
          phase = "touchdown"
          reason = "touchdown_taper_check" if lead_stable else f"touchdown_lead_gate_{gate_reason}"
          if lead_stable and d_rel > 2.5 and lead_time > 0.6 and ttc > 2.0 and v_ego < 1.2:
            accel_floor = float(interp(v_ego, self.ford_v12_touchdown_accel_bp, self.ford_v12_touchdown_accel_v))
            if accel < accel_floor:
              accel = accel_floor
              reason = "touchdown_taper"
        elif brake_actuate or precharge_actuate:
          phase = "approach_brake"
          reason = "baseline_resume_target"
        else:
          phase = "approach"
          reason = "baseline_resume_target"

    if brake_actuate:
      gas = CarControllerParams.INACTIVE_GAS

    accel = float(clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    if gas != CarControllerParams.INACTIVE_GAS:
      gas = float(clip(gas, CarControllerParams.MIN_GAS, CarControllerParams.ACCEL_MAX))
    target_speed = float(clip(target_speed, 0.0, V_CRUISE_MAX))

    self._record_ford_stock_acc_v12(phase, reason, accel, gas, brake_actuate, precharge_actuate,
                                    stop_request, resume_enable, target_speed, original_accel, original_gas,
                                    original_brake_actuate, original_precharge_actuate, lead_data, resume_age,
                                    locals().get("accel_floor"))
    return accel, gas, brake_actuate, precharge_actuate, stop_request, resume_enable, target_speed

  def handle_post_lane_change_transition(self, path_angle, path_offset, desired_curvature_rate):
    """
    Manages smooth transition of control variables after lane change
    Returns: Tuple of (path_angle, path_offset, desired_curvature_rate)
    """
    # Detect lane change completion (transition from True to False)
    if self.lane_change_last and not self.lane_change:
        self.post_lane_change_active = True
        self.post_lane_change_timer = 0
        # Store current values as starting point
        self.pre_lane_change_values = {
            'path_angle': 0.0,  # Start from zero since we're coming out of lane change
            'path_offset': 0.0,
            'desired_curvature_rate': 0.0
        }

    # Update previous lane change state
    self.lane_change_last = self.lane_change

    # If we're in post-lane change state
    if self.post_lane_change_active:
        self.post_lane_change_timer += 1

        # Apply smooth transition using rate limiting
        new_path_angle = clip(
            path_angle,
            self.pre_lane_change_values['path_angle'] - self.max_path_angle_change,
            self.pre_lane_change_values['path_angle'] + self.max_path_angle_change
        )

        new_path_offset = clip(
            path_offset,
            self.pre_lane_change_values['path_offset'] - self.max_path_offset_change,
            self.pre_lane_change_values['path_offset'] + self.max_path_offset_change
        )

        new_curvature_rate = clip(
            desired_curvature_rate,
            self.pre_lane_change_values['desired_curvature_rate'] - self.max_curvature_rate_change,
            self.pre_lane_change_values['desired_curvature_rate'] + self.max_curvature_rate_change
        )

        # Update stored values
        self.pre_lane_change_values = {
            'path_angle': new_path_angle,
            'path_offset': new_path_offset,
            'desired_curvature_rate': new_curvature_rate
        }

        # Exit transition state after 40 frames
        if self.post_lane_change_timer >= 160:
            self.post_lane_change_active = False

        return (new_path_angle, new_path_offset, new_curvature_rate)

    return (path_angle, path_offset, desired_curvature_rate)

  def calculate_lateral_uncertainty(self, requested_curvature, apply_curvature, max_curvature):
    max_curvature = np.clip(max_curvature, apply_curvature, self.curvature_max)  # ensure max_curvature is within reasonable bounds
    return float(requested_curvature / max_curvature)

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []
    self.sm.update(0)

    if self.sm.updated['modelV2']:
      self.model = self.sm["modelV2"]

    if self.sm.updated['liveParameters']:
      self.lp = self.sm["liveParameters"]

    if self.sm.updated['selfdriveState']:
      self.ss = self.sm['selfdriveState']

    if self.lp is not None:
      x = max(self.lp.stiffnessFactor, 0.1)
      sr = max(self.lp.steerRatio, 0.1)
      self.VM.update_params(x, sr)

    self._update_params()

    actuators = CC.actuators
    hud_control = CC.hudControl
    main_on = CS.out.cruiseState.available
    gasPressed = CS.out.gasPressed
    brakePressed = CS.out.brakePressed
    # if self.fordVariables is None:
      # act = actuators.as_builder()
      # self.fordVariables = act.fordVariables

    # Calculate steer_alert and fcw_alert
    steer_alert = False
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    # Compute the DM message values
    if self.send_driver_monitor_can_msg:
      # print(f'HudControl: {hud_control}')
      # print(f'tja_msg: {self.tja_msg} | tja_warn: {self.tja_warn}')
      if (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
        self.tja_msg, self.tja_warn, self.hands = compute_dm_msg_values(self.ss, hud_control, self.send_hands_free_cluster_msg, main_on, CS.out.cruiseState.standstill)
    else:
      steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
      if steer_alert:
        self.hands = 1
      else:
        self.hands = 0

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    # Intelligent Cruise Button Management (ICBM)
    icbm_can_sends, self.last_button_frame = IntelligentCruiseButtonManagementInterface.update(
     self, CC_SP, CS, self.packer, self.CAN, self.frame, self.last_button_frame
     )
    can_sends.extend(icbm_can_sends)

    ### lateral control ###

    apply_curvature = 0.0 # initialize apply_curvature
    desired_curvature_rate = 0.0 # initialize desired_curvature_rate
    path_offset = 0.0 # initialize path_offset
    path_angle = 0.0 # initialize path_angle
    reset_steering = 0 # initialize reset_steering
    ramp_type = 2 # initialize ramp_type

    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      if CC.latActive:
        self.precision_type = 1
        steeringPressed = CS.out.steeringPressed
        steeringAngleDeg_PV = CS.out.steeringAngleDeg

        # determine tuning profile
        if self.custom_profile == 1: # custom tuning profile
          self.pc_blend_ratio_low_C =  self.pc_blend_ratio_low_C_UI
          self.pc_blend_ratio_high_C =  self.pc_blend_ratio_high_C_UI
          self.LC_PID_gain = self.LC_PID_gain_UI
        else:
          self.pc_blend_ratio_low_C = self.pc_blend_ratio_low_C
          self.pc_blend_ratio_high_C = self.pc_blend_ratio_high_C
          self.LC_PID_gain = self.LC_PID_gain

        self.pc_blend_ratio_v = [self.pc_blend_ratio_low_C, self.pc_blend_ratio_high_C] # %-Predicted Curvature

        # calculate current curvature and model desired curvature
        current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)  # use canbus data to calculate current_curvature
        desired_curvature = actuators.curvature  # get desired curvature from model

        # extract predicted curvature from modelV2
        if self.model is not None and len(self.model.orientation.x) >= 17:
          # compute curvature from model predicted orientationRate, and blend with desired curvature based on max predicted curvature magnitude
          curvatures = np.array(self.model.orientationRate.z) / max(0.01, CS.out.vEgoRaw)
          predicted_curvature = interp(self.curvature_lookup_time, ModelConstants.T_IDXS, curvatures)
        else:
          predicted_curvature = 0.0

        # calculate blend ratio
        self.pc_blend_ratio = interp(abs(desired_curvature), self.pc_blend_ratio_bp, self.pc_blend_ratio_v)

        # equate requested_curvature to a blend of desired and predicted_curvature and apply curvature limits
        requested_curvature = (predicted_curvature * self.pc_blend_ratio) + (desired_curvature * (1 - self.pc_blend_ratio))

        # determine if a lane change is active
        if (self.model.meta.laneChangeState == 1 or self.model.meta.laneChangeState == 2 or self.model.meta.laneChangeState == 3):
            self.lane_change = True
        else:
            self.lane_change = False

        # determine lane_change_factor based on speed
        lane_change_factor = interp(CS.out.vEgoRaw, self.lane_change_factor_bp, [self.lane_change_factor_low, self.lane_change_factor_high])

        # if changing lanes, modify curvature to smooth out the lane change
        if self.lane_change and (self.model.meta.laneChangeDirection == 1): # if we are changing lanes to the left
          if requested_curvature < 0: # and the curvature is taking us to the left
              requested_curvature = requested_curvature * lane_change_factor # reduce the curvature to smooth out the lane change
          else:
              requested_curvature = requested_curvature # if we are moving back right to correct for over travel, do not reduce curvature

          self.precision_type = 0 # use comfort mode

        if self.lane_change and (self.model.meta.laneChangeDirection == 2): # if we are changing lanes to the right
          if requested_curvature > 0: # and the curvature is taking us to the right
              requested_curvature = requested_curvature * lane_change_factor # reduce the curvature to smooth out the lane change
          else:
              requested_curvature = requested_curvature # if we are moving back left to correct for over travel, do not reduce curvature

          self.precision_type = 0 # use comfort mode

        # Determine if a human is making a turn and trap the value
        # if a human turn is active, reset steering to prevent windup
        if steeringPressed and abs(steeringAngleDeg_PV) > 45:
          self.human_turn = True
        else:
          self.human_turn = False

        # Determine when to reset steering
        if ((self.human_turn) and self.enable_human_turn_detection) or (CS.out.vEgoRaw < 0.1):
          reset_steering = 1
        else:
          reset_steering = 0

        #if reset_steering is 1, set requested_curvature to 0
        if reset_steering == 1:
          requested_curvature = 0.0

        # apply curvature limits
        apply_curvature, max_curvature = apply_ford_curvature_limits(requested_curvature,
                                                                self.apply_curvature_last,
                                                                current_curvature,
                                                                CS.out.vEgoRaw,
                                                                0,
                                                                CC.latActive,
                                                                self.CP)

        # lateral uncertianty is needed for the torque bar on curvature vehicles.
        lateralUncertainty = self.calculate_lateral_uncertainty(requested_curvature, apply_curvature, max_curvature)

        #if reset_steering is 1, set apply_curvature to 0
        if reset_steering == 1:
          apply_curvature = 0.0
          self.post_reset_ramp_active = False  # Cancel any active ramp when resetting
        else:
          # Detect transition from reset to normal (reset_steering goes from 1 to 0)
          if self.reset_steering_last and not reset_steering:
            # Just came out of reset, start post-reset ramp
            self.post_reset_ramp_active = True
            self.apply_curvature_last = 0.0  # Reset to ensure clean ramp from 0

        # Post-reset ramp logic: gradually ramp from 0 to requested curvature to avoid tripping the safety limit code
        # Keep path_angle = 0 during ramp to maintain bypass in ford.h
        if self.post_reset_ramp_active:
          # Use rate limits to gradually ramp up from 0 towards requested_curvature
          # This prevents blocked messages when transitioning out of reset
          apply_curvature = apply_std_steer_angle_limits(requested_curvature, self.apply_curvature_last,
                                                         CS.out.vEgoRaw, 0, CC.latActive, CarControllerParams.ANGLE_LIMITS)

          # Check if we've ramped close enough to requested curvature (within 10% or 0.001, whichever is larger)
          curvature_error = abs(requested_curvature - apply_curvature)
          curvature_threshold = max(abs(requested_curvature) * 0.1, 0.001)

          if curvature_error < curvature_threshold:
            # Ramp complete, exit post-reset mode
            self.post_reset_ramp_active = False

        # Update reset_steering_last for next frame
        self.reset_steering_last = (reset_steering == 1)

        # compute curvature rate (which is really the derivative of curvature)
        self.curvature_rate_deque.append(predicted_curvature)
        if len(self.curvature_rate_deque) > 1:
          delta_t = (
            self.curvature_rate_delta_t if len(self.curvature_rate_deque) == self.curvature_rate_deque.maxlen else (len(self.curvature_rate_deque) - 1) * 0.05
          )
          desired_curvature_rate = (self.curvature_rate_deque[-1] - self.curvature_rate_deque[0]) / delta_t / max(0.01, CS.out.vEgoRaw)
        else:
          desired_curvature_rate = 0.0

        # calculate curvature rate PC factor
        curvature_rate_PC_factor = interp(abs(predicted_curvature), self.curvature_rate_PC_bp, self.curvature_rate_PC_v)
        desired_curvature_rate = desired_curvature_rate * curvature_rate_PC_factor

        # calcualte curvature rate speed factor
        curvature_rate_speed_factor = interp(CS.out.vEgoRaw, self.curvature_rate_speed_bp, self.curvature_rate_speed_v)
        desired_curvature_rate = desired_curvature_rate * curvature_rate_speed_factor

        # determine large curve factor
        large_curve_factor = interp(abs(requested_curvature), self.large_curve_factor_bp, self.large_curve_factor_v)

        # apply large curve factor to desired_curvature_rate
        desired_curvature_rate = desired_curvature_rate * large_curve_factor

        #no large curve factor in lane changes
        if self.lane_change:
          large_curve_factor = 1.0

        # if we are in a lane change, set the desired_curvature_rate to 0
        if self.lane_change:
          desired_curvature_rate = 0.0

        # get path offset from model.position.y
        path_offset_position = interp(self.path_offset_lookup_time, ModelConstants.T_IDXS, self.model.position.y)

        # now get path offset from lanelines
        path_offset_lanelines = (self.model.laneLines[1].y[0] + self.model.laneLines[2].y[0]) / 2

        # determinie laneline width tolerance scaling factor (this is to prevent the vehicle from jumping when two lanes start to merge or diverge)
        laneline_width = self.model.laneLines[2].y[0] + (-self.model.laneLines[1].y[0]) # laneLines[1] is a negative value because it is left of the vehicle.
        laneline_width_tolerance = interp(laneline_width, [3.75,4.25], [0.81, 0.59]) # 3.7 is the width of standard US lane in meters

        # determine laneline confidence
        laneline_confidence = min(self.model.laneLineProbs[1], self.model.laneLineProbs[2], laneline_width_tolerance)
        if not self.enable_lanefull_mode: # if lanefull mode is off, a 0 confidence will make the lane lines ignored.
          laneline_confidence = 0.0

        # determine laneline path offset scale
        laneline_path_offset_scale = interp(laneline_confidence, self.min_laneline_confidence_bp, [0.0, 1.0]) # this interp basically sets how much influence the lane lines have.

        # get the total path_offset combining model and lanelines by blending them based on confidence level.
        path_offset = (path_offset_position * (1-laneline_path_offset_scale) + (path_offset_lanelines * laneline_path_offset_scale)) + self.custom_path_offset

        # no path_offset during lane changes (it will fight you until it swaps to new lane if you don't set to zero)
        if self.lane_change:
          path_offset = 0

        # Use the UI variable for adjustable Gain because the PID gain is set to a fixed number, UI variable divided by 100 to make UI variable an easier to adjust number
        path_offset_error = (path_offset * (self.LC_PID_gain_UI/100))

        # Begin path_angle logic.  In this situation, path_angle is being used to drive our offset to zero.
        # determine speed factor (less PID action needed at higher speeds)
        LC_PID_speed_factor = interp(CS.out.vEgoRaw, self.LC_PID_speed_bp, self.LC_PID_speed_v)

        # apply speed factor to path_offset_error
        path_offset_error_adj = path_offset_error * LC_PID_speed_factor

        # if not using lane positioning, zero out path_offset_error_adj
        if not self.enable_lane_positioning:
          path_offset_error_adj = 0.0

        # Use path_angle to help with centering vehicle in lane, Use PID controller to calculate path_angle
        path_angle_low_c = self.LC_PID_controller.update(path_offset_error_adj)

        # if not using lane positioning, zero out path_angle_low_c (should be zeroed out in the PID controller, but just in case)
        if not self.enable_lane_positioning:
          path_angle_low_c = 0.0

        # reset path angle if steering reset is active
        # During post-reset ramp, path_angle can ramp normally (latch in ford.h handles bypass)
        if reset_steering == 1:
          path_angle_low_c = 0.0

        # rate limit path_angle_low_c for comfort
        path_angle_roc = interp(abs(CS.out.vEgoRaw), self.LC_path_angle_ROC_bp, self.LC_path_angle_ROC_v)
        path_angle_low_c = clip(path_angle_low_c, self.path_angle_last - path_angle_roc, self.path_angle_last + path_angle_roc)

        # if the driver is applying consistent pressure to the steering wheel, reset the path_angle_low_c PID controller
        if steeringPressed:
          self.LC_path_angle_reset_counter = self.LC_path_angle_reset_counter + 1
        else:
          self.LC_path_angle_reset_counter = 0
        if self.LC_path_angle_reset_counter > self.LC_path_angle_reset_duration * 20: #20 scans per second
          self.LC_PID_controller.reset()

        # path_angle_high_c is not used in the current implementation
        path_angle_high_c = 0.0

        # sum path_angle_low_c and path_angle_high_c
        path_angle = path_angle_low_c + path_angle_high_c

        # Apply post lane change transition logic
        path_angle, path_offset, desired_curvature_rate = self.handle_post_lane_change_transition(
            path_angle, path_offset, desired_curvature_rate
        )

        # reset path angle if steering reset is active
        # During post-reset ramp, path_angle can ramp normally (latch in ford.h handles bypass)
        if reset_steering == 1:
          path_angle = 0.0

        # clip all values to max.
        apply_curvature = clip(apply_curvature, -self.curvature_max, self.curvature_max)
        desired_curvature_rate = clip(desired_curvature_rate, -self.curvature_rate_max, self.curvature_rate_max)
        path_offset = clip(path_offset, -self.path_offset_max, self.path_offset_max)
        path_angle = clip(path_angle, -self.path_angle_max, self.path_angle_max)


        # if path_offset and path_angle disagree, it can result in a very uncomortable ride, since path_angle is so strong, zero out path_offset signal before it is sent over canbus
        path_offset = 0.0

        if self.disable_BP_lat_UI:
          reset_steering = 0
          path_offset = 0
          path_angle = 0
          desired_curvature_rate = 0
          ramp_type = 1

          self.anti_overshoot_curvature_last = anti_overshoot(desired_curvature, self.anti_overshoot_curvature_last, CS.out.vEgoRaw)
          apply_curvature = self.anti_overshoot_curvature_last

          current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)

          self.apply_curvature_last, max_curvature = apply_ford_curvature_limits(apply_curvature, self.apply_curvature_last, current_curvature,
                                                              CS.out.vEgoRaw, 0., CC.latActive, self.CP)

          #rem bluepilot sends apply_curvature, and at some point openpilot swapped to sending apply_curvature_last.
          apply_curvature = self.apply_curvature_last

          lateralUncertainty = self.calculate_lateral_uncertainty(requested_curvature, apply_curvature, max_curvature)

        # reset steering by setting all values to 0 (handle where each variable is calcualted) and ramp_type to immediate.  Also clear filters and PID controllers
        if reset_steering == 1:
          ramp_type = 3
          self.path_angle_deque.clear()
          self.LC_PID_controller.reset()
        else:
          ramp_type = 2 # ramp_type is fast for non-reset situations.
      else: # if lateral control is off, zero everything.
        apply_curvature = 0.0
        desired_curvature_rate = 0.0
        path_offset = 0.0
        path_angle = 0.0
        self.path_angle_deque.clear()
        self.LC_PID_controller.reset()
        ramp_type = 0
        lateralUncertainty = 0.0

      self.lateralUncertainty = lateralUncertainty
      self.apply_curvature_last = apply_curvature
      self.curvature_rate_last = desired_curvature_rate
      self.path_offset_last = path_offset
      self.path_angle_last = path_angle


      # set lat_active to the value of CC.latActive
      lat_active = CC.latActive

      if self.CP.flags & FordFlags.CANFD:
        # TODO: extended mode
        # Ford uses four individual signals to dictate how to drive to the car. Curvature alone (limited to 0.02m/s^2)
        # can actuate the steering for a large portion of any lateral movements. However, in order to get further control on
        # steer actuation, the other three signals are necessary. Ford controls vehicles differently than most other makes.
        # A detailed explanation on ford control can be found here:
        # https://www.f150gen14.com/forum/threads/introducing-bluepilot-a-ford-specific-fork-for-comma3x-openpilot.24241/#post-457706
        mode = 1 if lat_active else 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        can_sends.append(fordcan.create_lat_ctl2_msg(
          self.packer, self.CAN, mode, ramp_type, self.precision_type, -path_offset, -path_angle,
          -apply_curvature, -desired_curvature_rate, counter
        ))
      else:
        # Ford non-CANFD lateral control
        can_sends.append(fordcan.create_lat_ctl_msg(
          self.packer, self.CAN, lat_active, ramp_type, self.precision_type,
          -path_offset, -path_angle, -apply_curvature, -desired_curvature_rate
        ))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      lka_hud_control = None
      if self.send_lane_depart_can_msg:
        lka_hud_control = hud_control
      can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN, CC.latActive, lka_hud_control))

    ### longitudinal control ###
    # openpilot variable names can be confusing when looking at ford control.
    # accel is the analog signal to the brakes is m/s2
    # gas is the analog signal to the accelerator in m/s2
    # brake_actuate is the signal to actuall press the brakes (negative accel without brake_acutate results in engine braking)
    # For hybrids/EV the ford PCM determines when to use brake pedal versus regen, there is no way for openpilot to affect this.
    # send acc msg at 50Hz
    v_ego_mph = CS.out.vEgo * 2.23694  # m/s to mph

    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      # First calcualte the stock logic's accel, gas, and brake request
      op_accel = actuators.accel
      op_gas = op_accel

      if CC.longActive:
        # Compensate for engine creep at low speed.
        # Either the ABS does not account for engine creep, or the correction is very slow
        # TODO: verify this applies to EV/hybrid
        creep_accel = interp(CS.out.vEgo, [1., 3.], [0.6, 0.])
        creep_accel = interp(op_accel, [0., 0.2], [creep_accel, 0.])
        op_accel -= creep_accel

        # The stock system has been seen rate limiting the brake accel to 5 m/s^3,
        # however even 3.5 m/s^3 causes some overshoot with a step response.
        op_accel = max(op_accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))

      op_accel = float(np.clip(op_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      op_gas = float(np.clip(op_gas, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      # Both gas and accel are in m/s^2, accel is used solely for braking
      if not CC.longActive or op_gas < CarControllerParams.MIN_GAS:
        op_gas = CarControllerParams.INACTIVE_GAS # this is a quirk in the ford PCM, if you are not using gas, it has to be set to -5.0 m/s2 or you will get a cruise fault

      # PCM applies pitch compensation to gas/accel, but we need to compensate for the brake/pre-charge bits
      accel_due_to_pitch = 0.0
      if len(CC.orientationNED) == 3:
        accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY

      # some ford vehicles have downhill compensation built in, adding in accel_due_to_pitch is double dipping, creating harsh braking downhill
      if self.disable_downhill_comp_UI:
          if accel_due_to_pitch < 0:
              accel_due_to_pitch = 0

      accel_pitch_compensated = op_accel + accel_due_to_pitch
      stopping = CC.actuators.longControlState == LongCtrlState.stopping
      op_brake_actuate = self.op_brake_actuate_last
      if accel_pitch_compensated < self.brake_actuate_target:
        op_brake_actuate = True
      if accel_pitch_compensated > self.brake_actuate_release or not CC.longActive:
        op_brake_actuate = False
      # target_speed = float(np.clip(actuators.speed * self.target_speed_multiplier, 0, V_CRUISE_MAX))

      # if not CC.longActive and getattr(hud_control, "setSpeed", None) is not None:
        # target_speed = hud_control.setSpeed
      target_speed = V_CRUISE_MAX

      # TODO return to this signal later, it might help with highway control, but sending values ford doesn't like causes ACC to cancel.
      self.accel_pred = -5.0  # same as BluePilot branch until safe logic is confirmed

      # Speed deadband for BP long: engage above 50 mph, disallow below 45 mph; 45–50 keeps current state to avoid oscillation.
      bpSpeedTooSlow = v_ego_mph < self.MAX_URBAN_SPEED_MPH
      bpSpeedHighEnough = v_ego_mph > self.MAX_URBAN_SPEED_MPH + 5
      if bpSpeedHighEnough:
        self.bpSpeedAllow = True
      if bpSpeedTooSlow:
        self.bpSpeedAllow = False

      # BluePilot longitudinal: gas limits when following + rate-limited accel/brake to avoid stomping.
      if not self.disable_BP_long_UI:

        # Lead time (s) and lead state. leadOne.status: 0 = no lead, 1 = lead.
        v_ego = max(CS.out.vEgo, 0.5)
        lead_time_sec = 999.0  # no lead: treat as far
        lead = None
        v_rel = 0.0
        v_lead = 0.0
        if self.sm.valid.get('radarState', False):
          rs = self.sm['radarState']
          lead = getattr(rs, 'leadOne', None)
          if lead is not None and getattr(lead, 'status', 0) != 1:
            lead = None
          if lead:
            d_rel = float(getattr(lead, 'dRel', 0))
            v_rel = float(getattr(lead, 'vRel', 0))
            v_lead = float(getattr(lead, 'vLead', 0))  # m/s; schema is vLead (camelCase)
            if d_rel > 0:
              lead_time_sec = d_rel / v_ego
        lead_time_sec = float(np.clip(lead_time_sec, 0.0, 999.0))
        v_lead_mph = v_lead * 2.23694  # for apply_bp_long: only optimize when lead > 40 mph (don't coast into traffic jam)

        ttc_sec = 120.0
        if self.sm.valid.get('radarState', False):
          rs = self.sm['radarState']
          lead = getattr(rs, 'leadOne', None)
          if lead is not None and getattr(lead, 'status', 0) != 1:
            lead = None
          if lead:
            d_rel = float(getattr(lead, 'dRel', 0))
            v_rel = float(getattr(lead, 'vRel', 0))
            if d_rel > 0 and v_rel < 0:
              ttc_sec = d_rel / (-v_rel)
            else:
              ttc_sec = 60.0
        ttc_sec = float(np.clip(ttc_sec, 0.2, 120.0))

        # Defaults: pass through op_* when no lead or no mode; brake/precharge off until thresholds
        gaining = False
        pacing = False
        trailing = False
        max_follow_gas = op_gas
        min_follow_gas = op_gas
        max_follow_accel = op_accel
        min_follow_accel = op_accel
        bp_brake_actuate = False
        bp_precharge_actuate = False

        # Gaining on lead, pacing, or trailing away
        if lead:
          if v_rel < -0.1:
            gaining = True
          elif v_rel > 0.1:
            trailing = True
          else:
            pacing = True

        # limits when gaining
        if gaining:
          if lead_time_sec < 1.5:
              max_follow_gas = 0.0 # if we are within 1.5 seconds and gaining, why press the gas?
              min_follow_gas = 0.0
          else:
             max_follow_gas = op_gas
             min_follow_gas = op_gas
          max_follow_accel = op_accel
          min_follow_accel = op_accel # following braking for our primary target


        # limits when pacing
        if pacing:
          max_follow_gas = 0.2 + accel_due_to_pitch # don't get too happy with the gas when pacing.
          min_follow_gas = 0.0
          max_follow_accel = op_accel
          min_follow_accel = op_accel # we will always target op_accel for braking

        # limits when trailing
        if trailing:
          max_follow_gas = op_gas
          min_follow_gas = op_gas
          max_follow_accel = op_accel
          min_follow_accel = op_accel

        # limits with no lead
        if lead is None:
          max_follow_gas = op_gas
          min_follow_gas = op_gas
          max_follow_accel = 0
          min_follow_accel = 0


        # apply our bp gas and accel targets
        bp_gas = clip(op_gas, min_follow_gas, max_follow_gas)
        bp_accel = clip(op_accel, min_follow_accel, max_follow_accel)

        # now let's apply some rate limits, not much, just try to dampen the initial hit when braking
        # but only apply the limits if there is no imminent chance of a collision
        if ttc_sec > 8.0 and lead_time_sec > 0.5:
          bp_accel = clip(bp_accel, self.bp_accel_last - self.following_accel_ROC, 999) #only limit the downward change of braking

        # Set brake_actuate and precharge_actuate flags (initialized False above)
        if bp_accel < self.brake_actuate_target:
          bp_brake_actuate = True
        if bp_accel > self.brake_actuate_release:
          bp_brake_actuate = False
        if bp_accel < self.precharge_actuate_target:
          bp_precharge_actuate = True
        if bp_accel > self.precharge_actuate_release:
          bp_precharge_actuate = False

        # Determine if we will use bp smoothing (bpSpeedAllow deadband updated above, outside this block)
        gasPressed = CS.out.gasPressed
        brakePressed = CS.out.brakePressed

        # When we have a lead, require lead speed > 40 mph so we don't coast into a traffic jam; when no lead, allow BP long
        apply_bp_long = (self.disable_BP_long_UI == False) and (self.bpSpeedAllow) and (gasPressed == False) and (brakePressed == False) and (lead is None or v_lead_mph > 40.0)

        if apply_bp_long and CC.longActive:
          accel = bp_accel
          gas = bp_gas
          brake_actuate = bp_brake_actuate
          precharge_actuate = bp_precharge_actuate
        else:
          accel = op_accel
          gas = op_gas
          brake_actuate = op_brake_actuate
          precharge_actuate = op_brake_actuate

        self.bp_gas_last = bp_gas
        self.bp_accel_last = bp_accel
        bp_long_used = apply_bp_long
      else:
        accel = op_accel
        gas = op_gas
        brake_actuate = op_brake_actuate
        precharge_actuate = op_brake_actuate
        bp_long_used = False

      # no brake and gas at the same timne
      if brake_actuate:
        gas = CarControllerParams.INACTIVE_GAS

      # Clip to ford.h ACCDATA safety limits
      accel = float(clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      if gas != CarControllerParams.INACTIVE_GAS:
        gas = float(clip(gas, CarControllerParams.MIN_GAS, CarControllerParams.ACCEL_MAX))
      accel_pred_send = CarControllerParams.INACTIVE_GAS
      stop_request = stopping
      resume_enable = CC.longActive

      accel, gas, brake_actuate, precharge_actuate, stop_request, resume_enable, target_speed = self._apply_ford_stock_acc_stop_go_v12(
        CC, CS, accel, gas, brake_actuate, precharge_actuate, stopping, target_speed, now_nanos
      )

      can_sends.append(fordcan.create_acc_msg(
        self.packer, self.CAN, CC.longActive, gas, accel, accel_pred_send, stop_request,
        brake_actuate, precharge_actuate, v_ego_kph=target_speed, resume_enable=resume_enable
      ))

      self.accel = accel
      self.gas = gas
      self._bp_long_active_last = bp_long_used
      self.op_brake_actuate_last = op_brake_actuate

    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    # send lkas ui msg at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, self.hands, hud_control, CS.lkas_status_stock_values))

    # send acc ui msg at 5Hz or if ui state changes
    send_bars = False
    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True
      send_bars = True

    # Logic to keep sending the bars for 4 seconds
    if not self.send_bars_last and send_bars:
      # Save the frame # for the last flip from False to True
      self.send_bars_ts_last = self.frame
      self.distance_bar_frame = self.frame

    # keep sending the bars for 4 seconds (400 at 100Hz)
    if (self.send_bars_ts_last > 0 and (self.frame - self.send_bars_ts_last) <= 400):
      send_ui = True
      send_bars = True

    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      can_sends.append(
        fordcan.create_acc_ui_msg(
          self.packer,
          self.CAN,
          self.CP,
          main_on,
          CC.latActive,
          fcw_alert,
          CS.out.cruiseState.standstill,
          hud_control,
          CS.acc_tja_status_stock_values,
          self.send_hands_free_cluster_msg,
          send_ui,
          send_bars,
          self.tja_warn,
          self.tja_msg,
        )
      )

    self.main_on_last = main_on
    self.send_ui_last = send_ui
    self.send_bars_last = send_bars
    self.lkas_enabled_last = CC.latActive
    self.fcw_alert_last = fcw_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.as_builder()
    new_actuators.curvature = float(apply_curvature)
    new_actuators.accel = float(self.accel)
    new_actuators.gas = float(self.gas)
    self.frame += 1
    return new_actuators, can_sends
