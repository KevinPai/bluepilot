#!/usr/bin/env python3
import json
import os
import time
import threading

import cereal.messaging as messaging

from cereal import car, log, custom

from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog, ForwardingHandler

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData, CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.fw_versions import ObdCallback
from opendbc.car.car_helpers import get_car, interfaces
from opendbc.car.interfaces import CarInterfaceBase, RadarInterfaceBase
from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.selfdrive.car.helpers import convert_carControlSP, convert_to_capnp

from openpilot.sunnypilot.mads.helpers import set_alternative_experience, set_car_specific_params
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces

REPLAY = "REPLAY" in os.environ

EventName = log.OnroadEvent.EventName

# forward
carlog.addHandler(ForwardingHandler(cloudlog))


def obd_callback(params: Params) -> ObdCallback:
  def set_obd_multiplexing(obd_multiplexing: bool):
    if params.get_bool("ObdMultiplexingEnabled") != obd_multiplexing:
      cloudlog.warning(f"Setting OBD multiplexing to {obd_multiplexing}")
      params.remove("ObdMultiplexingChanged")
      params.put_bool("ObdMultiplexingEnabled", obd_multiplexing)
      params.get_bool("ObdMultiplexingChanged", block=True)
      cloudlog.warning("OBD multiplexing set successfully")
  return set_obd_multiplexing


def can_comm_callbacks(logcan: messaging.SubSocket, sendcan: messaging.PubSocket) -> tuple[CanRecvCallable, CanSendCallable]:
  def can_recv(wait_for_one: bool = False) -> list[list[CanData]]:
    """
    wait_for_one: wait the normal logcan socket timeout for a CAN packet, may return empty list if nothing comes

    Returns: CAN packets comprised of CanData objects for easy access
    """
    ret = []
    for can in messaging.drain_sock(logcan, wait_for_one=wait_for_one):
      ret.append([CanData(msg.address, msg.dat, msg.src) for msg in can.can])
    return ret

  def can_send(msgs: list[CanData]) -> None:
    sendcan.send(can_list_to_can_capnp(msgs, msgtype='sendcan'))

  return can_recv, can_send


class Car:
  CI: CarInterfaceBase
  RI: RadarInterfaceBase
  CP: car.CarParams
  CP_SP: structs.CarParamsSP
  CP_SP_capnp: custom.CarParamsSP

  def __init__(self, CI=None, RI=None) -> None:
    self.can_sock = messaging.sub_sock('can', timeout=20)
    self.sm = messaging.SubMaster(['pandaStates', 'carControl', 'onroadEvents', 'longitudinalPlan', 'radarState'] + ['carControlSP', 'longitudinalPlanSP'])
    self.pm = messaging.PubMaster(['sendcan', 'carState', 'carParams', 'carOutput', 'liveTracks'] + ['carParamsSP', 'carStateSP', 'controllerStateBP', 'carStateBP'])

    self.can_rcv_cum_timeout_counter = 0

    self.CC_prev = car.CarControl.new_message()
    self.CS_prev = car.CarState.new_message()
    self.CS_SP_prev = custom.CarStateSP.new_message()
    self.initialized_prev = False

    self.last_actuators_output = structs.CarControl.Actuators()
    self.ford_observe_last_log_t = 0.0
    self.ford_observe_prev_lead_valid = False
    self.ford_observe_prev_d_rel = 0.0
    self.ford_observe_prev_v_rel = 0.0
    self.ford_observe_lead_stable_since = 0.0
    self.ford_observe_source_key = None
    self.ford_observe_source_transition_t = 0.0
    self.ford_observe_resume_lead_move_t = 0.0
    self.controller_state_bp_last_send_t = 0.0

    self.params = Params()

    self.can_callbacks = can_comm_callbacks(self.can_sock, self.pm.sock['sendcan'])

    is_release = self.params.get_bool("IsReleaseBranch")
    is_release_sp = self.params.get_bool("IsReleaseSpBranch")

    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      while True:
        can = messaging.recv_one_retry(self.can_sock)
        if len(can.can) > 0:
          break

      alpha_long_allowed = self.params.get_bool("AlphaLongitudinalEnabled")
      num_pandas = len(messaging.recv_one_retry(self.sm.sock['pandaStates']).pandaStates)

      cached_params = None
      cached_params_raw = self.params.get("CarParamsCache")
      if cached_params_raw is not None:
        with car.CarParams.from_bytes(cached_params_raw) as _cached_params:
          cached_params = _cached_params

      fixed_fingerprint = (self.params.get("CarPlatformBundle") or {}).get("platform", None)
      init_params_list_sp = sunnypilot_interfaces.initialize_params(self.params)

      self.CI = get_car(*self.can_callbacks, obd_callback(self.params), alpha_long_allowed, is_release, num_pandas, cached_params,
                        fixed_fingerprint, init_params_list_sp, is_release_sp)
      sunnypilot_interfaces.setup_interfaces(self.CI, self.params)
      self.RI = interfaces[self.CI.CP.carFingerprint].RadarInterface(self.CI.CP, self.CI.CP_SP)
      self.CP = self.CI.CP
      self.CP_SP = self.CI.CP_SP

      # continue onto next fingerprinting step in pandad
      self.params.put_bool("FirmwareQueryDone", True)
    else:
      self.CI, self.CP, self.CP_SP = CI, CI.CP, CI.CP_SP
      self.RI = RI

    self.CP.alternativeExperience = 0
    # mads
    set_alternative_experience(self.CP, self.CP_SP, self.params)
    set_car_specific_params(self.CP, self.CP_SP, self.params)

    # Dynamic Experimental Control
    self.dynamic_experimental_control = self.params.get_bool("DynamicExperimentalControl")

    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")
    controller_available = self.CI.CC is not None and openpilot_enabled_toggle and not self.CP.dashcamOnly
    self.CP.passive = not controller_available or self.CP.dashcamOnly
    if self.CP.passive:
      safety_config = structs.CarParams.SafetyConfig()
      safety_config.safetyModel = structs.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    if self.CP.secOcRequired:
      # Copy user key if available
      try:
        with open("/cache/params/SecOCKey") as f:
          user_key = f.readline().strip()
          if len(user_key) == 32:
            self.params.put("SecOCKey", user_key)
      except Exception:
        pass

      secoc_key = self.params.get("SecOCKey")
      if secoc_key is not None:
        saved_secoc_key = bytes.fromhex(secoc_key.strip())
        if len(saved_secoc_key) == 16:
          self.CP.secOcKeyAvailable = True
          self.CI.CS.secoc_key = saved_secoc_key
          if controller_available:
            self.CI.CC.secoc_key = saved_secoc_key
        else:
          cloudlog.warning("Saved SecOC key is invalid")

    # Write previous route's CarParams
    prev_cp = self.params.get("CarParamsPersistent")
    if prev_cp is not None:
      self.params.put("CarParamsPrevRoute", prev_cp)

    # Write CarParams for controls and radard
    cp_bytes = self.CP.to_bytes()
    self.params.put("CarParams", cp_bytes)
    self.params.put_nonblocking("CarParamsCache", cp_bytes)
    self.params.put_nonblocking("CarParamsPersistent", cp_bytes)

    # Write CarParamsSP for controls
    # convert to pycapnp representation for caching and logging
    self.CP_SP_capnp = convert_to_capnp(self.CP_SP)
    cp_sp_bytes = self.CP_SP_capnp.to_bytes()
    self.params.put("CarParamsSP", cp_sp_bytes)
    self.params.put_nonblocking("CarParamsSPCache", cp_sp_bytes)
    self.params.put_nonblocking("CarParamsSPPersistent", cp_sp_bytes)

    self.v_cruise_helper = VCruiseHelper(self.CP, self.CP_SP)

    self.is_metric = self.params.get_bool("IsMetric")
    self.experimental_mode = self.params.get_bool("ExperimentalMode")

    # card is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)

    # log fingerprint in sentry
    sunnypilot_interfaces.log_fingerprint(self.CP)

  def state_update(self) -> tuple[car.CarState, custom.CarStateSP, structs.RadarDataT | None]:
    """carState update loop, driven by can"""

    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    can_list = can_capnp_to_list(can_strs)

    # Update carState from CAN
    CS, CS_SP = self.CI.update(can_list)
    CS_SP = convert_to_capnp(CS_SP)

    # Update radar tracks from CAN
    RD: structs.RadarDataT | None = self.RI.update(can_list)

    self.sm.update(0)

    can_rcv_valid = len(can_strs) > 0

    # Check for CAN timeout
    if not can_rcv_valid:
      self.can_rcv_cum_timeout_counter += 1

    if can_rcv_valid and REPLAY:
      self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

    self.v_cruise_helper.update_speed_limit_assist(self.is_metric, self.sm['longitudinalPlanSP'])
    self.v_cruise_helper.update_v_cruise(CS, self.sm['carControl'].enabled, self.is_metric)
    if self.sm['carControl'].enabled and not self.CC_prev.enabled:
      # Use CarState w/ buttons from the step selfdrived enables on
      self.v_cruise_helper.initialize_v_cruise(self.CS_prev, self.experimental_mode, self.dynamic_experimental_control)

    # TODO: mirror the carState.cruiseState struct?
    CS.vCruise = float(self.v_cruise_helper.v_cruise_kph)
    CS.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph)

    return CS, CS_SP, RD

  def state_publish(self, CS: car.CarState, CS_SP: custom.CarStateSP, RD: structs.RadarDataT | None):
    """carState and carParams publish loop"""

    # carParams - logged every 50 seconds (> 1 per segment)
    if self.sm.frame % int(50. / DT_CTRL) == 0:
      cp_send = messaging.new_message('carParams')
      cp_send.valid = True
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # publish new carOutput
    co_send = messaging.new_message('carOutput')
    co_send.valid = self.sm.all_checks(['carControl'])
    co_send.carOutput.actuatorsOutput = self.last_actuators_output
    self.pm.send('carOutput', co_send)

    # kick off controlsd step while we actuate the latest carControl packet
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.canErrorCounter = self.can_rcv_cum_timeout_counter
    cs_send.carState.cumLagMs = -self.rk.remaining * 1000.
    self.pm.send('carState', cs_send)

    if RD is not None:
      tracks_msg = messaging.new_message('liveTracks')
      tracks_msg.valid = not any(RD.errors.to_dict().values())
      tracks_msg.liveTracks = RD
      self.pm.send('liveTracks', tracks_msg)

    # carParamsSP - logged every 50 seconds (> 1 per segment)
    if self.sm.frame % int(50. / DT_CTRL) == 0:
      cp_sp_send = messaging.new_message('carParamsSP')
      cp_sp_send.valid = True
      cp_sp_send.carParamsSP = self.CP_SP_capnp
      self.pm.send('carParamsSP', cp_sp_send)

    cs_sp_send = messaging.new_message('carStateSP')
    cs_sp_send.valid = CS.canValid
    cs_sp_send.carStateSP = CS_SP
    self.pm.send('carStateSP', cs_sp_send)

    # carStateBP - hybrid drive gauge data
    if hasattr(self.CI.CS, 'car_state_bp_msg') and self.CI.CS.car_state_bp_msg is not None:
      cs_bp_send = self.CI.CS.car_state_bp_msg
      cs_bp_send.valid = CS.canValid
      self.pm.send('carStateBP', cs_bp_send)

  def controls_update(self, CS: car.CarState, CC: car.CarControl, CC_SP: custom.CarControlSP):
    """control update loop, driven by carControl"""

    if not self.initialized_prev:
      # Initialize CarInterface, once controls are ready
      # TODO: this can make us miss at least a few cycles when doing an ECU knockout
      self.CI.init(self.CP, self.CP_SP, *self.can_callbacks)
      # signal pandad to switch to car safety mode
      self.params.put_bool_nonblocking("ControlsReady", True)

    if self.sm.all_alive(['carControl']):
      # send car controls over can
      now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
      self.last_actuators_output, can_sends = self.CI.apply(CC, convert_carControlSP(CC_SP), now_nanos)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))

      self.CC_prev = CC
      self.log_ford_long_observe(CS, CC)

    if hasattr(self.CI.CC, "lateralUncertainty"):
      cc_obj = self.CI.CC
      controller_state_bp_now = time.monotonic()
      low_speed_debug = bool(CC.longActive and (CS.vEgo < 8.0 or CS.standstill or CS.gasPressed or CS.brakePressed))
      soft_crawl_debug = bool(getattr(cc_obj, "ford_soft_crawl_last_available", False) or
                              getattr(cc_obj, "ford_soft_crawl_last_control_active", False))
      controller_state_bp_interval = 0.5 if (low_speed_debug or soft_crawl_debug) else 2.0
      if controller_state_bp_now - self.controller_state_bp_last_send_t < controller_state_bp_interval:
        return
      self.controller_state_bp_last_send_t = controller_state_bp_now

      cs_bp = structs.ControllerStateBP()
      cs_bp.lateralUncertainty = cc_obj.lateralUncertainty
      cs_bp.stockGoLeadMoved = bool(getattr(cc_obj, "ford_v14_last_lead_moved", False))
      cs_bp.stockGoLeadStable = bool(getattr(cc_obj, "ford_v14_last_lead_stable", False))
      cs_bp.stockGoLeadStableAge = float(getattr(cc_obj, "ford_v14_last_lead_stable_age", 0.0))
      cs_bp.stockGoTimeSinceLeadMove = float(getattr(cc_obj, "ford_v14_last_time_since_lead_move", 0.0))
      cs_bp.stockGoEgoLag = float(getattr(cc_obj, "ford_v14_last_ego_lag", 0.0))
      cs_bp.stockGoCandidate = bool(getattr(cc_obj, "ford_v14_last_candidate", False))
      cs_bp.stockGoBlockedReason = getattr(cc_obj, "ford_v14_last_blocked_reason", "")
      cs_bp.stockGoDesiredAccel = float(getattr(cc_obj, "ford_v14_last_desired_accel", 0.0))
      cs_bp.stockGoJerkLimitedAccel = float(getattr(cc_obj, "ford_v14_last_jerk_limited_accel", 0.0))
      cs_bp.stockGoReleasePhase = getattr(cc_obj, "ford_v14_last_release_phase", "")
      cs_bp.stockGoControlEnabled = bool(getattr(cc_obj, "ford_v14_last_control_enabled", False))
      cs_bp.softCrawlObserveEnabled = bool(getattr(cc_obj, "ford_stock_acc_soft_crawl_observe", False))
      cs_bp.softCrawlControlEnabled = bool(getattr(cc_obj, "ford_stock_acc_soft_crawl_control", False) and
                                           (getattr(cc_obj, "ford_stock_acc_soft_crawl_v16", False) or
                                            getattr(cc_obj, "ford_stock_acc_soft_crawl_v17", False) or
                                            getattr(cc_obj, "ford_stock_acc_soft_crawl_v171", False)))
      cs_bp.softCrawlAvailable = bool(getattr(cc_obj, "ford_soft_crawl_last_available", False))
      cs_bp.softCrawlReason = getattr(cc_obj, "ford_soft_crawl_last_reason", "")
      cs_bp.softCrawlFallbackReason = getattr(cc_obj, "ford_soft_crawl_last_fallback_reason", "")
      cs_bp.softCrawlTargetAccel = float(getattr(cc_obj, "ford_soft_crawl_last_target_accel", 0.0))
      cs_bp.softCrawlDistanceToStop = float(getattr(cc_obj, "ford_soft_crawl_last_distance_to_stop", 0.0))
      cs_bp.softCrawlNeededDistance = float(getattr(cc_obj, "ford_soft_crawl_last_needed_distance", 0.0))
      cs_bp.softCrawlDistanceMargin = float(getattr(cc_obj, "ford_soft_crawl_last_distance_margin", 0.0))
      cs_bp.softCrawlCurrentStopDistance = float(getattr(cc_obj, "ford_soft_crawl_last_current_stop_distance", 0.0))
      cs_bp.softCrawlLeadDRel = float(getattr(cc_obj, "ford_soft_crawl_last_lead_d_rel", 0.0))
      cs_bp.softCrawlLeadVRel = float(getattr(cc_obj, "ford_soft_crawl_last_lead_v_rel", 0.0))
      cs_bp.softCrawlLeadVLead = float(getattr(cc_obj, "ford_soft_crawl_last_lead_v_lead", 0.0))
      cs_bp.softCrawlTtc = float(getattr(cc_obj, "ford_soft_crawl_last_ttc", 0.0))
      cs_bp.softCrawlVEgo = float(getattr(cc_obj, "ford_soft_crawl_last_v_ego", 0.0))
      cs_bp.softCrawlOriginalAccel = float(getattr(cc_obj, "ford_soft_crawl_last_original_accel", 0.0))
      cs_bp.softCrawlPlannerStopping = bool(getattr(cc_obj, "ford_soft_crawl_last_planner_stopping", False))
      cs_bp.softCrawlControlActive = bool(getattr(cc_obj, "ford_soft_crawl_last_control_active", False))
      cs_bp.softCrawlPhase = getattr(cc_obj, "ford_soft_crawl_last_phase", "")
      cs_bp.softCrawlStopGapTarget = float(getattr(cc_obj, "ford_soft_crawl_last_stop_gap_target", 0.0))
      cs_bp.softCrawlRawTargetAccel = float(getattr(cc_obj, "ford_soft_crawl_last_raw_target_accel", 0.0))
      cs_bp.softCrawlJerkLimitedAccel = float(getattr(cc_obj, "ford_soft_crawl_last_jerk_limited_accel", 0.0))
      cs_bp.softCrawlAccelAfterControl = float(getattr(cc_obj, "ford_soft_crawl_last_accel_after_control", 0.0))
      cs_bp.softCrawlV16Enabled = bool(getattr(cc_obj, "ford_stock_acc_soft_crawl_v16", False) or
                                       getattr(cc_obj, "ford_stock_acc_soft_crawl_v17", False) or
                                       getattr(cc_obj, "ford_stock_acc_soft_crawl_v171", False) or
                                       getattr(cc_obj, "ford_stock_acc_soft_crawl_v172", False))
      cs_bp.softCrawlControlStage = getattr(cc_obj, "ford_soft_crawl_last_control_stage", "")
      cs_bp.softCrawlTimeToStopEst = float(getattr(cc_obj, "ford_soft_crawl_last_time_to_stop_est", 0.0))
      cs_bp.softCrawlMainControlDistance = float(getattr(cc_obj, "ford_soft_crawl_last_main_control_distance", 0.0))
      cs_bp.softCrawlCloseControlDistance = float(getattr(cc_obj, "ford_soft_crawl_last_close_control_distance", 0.0))
      cs_bp.softCrawlMainControlAllowed = bool(getattr(cc_obj, "ford_soft_crawl_last_main_control_allowed", False))
      cs_bp.softCrawlV17Enabled = bool(getattr(cc_obj, "ford_stock_acc_soft_crawl_v17", False))
      cs_bp.softCrawlV171Enabled = bool(getattr(cc_obj, "ford_stock_acc_soft_crawl_v171", False))
      cs_bp.softCrawlV172Enabled = bool(getattr(cc_obj, "ford_stock_acc_soft_crawl_v172", False))
      cs_bp_capnp = convert_to_capnp(cs_bp)
      cs_bp_send = messaging.new_message('controllerStateBP')
      cs_bp_send.valid = True
      cs_bp_send.controllerStateBP = cs_bp_capnp
      self.pm.send('controllerStateBP', cs_bp_send)

  def log_ford_long_observe(self, CS: car.CarState, CC: car.CarControl):
    if self.CP.brand != "ford":
      return

    now = time.monotonic()
    lead = None
    if self.sm.valid.get('radarState', False):
      lead = getattr(self.sm['radarState'], 'leadOne', None)
      if lead is not None and getattr(lead, 'status', 0) != 1:
        lead = None

    low_speed = CS.vEgo < 22.0
    should_log = low_speed or CC.longActive or CS.gasPressed or CS.brakePressed or lead is not None
    if not should_log:
      return
    stop_go_debug = bool(CC.longActive and (CS.vEgo < 8.0 or CS.standstill or CS.gasPressed or CS.brakePressed))
    log_interval = 1.0 if stop_go_debug else 5.0
    if now - self.ford_observe_last_log_t < log_interval:
      return

    self.ford_observe_last_log_t = now

    planner_a_target = 0.0
    planner_should_stop = False
    planner_source = 0
    if self.sm.seen['longitudinalPlan']:
      longitudinal_plan = self.sm['longitudinalPlan']
      planner_a_target = float(getattr(longitudinal_plan, "aTarget", 0.0))
      planner_should_stop = bool(getattr(longitudinal_plan, "shouldStop", False))
      planner_source_raw = getattr(longitudinal_plan, "longitudinalPlanSource", 0)
      try:
        planner_source = int(getattr(planner_source_raw, "raw", planner_source_raw))
      except (TypeError, ValueError):
        planner_source = str(planner_source_raw)

    long_state = getattr(CC.actuators, "longControlState", 0)
    try:
      long_state = int(getattr(long_state, "raw", long_state))
    except (TypeError, ValueError):
      long_state = 0

    radar = {
      "valid": False,
      "dRel": 0.0,
      "vRel": 0.0,
      "vLead": 0.0,
      "aLeadK": 0.0,
      "leadTime": 999.0,
      "ttc": 120.0,
    }
    if lead is not None:
      d_rel = float(getattr(lead, 'dRel', 0.0))
      v_rel = float(getattr(lead, 'vRel', 0.0))
      radar.update({
        "valid": True,
        "dRel": d_rel,
        "vRel": v_rel,
        "vLead": float(getattr(lead, 'vLead', 0.0)),
        "aLeadK": float(getattr(lead, 'aLeadK', 0.0)),
        "leadTime": d_rel / max(float(CS.vEgo), 0.5) if d_rel > 0.0 else 999.0,
        "ttc": d_rel / (-v_rel) if d_rel > 0.0 and v_rel < 0.0 else 60.0,
      })

    lead_valid = bool(radar["valid"])
    d_rel = float(radar["dRel"])
    v_rel = float(radar["vRel"])
    v_ego = max(float(CS.vEgo), 0.0)
    prev_lead_valid = self.ford_observe_prev_lead_valid
    prev_d_rel = self.ford_observe_prev_d_rel

    cut_in_detected = False
    cut_out_detected = False
    if lead_valid and prev_lead_valid:
      cut_in_detected = d_rel < prev_d_rel - max(4.0, 0.18 * max(prev_d_rel, 1.0)) or (v_rel < -4.0 and d_rel < 25.0)
      cut_out_detected = d_rel > prev_d_rel + max(6.0, 0.25 * max(prev_d_rel, 1.0))
    elif prev_lead_valid and not lead_valid:
      cut_out_detected = True

    if lead_valid and not cut_in_detected and not cut_out_detected:
      if not prev_lead_valid or self.ford_observe_lead_stable_since <= 0.0:
        self.ford_observe_lead_stable_since = now
    elif lead_valid:
      self.ford_observe_lead_stable_since = now
    else:
      self.ford_observe_lead_stable_since = 0.0

    lead_stability_age = now - self.ford_observe_lead_stable_since if self.ford_observe_lead_stable_since > 0.0 else 0.0
    lead_stable = lead_valid and lead_stability_age >= 1.0 and abs(v_rel) < 6.0

    source_key = (
      bool(self.CP.openpilotLongitudinalControl),
      bool(CC.longActive),
      bool(CS.cruiseState.enabled),
      bool(CS.cruiseState.standstill),
      lead_valid,
      long_state,
      planner_source,
    )
    if self.ford_observe_source_key is None:
      self.ford_observe_source_key = source_key
      self.ford_observe_source_transition_t = now
    elif source_key != self.ford_observe_source_key:
      self.ford_observe_source_key = source_key
      self.ford_observe_source_transition_t = now
    source_transition_age = now - self.ford_observe_source_transition_t

    human_override = bool(CS.gasPressed or CS.brakePressed)
    stock_time_gap_measured = d_rel / max(v_ego, 0.5) if lead_valid and d_rel > 0.0 else None
    comfort_decel = 1.4
    required_stop_distance = (v_ego * v_ego) / (2.0 * comfort_decel) + 2.5
    stop_distance_margin = d_rel - required_stop_distance if lead_valid else None

    city_stop_gap_target = 4.5
    high_speed_time_gap_target = 1.7 * v_ego + 2.0
    blend_ratio = min(max((v_ego - 5.0) / 15.0, 0.0), 1.0)
    target_gap = city_stop_gap_target * (1.0 - blend_ratio) + high_speed_time_gap_target * blend_ratio
    gap_error = d_rel - target_gap if lead_valid else None

    gap_too_small = lead_valid and stop_distance_margin is not None and stop_distance_margin < 0.5
    soft_stop_candidate = (
      bool(self.CP.openpilotLongitudinalControl) and CC.longActive and low_speed and lead_stable and
      not human_override and not cut_in_detected and not cut_out_detected and v_rel < -0.05 and d_rel < 45.0 and
      stop_distance_margin is not None and stop_distance_margin > 1.0
    )
    glide_allowed = soft_stop_candidate and gap_error is not None and gap_error > 1.5 and radar["ttc"] > 4.0
    resume_candidate = bool(CS.standstill and lead_valid and (float(radar["vLead"]) > 0.35 or v_rel > 0.35))
    if resume_candidate and self.ford_observe_resume_lead_move_t <= 0.0:
      self.ford_observe_resume_lead_move_t = now
    elif not CS.standstill or not lead_valid:
      self.ford_observe_resume_lead_move_t = 0.0

    resume_lead_move_age = now - self.ford_observe_resume_lead_move_t if self.ford_observe_resume_lead_move_t > 0.0 else None
    car_output_accel = float(getattr(self.last_actuators_output, "accel", 0.0))
    controlsd_accel = float(getattr(CC.actuators, "accel", 0.0))
    resume_command_delay = resume_lead_move_age if resume_lead_move_age is not None and car_output_accel > 0.15 else None
    resume_vehicle_delay = resume_lead_move_age if resume_lead_move_age is not None and v_ego > 0.35 else None
    resume_ramp_active = resume_lead_move_age is not None and resume_vehicle_delay is None and controlsd_accel > 0.15

    if not self.CP.openpilotLongitudinalControl or not CC.longActive:
      fscs_mode = "observe"
    elif resume_candidate:
      fscs_mode = "resume"
    elif CS.standstill:
      fscs_mode = "hold"
    elif long_state == 2 and v_ego < 1.2:
      fscs_mode = "touchdown"
    elif glide_allowed:
      fscs_mode = "glide"
    elif soft_stop_candidate:
      fscs_mode = "approach"
    else:
      fscs_mode = "observe"

    guard_reasons = []
    if human_override:
      guard_reasons.append("human")
    if lead_valid and not lead_stable:
      guard_reasons.append("lead_unstable")
    if cut_in_detected:
      guard_reasons.append("cut_in")
    if cut_out_detected:
      guard_reasons.append("cut_out")
    if gap_too_small:
      guard_reasons.append("gap_too_small")
    if v_ego >= 22.0:
      guard_reasons.append("speed_too_high")
    if source_transition_age < 1.0:
      guard_reasons.append("source_transition")
    if not guard_reasons:
      guard_reasons.append("none")

    desired_accel_raw = controlsd_accel
    desired_accel_fscs = desired_accel_raw
    accel_limit_reason = "none"
    if cut_in_detected and desired_accel_raw > -1.0:
      desired_accel_fscs = -1.0
      accel_limit_reason = "cut_in_guard"
    elif gap_too_small and desired_accel_raw > -0.8:
      desired_accel_fscs = -0.8
      accel_limit_reason = "gap_guard"
    elif fscs_mode == "touchdown" and desired_accel_raw < -0.6:
      desired_accel_fscs = -0.6
      accel_limit_reason = "touchdown"
    elif resume_ramp_active and desired_accel_raw > 0.6:
      desired_accel_fscs = 0.6
      accel_limit_reason = "resume_jerk"

    self.ford_observe_prev_lead_valid = lead_valid
    self.ford_observe_prev_d_rel = d_rel
    self.ford_observe_prev_v_rel = v_rel

    cc_obj = self.CI.CC
    payload = {
      "tag": "FORD_LONG_OBS_V3",
      "opLong": bool(self.CP.openpilotLongitudinalControl),
      "fordLongitudinalGap": self.params.get("FordLongitudinalGap", return_default=True),
      "longActive": bool(CC.longActive),
      "longState": long_state,
      "vEgo": float(CS.vEgo),
      "aEgo": float(CS.aEgo),
      "standstill": bool(CS.standstill),
      "cruiseEnabled": bool(CS.cruiseState.enabled),
      "cruiseAvailable": bool(CS.cruiseState.available),
      "cruiseStandstill": bool(CS.cruiseState.standstill),
      "gasPressed": bool(CS.gasPressed),
      "brakePressed": bool(CS.brakePressed),
      "steeringPressed": bool(CS.steeringPressed),
      "plannerATarget": planner_a_target,
      "plannerShouldStop": planner_should_stop,
      "plannerSource": planner_source,
      "controlsdAccel": controlsd_accel,
      "carOutputAccel": car_output_accel,
      "carOutputGas": float(getattr(self.last_actuators_output, "gas", 0.0)),
      "controllerAccel": float(getattr(cc_obj, "accel", 0.0)),
      "controllerGas": float(getattr(cc_obj, "gas", 0.0)),
      "bpLongActiveLast": bool(getattr(cc_obj, "_bp_long_active_last", False)),
      "bpSpeedAllow": bool(getattr(cc_obj, "bpSpeedAllow", False)),
      "debug": {
        "fscsEnabled": True,
        "fscsMode": fscs_mode,
        "fscsGuardReason": guard_reasons,
        "stockAccGapLevel": None,
        "stockTimeGapMeasured": stock_time_gap_measured,
        "cityStopGapTarget": city_stop_gap_target,
        "highSpeedTimeGapTarget": high_speed_time_gap_target,
        "blendRatio": blend_ratio,
        "targetGap": target_gap,
        "gapError": gap_error,
        "requiredStopDistance": required_stop_distance,
        "stopDistanceMargin": stop_distance_margin,
        "leadStable": lead_stable,
        "leadStabilityAge": lead_stability_age,
        "cutInDetected": cut_in_detected,
        "cutOutDetected": cut_out_detected,
        "sourceTransitionAge": source_transition_age,
        "softStopCandidate": soft_stop_candidate,
        "glideAllowed": glide_allowed,
        "resumeCandidate": resume_candidate,
        "resumeRampActive": resume_ramp_active,
        "resumeLeadMoveAge": resume_lead_move_age,
        "resumeCommandDelay": resume_command_delay,
        "resumeVehicleDelay": resume_vehicle_delay,
        "desiredAccelRaw": desired_accel_raw,
        "desiredAccelFscs": desired_accel_fscs,
        "accelLimitReason": accel_limit_reason,
        "stockAccV12Enabled": bool(getattr(cc_obj, "ford_stock_acc_stop_go_v12", False)),
        "stockAccV12Phase": getattr(cc_obj, "ford_v12_phase", "unknown"),
        "stockAccV12Reason": getattr(cc_obj, "ford_v12_reason", "unknown"),
        "stockAccV12StopRequest": bool(getattr(cc_obj, "ford_v12_last_stop_request", False)),
        "stockAccV12ResumeEnable": bool(getattr(cc_obj, "ford_v12_last_resume_enable", False)),
        "stockAccV12TargetSpeedKph": float(getattr(cc_obj, "ford_v12_last_target_speed", 0.0)),
        "stockAccV12Accel": float(getattr(cc_obj, "ford_v12_last_accel", 0.0)),
        "stockAccV12Gas": float(getattr(cc_obj, "ford_v12_last_gas", 0.0)),
        "stockAccV12BrakeActuate": bool(getattr(cc_obj, "ford_v12_last_brake_actuate", False)),
        "stockAccV12PrechargeActuate": bool(getattr(cc_obj, "ford_v12_last_precharge_actuate", False)),
        "stockAccV12OriginalAccel": float(getattr(cc_obj, "ford_v12_last_original_accel", 0.0)),
        "stockAccV12OriginalGas": float(getattr(cc_obj, "ford_v12_last_original_gas", 0.0)),
        "stockAccV12OriginalBrakeActuate": bool(getattr(cc_obj, "ford_v12_last_original_brake_actuate", False)),
        "stockAccV12OriginalPrechargeActuate": bool(getattr(cc_obj, "ford_v12_last_original_precharge_actuate", False)),
        "stockAccV12LeadValid": bool(getattr(cc_obj, "ford_v12_last_lead_valid", False)),
        "stockAccV12LeadDRel": float(getattr(cc_obj, "ford_v12_last_lead_d_rel", 0.0)),
        "stockAccV12LeadVRel": float(getattr(cc_obj, "ford_v12_last_lead_v_rel", 0.0)),
        "stockAccV12LeadVLead": float(getattr(cc_obj, "ford_v12_last_lead_v_lead", 0.0)),
        "stockAccV12Ttc": float(getattr(cc_obj, "ford_v12_last_ttc", 120.0)),
        "stockAccV12ResumeAge": float(getattr(cc_obj, "ford_v12_last_resume_age", 0.0)),
        "stockAccV12HoldAccel": float(getattr(cc_obj, "ford_v12_last_hold_accel", -0.45)),
        "stockAccV12TouchdownFloor": float(getattr(cc_obj, "ford_v12_last_touchdown_floor", -0.45)),
        "stockAccV13LeadStable": bool(getattr(cc_obj, "ford_v13_last_lead_stable", False)),
        "stockAccV13LeadStabilityAge": float(getattr(cc_obj, "ford_v13_last_lead_stability_age", 0.0)),
        "stockAccV13LeadDropoutAge": float(getattr(cc_obj, "ford_v13_last_lead_dropout_age", 0.0)),
        "stockAccV13CutIn": bool(getattr(cc_obj, "ford_v13_last_cut_in", False)),
        "stockAccV13CutOut": bool(getattr(cc_obj, "ford_v13_last_cut_out", False)),
        "stockAccV13GateReason": getattr(cc_obj, "ford_v13_last_gate_reason", "unknown"),
        "stockGoLeadMoved": bool(getattr(cc_obj, "ford_v14_last_lead_moved", False)),
        "stockGoLeadStable": bool(getattr(cc_obj, "ford_v14_last_lead_stable", False)),
        "stockGoLeadStableAge": float(getattr(cc_obj, "ford_v14_last_lead_stable_age", 0.0)),
        "stockGoTimeSinceLeadMove": float(getattr(cc_obj, "ford_v14_last_time_since_lead_move", 0.0)),
        "stockGoEgoLag": float(getattr(cc_obj, "ford_v14_last_ego_lag", 0.0)),
        "stockGoCandidate": bool(getattr(cc_obj, "ford_v14_last_candidate", False)),
        "stockGoBlockedReason": getattr(cc_obj, "ford_v14_last_blocked_reason", "unknown"),
        "stockGoDesiredAccel": float(getattr(cc_obj, "ford_v14_last_desired_accel", 0.0)),
        "stockGoJerkLimitedAccel": float(getattr(cc_obj, "ford_v14_last_jerk_limited_accel", 0.0)),
        "stockGoReleasePhase": getattr(cc_obj, "ford_v14_last_release_phase", "unknown"),
        "stockGoControlEnabled": bool(getattr(cc_obj, "ford_v14_last_control_enabled", False)),
        "softCrawlObserveEnabled": bool(getattr(cc_obj, "ford_stock_acc_soft_crawl_observe", False)),
        "softCrawlControlEnabled": bool(getattr(cc_obj, "ford_stock_acc_soft_crawl_control", False) and
                                        (getattr(cc_obj, "ford_stock_acc_soft_crawl_v16", False) or
                                         getattr(cc_obj, "ford_stock_acc_soft_crawl_v17", False) or
                                         getattr(cc_obj, "ford_stock_acc_soft_crawl_v171", False))),
        "softCrawlAvailable": bool(getattr(cc_obj, "ford_soft_crawl_last_available", False)),
        "softCrawlReason": getattr(cc_obj, "ford_soft_crawl_last_reason", "unknown"),
        "softCrawlFallbackReason": getattr(cc_obj, "ford_soft_crawl_last_fallback_reason", "unknown"),
        "softCrawlTargetAccel": float(getattr(cc_obj, "ford_soft_crawl_last_target_accel", 0.0)),
        "softCrawlDistanceToStop": float(getattr(cc_obj, "ford_soft_crawl_last_distance_to_stop", 0.0)),
        "softCrawlNeededDistance": float(getattr(cc_obj, "ford_soft_crawl_last_needed_distance", 0.0)),
        "softCrawlDistanceMargin": float(getattr(cc_obj, "ford_soft_crawl_last_distance_margin", 0.0)),
        "softCrawlCurrentStopDistance": float(getattr(cc_obj, "ford_soft_crawl_last_current_stop_distance", 0.0)),
        "softCrawlLeadDRel": float(getattr(cc_obj, "ford_soft_crawl_last_lead_d_rel", 0.0)),
        "softCrawlLeadVRel": float(getattr(cc_obj, "ford_soft_crawl_last_lead_v_rel", 0.0)),
        "softCrawlLeadVLead": float(getattr(cc_obj, "ford_soft_crawl_last_lead_v_lead", 0.0)),
        "softCrawlTtc": float(getattr(cc_obj, "ford_soft_crawl_last_ttc", 0.0)),
        "softCrawlVEgo": float(getattr(cc_obj, "ford_soft_crawl_last_v_ego", 0.0)),
        "softCrawlOriginalAccel": float(getattr(cc_obj, "ford_soft_crawl_last_original_accel", 0.0)),
        "softCrawlPlannerStopping": bool(getattr(cc_obj, "ford_soft_crawl_last_planner_stopping", False)),
        "softCrawlControlActive": bool(getattr(cc_obj, "ford_soft_crawl_last_control_active", False)),
        "softCrawlPhase": getattr(cc_obj, "ford_soft_crawl_last_phase", "unknown"),
        "softCrawlControlStage": getattr(cc_obj, "ford_soft_crawl_last_control_stage", "unknown"),
        "softCrawlTimeToStopEst": float(getattr(cc_obj, "ford_soft_crawl_last_time_to_stop_est", 0.0)),
        "softCrawlMainControlDistance": float(getattr(cc_obj, "ford_soft_crawl_last_main_control_distance", 0.0)),
        "softCrawlCloseControlDistance": float(getattr(cc_obj, "ford_soft_crawl_last_close_control_distance", 0.0)),
        "softCrawlMainControlAllowed": bool(getattr(cc_obj, "ford_soft_crawl_last_main_control_allowed", False)),
        "softCrawlStopGapTarget": float(getattr(cc_obj, "ford_soft_crawl_last_stop_gap_target", 0.0)),
        "softCrawlRawTargetAccel": float(getattr(cc_obj, "ford_soft_crawl_last_raw_target_accel", 0.0)),
        "softCrawlJerkLimitedAccel": float(getattr(cc_obj, "ford_soft_crawl_last_jerk_limited_accel", 0.0)),
        "softCrawlAccelAfterControl": float(getattr(cc_obj, "ford_soft_crawl_last_accel_after_control", 0.0)),
        "softCrawlV16Enabled": bool(getattr(cc_obj, "ford_stock_acc_soft_crawl_v16", False) or
                                    getattr(cc_obj, "ford_stock_acc_soft_crawl_v17", False) or
                                    getattr(cc_obj, "ford_stock_acc_soft_crawl_v171", False) or
                                    getattr(cc_obj, "ford_stock_acc_soft_crawl_v172", False)),
        "softCrawlV17Enabled": bool(getattr(cc_obj, "ford_stock_acc_soft_crawl_v17", False)),
        "softCrawlV171Enabled": bool(getattr(cc_obj, "ford_stock_acc_soft_crawl_v171", False)),
        "softCrawlV172Enabled": bool(getattr(cc_obj, "ford_stock_acc_soft_crawl_v172", False)),
      },
      "radar": radar,
    }
    cloudlog.info("FORD_LONG_OBS_V3 " + json.dumps(payload, separators=(",", ":")))

  def step(self):
    CS, CS_SP, RD = self.state_update()

    self.state_publish(CS, CS_SP, RD)

    initialized = (not any(e.name == EventName.selfdriveInitializing for e in self.sm['onroadEvents']) and
                   self.sm.seen['onroadEvents'])
    if not self.CP.passive and initialized:
      self.controls_update(CS, self.sm['carControl'], self.sm['carControlSP'])

    self.initialized_prev = initialized
    self.CS_prev = CS
    self.CS_SP_prev = CS_SP

  def params_thread(self, evt):
    while not evt.is_set():
      self.is_metric = self.params.get_bool("IsMetric")
      self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl

      # sunnypilot
      self.dynamic_experimental_control = self.params.get_bool("DynamicExperimentalControl")
      self.v_cruise_helper.read_custom_set_speed_params()

      time.sleep(0.1)

  def card_thread(self):
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e, ))
    try:
      t.start()
      while True:
        self.step()
        self.rk.monitor_time()
    finally:
      e.set()
      t.join()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  car = Car()
  car.card_thread()


if __name__ == "__main__":
  main()
