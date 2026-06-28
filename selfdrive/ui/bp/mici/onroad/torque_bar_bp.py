import pyray as rl
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TorqueBar
from openpilot.selfdrive.ui.ui_state import ui_state

class TorqueBarBP(TorqueBar):
  """BluePilot TorqueBar with lateral uncertainty from controllerStateBP.

  On angleState vehicles (e.g. Tesla), uses lateralUncertainty from
  controllerStateBP instead of the acceleration-based calculation.
  """

  def _update_state(self):
    if self._demo:
      return

    # BluePilot: Use lateral uncertainty from controllerStateBP when available on angleState
    if ui_state.sm['controlsState'].lateralControlState.which() == 'angleState':
      if ui_state.sm.valid.get("controllerStateBP", False):
        try:
          lateral_uncertainty = ui_state.sm['controllerStateBP'].lateralUncertainty
          # lateralUncertainty is already normalized to [-1, 1] range
          self._torque_filter.update(min(max(lateral_uncertainty, -1), 1))
          return
        except (KeyError, AttributeError):
          pass

    # Fall back to stock behavior for non-angleState or when controllerStateBP unavailable
    super()._update_state()

    def _render(self, rect: rl.Rectangle) -> None:
      if ui_state.sm['controlsState'].lateralControlState.which() == 'angleState' and not ui_state.sm.valid.get("controllerStateBP", False):
         return
      super()._render(rect)
