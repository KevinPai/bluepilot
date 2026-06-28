"""
BluePilot: MICI vehicle menu — current vehicle, clear manual fingerprint, select vehicle.

Uses horizontal NavScroller (same pattern as WiFi / preferred network).
"""

from __future__ import annotations

import os
from collections.abc import Callable

from openpilot.common.basedir import BASEDIR
from openpilot.selfdrive.ui.bp.mici.widgets.button_bp import BigButtonBP
from openpilot.selfdrive.ui.bp.mici.widgets.vehicle_select_mici import (
  VehicleMakeSelectMici,
  load_car_platforms,
)
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller

CAR_LIST_JSON = os.path.join(BASEDIR, "sunnypilot", "selfdrive", "car", "car_list.json")


def _truncate(s: str, max_len: int = 36) -> str:
  s = s.strip()
  if len(s) <= max_len:
    return s
  return s[: max_len - 3] + "..."


def get_vehicle_status_text() -> tuple[str, str]:
  """
  Returns (title_line, value_line) for the current vehicle display.
  Mirrors TICI PlatformSelector.refresh() semantics.
  """
  if bundle := ui_state.params.get("CarPlatformBundle"):
    name = bundle.get("name", "")
    if isinstance(name, bytes):
      name = name.decode("utf-8", errors="replace")
    return (tr("manual selection"), _truncate(str(name)))
  if ui_state.CP is not None and ui_state.CP.carFingerprint != "MOCK":
    fp = str(ui_state.CP.carFingerprint)
    return (tr("auto fingerprint"), _truncate(fp))
  return (tr("vehicle"), tr("unrecognized"))


class VehicleLayoutMici(NavScroller):
  """Three-button horizontal strip: current vehicle | clear | select."""

  def __init__(self, back_callback: Callable[[], None]):
    super().__init__()
    self.set_back_callback(back_callback)
    self._platforms: dict = {}
    try:
      self._platforms = load_car_platforms()
    except OSError as e:
      from openpilot.common.swaglog import cloudlog

      cloudlog.error(f"MICI vehicle: could not load {CAR_LIST_JSON}: {e}")

    self._btn_current = BigButtonBP(tr("current vehicle"), "", "../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png")
    self._btn_clear = BigButtonBP(tr("clear vehicle"), "", "../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png")
    self._btn_select = BigButtonBP(tr("select vehicle"), "", "../../sunnypilot/selfdrive/assets/offroad/icon_vehicle.png")

    self._btn_current.set_enabled(False)
    self._btn_clear.set_click_callback(self._on_clear)
    self._btn_select.set_click_callback(self._on_select)

    self._scroller.add_widgets([self._btn_current, self._btn_clear, self._btn_select])

  def show_event(self):
    super().show_event()
    ui_state.update_params()
    self._refresh_display()

  def _update_state(self):
    super()._update_state()
    ui_state.update_params()
    self._refresh_display()

  def _refresh_display(self):
    t, v = get_vehicle_status_text()
    self._btn_current.set_text(t)
    self._btn_current.set_value(v)
    has_manual = bool(ui_state.params.get("CarPlatformBundle"))
    self._btn_clear.set_enabled(has_manual)
    self._btn_select.set_enabled(len(self._platforms) > 0)

  def _on_clear(self):
    if ui_state.params.get("CarPlatformBundle"):
      ui_state.params.remove("CarPlatformBundle")
    self._refresh_display()

  def _on_select(self):
    if not self._platforms:
      return

    def on_complete():
      self._refresh_display()

    gui_app.push_widget(VehicleMakeSelectMici(self._platforms, on_stack_done=on_complete))
