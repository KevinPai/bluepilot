"""Section header widget for TICI BluePilot settings menu."""
import pyray as rl

from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.lib.application import FontWeight

# Match list item width; height for bold section title
SECTION_HEADER_WIDTH = 600
SECTION_HEADER_HEIGHT = 65
SECTION_HEADER_FONT_SIZE = 52


class SectionHeader(Widget):
  """Non-interactive bold section header for dividing menu sections."""

  def __init__(self, title: str):
    super().__init__()
    self._title = title
    self.set_rect(rl.Rectangle(0, 0, SECTION_HEADER_WIDTH, SECTION_HEADER_HEIGHT))

  def _render(self, rect: rl.Rectangle) -> None:
    gui_label(
      rect,
      self._title,
      font_size=SECTION_HEADER_FONT_SIZE,
      font_weight=FontWeight.BOLD,
      color=rl.Color(220, 220, 220, 255),
      alignment=rl.GuiTextAlignment.TEXT_ALIGN_LEFT,
      alignment_vertical=rl.GuiTextAlignmentVertical.TEXT_ALIGN_MIDDLE,
      elide_right=True,
    )
