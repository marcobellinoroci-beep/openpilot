import pyray as rl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.starpilot.common.favorite_slots import request_starpilot_toggle_refresh
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

STEER_ONLY_PARAM = "SteerOnlyMode"


def steer_only_tap_allowed() -> bool:
  """Flip Steer Only only while neither openpilot nor the car's cruise is engaged.

  With cruise off nothing is accelerating, so turning it ON never cancels an active cruise at speed and
  turning it OFF never hands speed to cruise on its own: the driver still has to engage it with the stalk.
  The brake is not involved.
  """
  if not ui_state.started:
    return False
  sm = ui_state.sm
  return not sm["selfdriveState"].enabled and not sm["carState"].cruiseState.enabled


class SteerOnlyChip(Widget):
  """Tappable onroad chip that turns Steer Only mode on or off."""
  FONT_SIZE = 24
  PAD_X, PAD_Y = 12, 8

  def __init__(self):
    super().__init__()
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._interacting = False
    self._on = False

  def interacting(self) -> bool:
    interacting, self._interacting = self._interacting, False
    return interacting

  def layout_at(self, right_x: float, top_y: float) -> None:
    self._on = ui_state.params.get_bool(STEER_ONLY_PARAM)
    size = measure_text_cached(self._font, self._label(), self.FONT_SIZE)
    width, height = size.x + self.PAD_X * 2, size.y + self.PAD_Y * 2
    self.set_rect(rl.Rectangle(right_x - width, top_y, width, height))

  def _label(self) -> str:
    return "STEER ONLY" if self._on else "STEER ONLY OFF"

  def _handle_mouse_press(self, mouse_pos) -> None:
    self._interacting = True

  def _handle_mouse_release(self, mouse_pos) -> None:
    self._interacting = True
    if not steer_only_tap_allowed():
      return
    self._on = not ui_state.params.get_bool(STEER_ONLY_PARAM)
    ui_state.params.put_bool(STEER_ONLY_PARAM, self._on)
    request_starpilot_toggle_refresh(ui_state.params_memory)

  def _render(self, rect: rl.Rectangle) -> None:
    alpha = 255 if steer_only_tap_allowed() else 140
    if self._on:
      bg, fg = rl.Color(255, 152, 0, int(alpha * 0.85)), rl.Color(0, 0, 0, alpha)
    else:
      bg, fg = rl.Color(0, 0, 0, int(alpha * 0.5)), rl.Color(220, 220, 220, alpha)
    rl.draw_rectangle_rounded(rect, 0.4, 6, bg)
    if not self._on:
      rl.draw_rectangle_rounded_lines_ex(rect, 0.4, 6, 2, rl.Color(220, 220, 220, int(alpha * 0.6)))
    rl.draw_text_ex(self._font, self._label(), rl.Vector2(rect.x + self.PAD_X, rect.y + self.PAD_Y), self.FONT_SIZE, 0, fg)
