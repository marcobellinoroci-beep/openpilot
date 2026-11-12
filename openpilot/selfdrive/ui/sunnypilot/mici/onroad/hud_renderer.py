"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import raylib_nb as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import SET_SPEED_PERSISTENCE, HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_assist_badge import draw_gauge_centered
from openpilot.selfdrive.ui.ui_state import ChestnutState, UIStatus, ui_state
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.approach import approach_ceiling
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

# zoompilot's Set Speed Assist gauge mirrors the steering wheel in the bottom-right corner
SPEED_ASSIST_HEIGHT = 44  # the wheel is 50, and round; the gauge's swoosh carries it to the same weight
# the HUD's idle and lit levels: armed draws like an unlit distance bar, and brightens to the
# wheel's 0.9 as the lift nears the most the personality's approach pace asks for at this speed
SPEED_ASSIST_ALPHA_IDLE = 0.35
SPEED_ASSIST_ALPHA_LIFT = 0.9

# Pixels added on every side of the drawn AO badge. Raylib's point-in-rect test is
# half-open on the far edges, so this covers a full 100 px on the near sides.
AO_HIT_PAD = 100.0

AO_ON_BORDER = rl.Color(255, 200, 0, 200)
AO_ON_TEXT = rl.Color(255, 200, 0, 230)
AO_OFF_BORDER = rl.Color(200, 200, 200, 120)
AO_OFF_TEXT = rl.Color(200, 200, 200, 160)
AO_BG_ON = rl.Color(0, 0, 0, 166)
AO_BG_OFF = rl.Color(0, 0, 0, 140)


def ao_hit_rect(x: float, y: float, width: float, height: float, pad: float = AO_HIT_PAD) -> tuple[float, float, float, float]:
  """Drawn badge expanded by `pad` pixels on every side."""
  return (x - pad, y - pad, width + 2 * pad, height + 2 * pad)


def ao_hit_contains(px: float, py: float, x: float, y: float, width: float, height: float, pad: float = AO_HIT_PAD) -> bool:
  """Raylib CheckCollisionPointRec: x in [left, right) and y in [top, bottom)."""
  hx, hy, hw, hh = ao_hit_rect(x, y, width, height, pad)
  return hx <= px < hx + hw and hy <= py < hy + hh


class AutoOvertakeBadge(Widget):
  """Onroad AO badge. A tap toggles AutoOvertake. Dimmed 'AO OFF' when the feature is off."""

  def __init__(self, font, size: int, pad_x: int, pad_y: int, border: float, segments: int, text_nudge: float):
    super().__init__()
    self._font = font
    self._size = size
    self._pad_x = pad_x
    self._pad_y = pad_y
    self._border = border
    self._segments = segments
    self._text_nudge = text_nudge
    self._text = "AO"
    self._on = False
    self._visual_x = 0.0
    self._visual_y = 0.0
    self._visual_w = 0.0
    self._visual_h = 0.0
    # The road view handles taps after this badge and ignores them while it is pressed.
    # This delay keeps a quick tap (press and release in one frame) from also toggling
    # the sidebar or scrolling home. Same hold as SpeedAssistButton.
    self._click_delay = 0.075

  def place(self, x: float, y: float, *, align_right: bool = False) -> None:
    self._on = bool(getattr(ui_state, "auto_overtake", False))
    self._text = "AO" if self._on else "AO OFF"
    tw = measure_text_cached(self._font, self._text, self._size).x
    self._visual_w = tw + self._pad_x * 2
    self._visual_h = self._size + self._pad_y * 2
    self._visual_x = (x - self._visual_w) if align_right else x
    self._visual_y = y
    hx, hy, hw, hh = ao_hit_rect(self._visual_x, self._visual_y, self._visual_w, self._visual_h)
    self.set_rect(rl.Rectangle(hx, hy, hw, hh))

  def _handle_mouse_release(self, mouse_pos) -> None:
    super()._handle_mouse_release(mouse_pos)
    if ui_state.is_onroad():
      ui_state.toggle_auto_overtake()

  def _render(self, _rect: rl.Rectangle) -> None:
    border = AO_ON_BORDER if self._on else AO_OFF_BORDER
    text_color = AO_ON_TEXT if self._on else AO_OFF_TEXT
    bg = AO_BG_ON if self._on else AO_BG_OFF
    if self.is_pressed:
      border = rl.fade(border, 0.55)
      text_color = rl.fade(text_color, 0.55)
      bg = rl.fade(bg, 0.55)
    badge = rl.Rectangle(self._visual_x, self._visual_y, self._visual_w, self._visual_h)
    rl.draw_rectangle_rounded(badge, 0.35, self._segments, bg)
    rl.draw_rectangle_rounded_lines_ex(badge, 0.35, self._segments, self._border, border)
    rl.draw_text_ex(
      self._font,
      self._text,
      rl.Vector2(self._visual_x + self._pad_x, self._visual_y + self._pad_y - self._text_nudge),
      self._size,
      0,
      text_color,
    )


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self._sp_engaged: bool = False
    # the wheel's filters, so the two corners come and go together
    self._speed_assist_shown_filter = FirstOrderFilter(0.0, 0.05, 1 / gui_app.target_fps)
    self._speed_assist_y_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)
    # the lift lets go fast; ease the glow so it doesn't blink
    self._speed_assist_lift_filter = FirstOrderFilter(0.0, 0.3, 1 / gui_app.target_fps)
    self._ao_badge = AutoOvertakeBadge(self._font_semi_bold, size=22, pad_x=8, pad_y=4, border=2, segments=8, text_nudge=1)

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()

    sp_engaged = ui_state.engaged
    if sp_engaged != self._sp_engaged:
      self._chestnut_fade_time = rl.get_time() if sp_engaged else 0
    self._sp_engaged = sp_engaged

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    self._draw_speed_assist(rect)
    self._draw_auto_overtake_badge(rect)

  def _chestnut_showing(self) -> bool:
    """The model icon wants the corner: while loading, and for its persistence after engaging or a
    state change (HudRenderer._draw_model_source)."""
    state = ui_state.chestnut_state
    if state == ChestnutState.LOADING:
      return True
    if state not in (ChestnutState.UNCOMPILED, ChestnutState.FAILED, ChestnutState.ACTIVE):
      return False
    return rl.get_time() - self._chestnut_fade_time < SET_SPEED_PERSISTENCE

  def _draw_speed_assist(self, rect: rl.Rectangle) -> None:
    # comes and goes with the wheel, and swaps in once the model icon is done with the shared corner
    wheel = self._txt_wheel
    shown = (ui_state.speed_assist_active and ui_state.status != UIStatus.DISENGAGED and
             not self.blind_spot_indicators.right_detected and not self._chestnut_showing())
    lift = self._speed_assist_lift_filter.update(self._speed_assist_lift_fraction() if shown else 0.0)
    alpha = self._speed_assist_shown_filter.update(float(shown)) * \
      (SPEED_ASSIST_ALPHA_IDLE + (SPEED_ASSIST_ALPHA_LIFT - SPEED_ASSIST_ALPHA_IDLE) * lift)
    y_offset = self._speed_assist_y_filter.update(0.0 if shown else wheel.height / 2)
    if alpha < 1e-2:
      return
    draw_gauge_centered(rect.x + rect.width - 21 - wheel.width / 2, rect.y + rect.height - 14 - wheel.height / 2 + y_offset,
                        SPEED_ASSIST_HEIGHT, alpha)

  @staticmethod
  def _speed_assist_lift_fraction() -> float:
    """The lift on the model's acceleration over the personality's approach ceiling, 0..1."""
    sm = ui_state.sm
    if not sm.valid['longitudinalPlanSP']:
      return 0.0
    boost = sm['longitudinalPlanSP'].zoompilot.e2eSetSpeed.boost
    # the lift never goes negative, and the ceiling's floor is 0.5 m/s^2
    return min(boost / approach_ceiling(sm['carState'].vEgo, sm['selfdriveState'].personality.raw), 1.0)

  def left_blind_spot_detected(self) -> bool:
    return self.blind_spot_indicators.left_detected

  def _draw_auto_overtake_badge(self, rect: rl.Rectangle) -> None:
    """AO badge, tappable while onroad. Dimmed 'AO OFF' when AutoOvertake is disabled.
    Top-right: set speed is top-left, current speed is centered, wheel is bottom-left,
    and the model / Set Speed Assist icon uses the bottom-right."""
    if not ui_state.is_onroad():
      return
    self._ao_badge.place(rect.x + rect.width - 12, rect.y + 10, align_right=True)
    self._ao_badge.render()

  def user_interacting(self) -> bool:
    return self._ao_badge.is_pressed

  def _has_blind_spot_detected(self) -> bool:
    return self.blind_spot_indicators.detected
