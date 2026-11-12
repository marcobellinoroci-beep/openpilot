"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import raylib_nb as rl

from openpilot.common.constants import CV
from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.sunnypilot.onroad.blendful_zp import TorqueBarZP
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import DeveloperUiRenderer, DeveloperUiState, get_bottom_dev_ui_offset
from openpilot.selfdrive.ui.sunnypilot.onroad.confidence_ball import ConfidenceBallTizi
from openpilot.selfdrive.ui.sunnypilot.onroad.long_indicator import LongIndicatorTizi
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_assist_button import SpeedAssistButton
from openpilot.selfdrive.ui.sunnypilot.onroad.road_name import RoadNameRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.rocket_fuel import RocketFuel
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_limit import SpeedLimitRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.smart_cruise_control import SmartCruiseControlRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.turn_signal import TurnSignalController
from openpilot.selfdrive.ui.sunnypilot.onroad.circular_alerts import CircularAlertsRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_renderer import SpeedRenderer
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.onroad.hud_renderer import HudRenderer, UI_CONFIG, FONT_SIZES, COLORS, CRUISE_DISABLED_CHAR
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

SLA_ACTIVE_COLOR = rl.Color(0x91, 0x9b, 0x95, 0xff)

# the experimental mode button's place, as upstream drew it before retiring chill mode
EXP_BUTTON_SIZE = 192

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
    self.developer_ui = DeveloperUiRenderer()
    self.road_name_renderer = RoadNameRenderer()
    self.rocket_fuel = RocketFuel()
    self.speed_limit_renderer = SpeedLimitRenderer()
    self.smart_cruise_control_renderer = SmartCruiseControlRenderer()
    self.turn_signal_controller = TurnSignalController()
    self.circular_alerts_renderer = CircularAlertsRenderer()
    self.speed_renderer = SpeedRenderer()
    self._torque_bar = TorqueBarZP(scale=3.0, always=True, show_bar=lambda: ui_state.torque_bar)
    self._speed_assist_button = SpeedAssistButton(EXP_BUTTON_SIZE)
    self._ao_badge = AutoOvertakeBadge(self._font_semi_bold, size=32, pad_x=12, pad_y=6, border=3, segments=10, text_nudge=2)
    self._long_indicator = LongIndicatorTizi()
    self._confidence_ball = ConfidenceBallTizi()

    self.pcm_cruise_speed: bool = True
    self.show_icbm_status: bool = False
    self.icbm_active_counter: int = 0
    self.speed_cluster: float = 0.0
    self.speed_conv: float = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH

  def _update_state(self) -> None:
    if ui_state.sm.recv_frame["carState"] < ui_state.started_frame:
      return

    if ui_state.CP_SP is not None:
      self.pcm_cruise_speed = not ui_state.icbm_running
    self.speed_conv = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed_cluster = ui_state.sm['carState'].cruiseState.speedCluster * self.speed_conv

    super()._update_state()
    self.road_name_renderer.update()
    self.speed_limit_renderer.update()
    self.smart_cruise_control_renderer.update()
    self.turn_signal_controller.update()
    self.circular_alerts_renderer.update()
    self.speed_renderer.update()

  def _get_icbm_status(self):
    if not self.pcm_cruise_speed and ui_state.sm['carControl'].enabled:
      if round(self.set_speed) != round(self.speed_cluster):
        self.icbm_active_counter = 3 * gui_app.target_fps  # 3 seconds usually
      elif self.icbm_active_counter > 0:
        self.icbm_active_counter -= 1
    else:
      self.icbm_active_counter = 0

    self.show_icbm_status = self.icbm_active_counter > 0

  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    long_plan_sp = ui_state.sm['longitudinalPlanSP']
    long_override = ui_state.sm['carControl'].cruiseControl.override
    self._get_icbm_status()

    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 45

    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height)
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 10, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 10, 6, COLORS.BORDER_TRANSLUCENT)

    max_color = COLORS.GREY
    set_speed_color = COLORS.DARK_GREY
    if self.is_cruise_set:
      set_speed_color = COLORS.WHITE
      if long_plan_sp.speedLimit.assist.active:
        set_speed_color = SLA_ACTIVE_COLOR if long_override else rl.Color(0, 0xff, 0, 0xff)
        max_color = SLA_ACTIVE_COLOR if long_override else rl.Color(0x80, 0xd8, 0xa6, 0xff)
      else:
        if ui_state.status == UIStatus.ENGAGED:
          max_color = COLORS.ENGAGED
        elif ui_state.status == UIStatus.DISENGAGED:
          max_color = COLORS.DISENGAGED
        elif ui_state.status == UIStatus.OVERRIDE:
          max_color = COLORS.OVERRIDE

    max_str_size = 60 if self.show_icbm_status else 40
    max_str_y = 15 if self.show_icbm_status else 27

    max_text = str(round(self.speed_cluster)) if self.show_icbm_status else tr("MAX")
    max_text_width = measure_text_cached(self._font_semi_bold, max_text, max_str_size).x
    rl.draw_text_ex(
      self._font_semi_bold,
      max_text,
      rl.Vector2(x + (set_speed_width - max_text_width) / 2, y + max_str_y),
      max_str_size,
      0,
      max_color,
    )

    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(self.set_speed))
    speed_text_width = measure_text_cached(self._font_bold, set_speed_text, FONT_SIZES.set_speed).x
    rl.draw_text_ex(
      self._font_bold,
      set_speed_text,
      rl.Vector2(x + (set_speed_width - speed_text_width) / 2, y + 77),
      FONT_SIZES.set_speed,
      0,
      set_speed_color,
    )

  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    self.speed_renderer.render(rect)

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)

    if ui_state.has_longitudinal_control:
      button_x = rect.x + rect.width - UI_BORDER_SIZE - EXP_BUTTON_SIZE
      button_y = rect.y + UI_BORDER_SIZE
      self._speed_assist_button.render(rl.Rectangle(button_x, button_y, EXP_BUTTON_SIZE, EXP_BUTTON_SIZE))
    self._long_indicator.render(rect)
    if ui_state.confidence_ball:
      self._confidence_ball.render(rect)

    # always rendered: lane center's gauge shows under it even with the bar itself off
    torque_rect = rect
    if ui_state.developer_ui in (DeveloperUiState.BOTTOM, DeveloperUiState.BOTH):
      torque_rect = rl.Rectangle(rect.x, rect.y, rect.width, rect.height - get_bottom_dev_ui_offset())
    self._torque_bar.render(torque_rect)

    self.developer_ui.render(rect)
    self.road_name_renderer.render(rect)
    self.speed_limit_renderer.render(rect)
    self.smart_cruise_control_renderer.render(rect)
    self.turn_signal_controller.render(rect)
    self.circular_alerts_renderer.render(rect)
    self.rocket_fuel.render(rect, ui_state.sm)
    self._draw_auto_overtake_badge(rect)

  def _draw_auto_overtake_badge(self, rect: rl.Rectangle) -> None:
    """AO badge, tappable while onroad. Dimmed 'AO OFF' when AutoOvertake is disabled.
    Just below the MAX/set-speed box on the left, away from the speed and
    the top-right Set Speed Assist button."""
    if not ui_state.is_onroad():
      return
    self._ao_badge.place(rect.x + 60, rect.y + 45 + UI_CONFIG.set_speed_height + 10)
    self._ao_badge.render()

  def user_interacting(self) -> bool:
    return self._speed_assist_button.is_pressed or self._ao_badge.is_pressed
