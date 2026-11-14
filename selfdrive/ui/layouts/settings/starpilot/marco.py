from __future__ import annotations

from openpilot.selfdrive.ui.layouts.settings.starpilot.panel import _SettingsPage
from openpilot.selfdrive.ui.layouts.settings.starpilot.aethergrid import (
  CUSTOM_SETTING_TITLE_COLOR,
  DEFAULT_PANEL_STYLE,
  AetherSettingsView,
  SettingRow,
  SettingSection,
)
from openpilot.starpilot.common.custom_settings import CUSTOM_SETTING_SECTION_NAME
from openpilot.system.ui.lib.multilang import tr, tr_noop


PANEL_STYLE = DEFAULT_PANEL_STYLE
_LANE_CHANGES_DISABLED_LABEL = tr_noop("Enable Lane Changes in Steering first.")


class StarPilotMarcoLayout(_SettingsPage):
  def __init__(self):
    super().__init__()
    self._build_view()

  def _build_view(self):
    p = self._params

    def lc_on():
      return p.get_bool("LaneChanges")

    def ao_on():
      return lc_on() and p.get_bool("AutoOvertake")

    rows = [
      SettingRow(
        "AutoOvertake", "toggle", tr_noop("Auto Overtake"),
        subtitle=tr_noop("Blinkerless left pass of a slower lead on the highway."),
        get_state=lambda: p.get_bool("AutoOvertake"),
        set_state=lambda s: p.put_bool("AutoOvertake", s),
        enabled=lc_on,
        disabled_label=_LANE_CHANGES_DISABLED_LABEL,
      ),
      SettingRow(
        "AutoOvertakeReturn", "toggle", tr_noop("Auto Return After Overtake"),
        subtitle=tr_noop("Blinkerless return to the right lane after an automatic overtake."),
        get_state=lambda: p.get_bool("AutoOvertakeReturn"),
        set_state=lambda s: p.put_bool("AutoOvertakeReturn", s),
        visible=ao_on,
      ),
      SettingRow(
        "OvertakeReturnDelay", "value", tr_noop("Overtake Return Delay"),
        subtitle=tr_noop("Wait this long after an overtake before signaling a return to the right."),
        get_value=self._get_overtake_return_delay_display,
        on_click=lambda: self._show_slider("OvertakeReturnDelay", 0.0, 10.0, step=0.5, unit="s", value_type="float"),
        visible=ao_on,
      ),
      SettingRow(
        "NavLaneChange", "toggle", tr_noop("Follow Route"),
        subtitle=tr_noop("On the highway, automatically change lanes toward an upcoming exit or keep-left/keep-right from navigation, without a turn signal."),
        get_state=lambda: p.get_bool("NavLaneChange"),
        set_state=lambda s: p.put_bool("NavLaneChange", s),
        enabled=lc_on,
        disabled_label=_LANE_CHANGES_DISABLED_LABEL,
      ),
    ]

    self._manager_view = AetherSettingsView(
      self,
      [SettingSection(title="", rows=rows)],
      header_title=tr_noop(CUSTOM_SETTING_SECTION_NAME),
      header_subtitle=tr_noop("Fork settings for automatic overtakes and following the navigation route."),
      header_title_color=CUSTOM_SETTING_TITLE_COLOR,
      panel_style=PANEL_STYLE,
    )

  def _get_overtake_return_delay_display(self) -> str:
    val = self._params.get_float("OvertakeReturnDelay")
    if val == 0.0:
      return tr("Instant")
    return f"{val:.1f}s"
