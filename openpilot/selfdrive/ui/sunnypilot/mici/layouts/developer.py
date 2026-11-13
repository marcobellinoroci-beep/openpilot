"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.mici.layouts.settings.developer import DeveloperLayoutMici
from openpilot.selfdrive.ui.mici.widgets.button import BigButton
from openpilot.selfdrive.ui.mici.widgets.dialog import BigInputDialog
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app


def mapbox_token_value(token: str) -> str:
  """SET when empty, EDIT when set, with a masked preview only: never the whole token."""
  if not token:
    return "SET"
  return f"EDIT  {token[:6]}…{token[-4:]}" if len(token) > 12 else "EDIT"


class DeveloperLayoutMiciSP(DeveloperLayoutMici):
  def __init__(self):
    super().__init__()
    # Jetlink holds the USB port that ADB needs
    self._adb_toggle.set_enabled(lambda: ui_state.is_offroad() and not ui_state.adb_blocked)
    # the alpha switch lives in the alpha longitudinal panel
    self._scroller.items.remove(self._alpha_long_toggle)

    # Navigation: own Mapbox token for navd (MAPBOX_TOKEN env var still wins when set), as on the big UI
    self._mapbox_token_btn = BigButton("mapbox token", mapbox_token_value(self._mapbox_token()),
                                       description="Your own Mapbox access token (pk.…) for navigation routes. " +
                                                   "Confirm an empty field to remove it.")
    self._mapbox_token_btn.set_click_callback(self._on_mapbox_token_clicked)
    self._scroller.add_widget(self._mapbox_token_btn)

  @staticmethod
  def _mapbox_token() -> str:
    token = ui_state.params.get("MapboxToken")
    return token.strip() if isinstance(token, str) else ""

  def show_event(self):
    super().show_event()
    self._mapbox_token_btn.set_value(mapbox_token_value(self._mapbox_token()))

  def _on_mapbox_token_clicked(self):
    def on_confirm(text: str):
      token = text.strip()
      if token:
        ui_state.params.put("MapboxToken", token)
      else:
        ui_state.params.remove("MapboxToken")
      self._mapbox_token_btn.set_value(mapbox_token_value(token))

    # starts empty so the saved token is never shown in full
    gui_app.push_widget(BigInputDialog("enter Mapbox token...", "", minimum_length=0, confirm_callback=on_confirm))
