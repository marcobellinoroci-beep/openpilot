"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import datetime
import os
import time
from pathlib import Path

from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.layouts.settings.developer import DeveloperLayout
from openpilot.common.hardware import PC
from openpilot.common.hardware.hw import Paths
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import button_item, text_item

from openpilot.system.ui.sunnypilot.widgets.html_render import HtmlModalSP
from openpilot.system.ui.sunnypilot.widgets.input_dialog import InputDialogSP
from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp
from openpilot.sunnypilot.selfdrive.navigation.tesla_nav_status import STATUS_PARAM as TESLA_NAV_STATUS_PARAM, format_status

PREBUILT_PATH = os.path.join(Paths.comma_home(), "prebuilt") if PC else "/data/openpilot/prebuilt"
TESLA_NAV_STATUS_REFRESH = 1.0  # s between param reads for the "Navigatore Tesla" row


class DeveloperLayoutSP(DeveloperLayout):
  def __init__(self):
    super().__init__()
    # Jetlink holds the USB port that ADB needs
    self._adb_toggle.action_item.set_enabled(lambda: ui_state.is_offroad() and not ui_state.adb_blocked)
    # zoompilot: the alpha switch lives in the Alpha Longitudinal panel
    self._scroller._items.remove(self._alpha_long_toggle)
    self.error_log_path = os.path.join(Paths.crash_log_root(), "error.log")
    self._is_release_branch: bool = self._is_release or ui_state.params.get_bool("IsReleaseSpBranch")
    self._is_development_branch: bool = ui_state.params.get_bool("IsTestedBranch") or ui_state.params.get_bool("IsDevelopmentBranch")
    self._initialize_items()

    for item in self.items:
      self._scroller.add_widget(item)

  def _initialize_items(self):
    self.show_advanced_controls = toggle_item_sp(tr("Show Advanced Controls"),
                                                 tr("Toggle visibility of advanced zoompilot controls.<br>This only changes the visibility of the toggles; " +
                                                    "it does not change the actual enabled/disabled state."), param="ShowAdvancedControls")

    self.enable_github_runner_toggle = toggle_item_sp(tr("GitHub Runner Service"), tr("Enables or disables the GitHub runner service."),
                                                      param="EnableGithubRunner")

    self.enable_copyparty_toggle = toggle_item_sp(tr("copyparty Service"),
                                                  tr("copyparty is a very capable file server, you can use it to download your routes, view your logs " +
                                                     "and even make some edits on some files from your browser. " +
                                                     "Requires you to connect to your comma locally via its IP address."), param="EnableCopyparty")

    self.prebuilt_toggle = toggle_item_sp(tr("Quickboot Mode"), "", param="QuickBootToggle", callback=self._on_prebuilt_toggled)

    self.error_log_btn = button_item(tr("Error Log"), tr("VIEW"), tr("View the error log for zoompilot crashes."), callback=self._on_error_log_clicked)

    # Navigation: own Mapbox token for navd (MAPBOX_TOKEN env var still wins when set).
    self.mapbox_token_btn = button_item(tr("Mapbox Token"), self._mapbox_token_button_text, self._mapbox_token_description,
                                        callback=self._on_mapbox_token_clicked)

    # Navigation: destination typed here. navd geocodes it with Mapbox and sets NavDestination when onroad.
    self.nav_destination_btn = button_item(tr("Destinazione"), self._nav_destination_button_text, self._nav_destination_description,
                                           callback=self._on_nav_destination_clicked)

    # Navigation: read-only live status of tesla_navd (TeslaNavStatus), for checking without SSH/connect.
    self._tesla_nav_status_text = ""
    self._tesla_nav_status_time = 0.0
    self.tesla_nav_status_item = text_item(tr("Navigatore Tesla"), self._tesla_nav_status,
                                           tr("Stato della lettura delle uscite dal navigatore Tesla (Usa navigatore Tesla). " +
                                              "Solo lettura, si aggiorna ogni secondo."))

    # zoompilot: no Error Log row, nothing writes error.log since sentry was removed
    self.items: list = [self.show_advanced_controls, self.enable_github_runner_toggle, self.enable_copyparty_toggle, self.prebuilt_toggle,
                        self.mapbox_token_btn, self.nav_destination_btn, self.tesla_nav_status_item,]

  def _tesla_nav_status(self) -> str:
    # Called every frame by the row: read the params at most once per second.
    now = time.monotonic()
    if not self._tesla_nav_status_text or now - self._tesla_nav_status_time >= TESLA_NAV_STATUS_REFRESH:
      self._tesla_nav_status_time = now
      try:
        status = ui_state.params.get(TESLA_NAV_STATUS_PARAM)
      except Exception:
        status = None
      self._tesla_nav_status_text = tr(format_status(ui_state.params.get_bool("TeslaNavSource"), status, ui_state.started))
    return self._tesla_nav_status_text

  @staticmethod
  def _mapbox_token() -> str:
    token = ui_state.params.get("MapboxToken")
    return token.strip() if isinstance(token, str) else ""

  def _mapbox_token_button_text(self) -> str:
    return tr("EDIT") if self._mapbox_token() else tr("SET")

  def _mapbox_token_description(self) -> str:
    token = self._mapbox_token()
    status = tr("Set") + f" ({token[:6]}…{token[-4:]})" if len(token) > 12 else (tr("Set") if token else tr("Not set"))
    return tr("Your own Mapbox access token (pk.…) for navigation routes. Leave empty and confirm to remove it.") + \
      f"<br><br>{tr('Status')}: {status}"

  @staticmethod
  def _nav_pending() -> dict:
    pending = ui_state.params.get("NavDestinationPending")
    return pending if isinstance(pending, dict) and str(pending.get("query") or "").strip() else {}

  def _nav_destination_button_text(self) -> str:
    return tr("MODIFICA") if self._nav_pending() else tr("IMPOSTA")

  def _nav_destination_description(self) -> str:
    pending = self._nav_pending()
    intro = tr("Scrivi un indirizzo o un luogo. Viene cercato con Mapbox quando l'auto è accesa e la navigazione parte da sola. " +
               "Servono Navigazione (Alpha) attiva e il Mapbox Token. Conferma un campo vuoto per cancellarla.")
    if not pending:
      return intro + "<br><br>" + tr("Stato") + ": " + tr("nessuna destinazione")
    query = str(pending.get("query"))
    status = pending.get("status")
    message = str(pending.get("message") or "")
    if status == "ok":
      state = tr("destinazione") + f": {pending.get('name') or query}"
    elif status == "error":
      state = tr("errore") + f" ({query}): {message or tr('nessun risultato')}"
    else:
      state = tr("in attesa") + f": \"{query}\""
      if message:
        state += f" - {message}"
      elif not self._mapbox_token():
        state += " - " + tr("manca il Mapbox Token")
      else:
        state += " - " + tr("verrà cercata quando l'auto è accesa")
    return intro + "<br><br>" + tr("Stato") + ": " + state

  def _on_nav_destination_clicked(self):
    def on_done(result: DialogResult, text: str):
      if result != DialogResult.CONFIRM:
        return
      query = " ".join(text.split())
      if query:
        ui_state.params.put("NavDestinationPending", {"query": query, "status": "pending"})
      else:
        ui_state.params.remove("NavDestinationPending")
        ui_state.params.remove("NavDestination")  # stop the current route too

    current = str(self._nav_pending().get("query") or "")
    dialog = InputDialogSP(tr("Destinazione"), tr("Indirizzo o luogo, es. Via Roma 1, Milano"), current_text=current, callback=on_done)
    dialog.show()

  def _on_mapbox_token_clicked(self):
    def on_done(result: DialogResult, text: str):
      if result != DialogResult.CONFIRM:
        return
      token = text.strip()
      if token:
        ui_state.params.put("MapboxToken", token)
      else:
        ui_state.params.remove("MapboxToken")

    dialog = InputDialogSP(tr("Mapbox Token"), tr("Paste or type your Mapbox access token"), current_text=self._mapbox_token(),
                           callback=on_done)
    dialog.show()

  @staticmethod
  def _on_prebuilt_toggled(state):
    if state:
      Path(PREBUILT_PATH).touch(exist_ok=True)
    else:
      os.remove(PREBUILT_PATH)
    ui_state.params.put_bool("QuickBootToggle", state)

  def _on_delete_confirm(self, result):
    if result == DialogResult.CONFIRM:
      if os.path.exists(self.error_log_path):
        os.remove(self.error_log_path)

  def _on_error_log_closed(self, result, log_exists):
    if result == DialogResult.CONFIRM and log_exists:
      dialog2 = ConfirmDialog(tr("Would you like to delete this log?"), tr("Yes"), tr("No"), rich=False, callback=self._on_delete_confirm)
      gui_app.push_widget(dialog2)

  def _on_error_log_clicked(self):
    text = ""
    if os.path.exists(self.error_log_path):
      text = f"<b>{datetime.datetime.fromtimestamp(os.path.getmtime(self.error_log_path)).strftime('%d-%b-%Y %H:%M:%S').upper()}</b><br><br>"
      try:
        with open(self.error_log_path) as file:
          text += file.read()
      except Exception:
        pass
    dialog = HtmlModalSP(text=text, callback=lambda result: self._on_error_log_closed(result, os.path.exists(self.error_log_path)))
    gui_app.push_widget(dialog)

  def _update_state(self):
    disable_updates = ui_state.params.get_bool("DisableUpdates")
    show_advanced = ui_state.params.get_bool("ShowAdvancedControls")

    if (prebuilt_file := os.path.exists(PREBUILT_PATH)) != ui_state.params.get_bool("QuickBootToggle"):
      ui_state.params.put_bool("QuickBootToggle", prebuilt_file)
      self.prebuilt_toggle.action_item.set_state(prebuilt_file)

    self.prebuilt_toggle.set_visible(show_advanced and not (self._is_release_branch or self._is_development_branch))
    self.prebuilt_toggle.action_item.set_enabled(disable_updates)

    if disable_updates:
      self.prebuilt_toggle.set_description(tr("When toggled on, this creates a prebuilt file to allow accelerated boot times. When toggled off, it " +
                                              "removes the prebuilt file so compilation of locally edited cpp files can be made."))
    else:
      self.prebuilt_toggle.set_description(tr("Quickboot mode requires updates to be disabled.<br>Enable 'Disable Updates' in the Software panel first."))

    self.enable_copyparty_toggle.set_visible(show_advanced)
    self.enable_github_runner_toggle.set_visible(show_advanced and not self._is_release_branch)
    self.error_log_btn.set_visible(not self._is_release_branch)
