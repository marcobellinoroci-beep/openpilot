"""Keys for settings added in this fork, so the UI can mark them distinctly."""

CUSTOM_SETTING_KEYS = frozenset({
  # Solo sterzo: openpilot sterza, la velocità resta ai pedali (nessuna accelerazione da openpilot o TACC)
  "SteerOnlyMode",
  # Velocità minima dello sterzo: parametro esistente (QOL laterale), spostato nella sezione Marco
  "PauseLateralSpeed",
  "AutoOvertake",
  "AutoOvertakeReturn",
  "OvertakeReturnDelay",
  "NavLaneChange",
  # Speed Limit Controller settings shown in the Marco section of Galaxy
  # without Developer Mode. They are existing params: membership here only
  # drives the Marco section and the orange color; the fork-key overlay is
  # decided by params_declared.py from the prebuilt params library.
  "SpeedLimitController",
  "SLCPriority1",
  "SLCPriority2",
  "Offset1",
  "Offset2",
  "Offset3",
  "Offset4",
  "Offset5",
  "Offset6",
  "Offset7",
})

# Orange title/badge color used on C4 and in Galaxy.
CUSTOM_SETTING_TITLE_COLOR_HEX = "#FF9800"
CUSTOM_SETTING_TITLE_COLOR_RGBA = (255, 152, 0, 255)

# Dedicated settings section/panel for the fork-added keys above.
CUSTOM_SETTING_SECTION_NAME = "Marco"
