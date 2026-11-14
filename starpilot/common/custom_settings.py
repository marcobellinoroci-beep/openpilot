"""Keys for settings added in this fork, so the UI can mark them distinctly."""

CUSTOM_SETTING_KEYS = frozenset({
  "AutoOvertake",
  "AutoOvertakeReturn",
  "OvertakeReturnDelay",
  "NavLaneChange",
})

# Orange title/badge color used on C4 and in Galaxy.
CUSTOM_SETTING_TITLE_COLOR_HEX = "#FF9800"
CUSTOM_SETTING_TITLE_COLOR_RGBA = (255, 152, 0, 255)

# Dedicated settings section/panel for the fork-added keys above.
CUSTOM_SETTING_SECTION_NAME = "Marco"
