import json
import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
UI_ROOT = REPO_ROOT / "starpilot/system/the_galaxy/assets/mobile"
LAYOUT_PATH = REPO_ROOT / "starpilot/common/assets/device_settings_layout.json"
CATALOG_JS = UI_ROOT / "js/i18n_it_settings.js"
I18N_JS = UI_ROOT / "js/i18n.js"
SELECT_JS = UI_ROOT / "js/components/GalaxySelect.js"

FIELDS = {
  "label", "description", "placeholder", "action_label", "unit", "hint", "title",
  "name", "picker_description", "disabled_reason", "confirm_message",
}

CHROME = {
  "Choose an option",
  "Disabled by another setting.",
  "Required setting is off.",
  "Required configuration missing.",
  "Search toggles",
  "Search toggles...",
  "Loading configuration...",
  "No settings in this section.",
  "No settings available.",
  "No options available",
  "Working...",
  "Language",
  "Fine scrubbing",
  "Hold to fine scrub",
  "to",
}

# Units, brands, and labels intentionally left identical in Italian.
ALLOW_SAME = {
  "Marco", " s", " m/s²", "%", "x", " seconds", "Eco", "Sport", "Sport+",
  "Standard", "Auto", "Off", "Goat Scream", "CAN 0", "CAN 2", "GM Auto Hold",
  "Stop and Go", "Toyota Auto Hold", "Pulse and Glide", "Redneck Cruise",
  "Subaru Redneck Cruise", "Galaxy", "Home", "Menu", "Offline",
  "Neural Network Feedforward (NNFF)", "Neural Network Feedforward (NNFF) Lite",
}


def _layout_strings():
  layout = json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))
  seen = []
  seen_set = set()

  def add(value):
    if isinstance(value, str) and value.strip() and value not in seen_set:
      seen_set.add(value)
      seen.append(value)

  def walk(obj):
    if isinstance(obj, dict):
      for key, value in obj.items():
        if key in FIELDS:
          add(value)
        if key in ("options", "choices") and isinstance(value, list):
          for item in value:
            if isinstance(item, str):
              add(item)
            elif isinstance(item, dict):
              for field in FIELDS:
                add(item.get(field))
        walk(value)
    elif isinstance(obj, list):
      for item in obj:
        walk(item)

  walk(layout)
  return seen


def _catalog():
  text = CATALOG_JS.read_text(encoding="utf-8")
  start = text.index("{")
  end = text.rindex("}")
  return json.loads(text[start:end + 1])


def test_i18n_js_loads_generated_italian_settings_catalog():
  i18n = I18N_JS.read_text(encoding="utf-8")
  assert 'import { IT_SETTINGS } from "./i18n_it_settings.js"' in i18n
  assert "Object.assign(TRANSLATIONS.it, IT_SETTINGS, handwrittenIt)" in i18n
  assert 't("Choose an option")' in SELECT_JS.read_text(encoding="utf-8")
  # Param keys must stay in English in the layout; catalog is phrase-based.
  catalog = _catalog()
  assert "CarMake" not in catalog
  assert "LanguageSetting" not in catalog



# Runtime labels/descriptions written by resolveVehicleUnitParam in params.js.
# These overwrite the mph layout strings when IsMetric is on, and use slightly
# different English than the layout ("speed limits" and an en-dash range).
SPEED_OFFSET_RANGES = {
  "imperial": ["0–24", "25–34", "35–44", "45–54", "55–64", "65–74", "75–99"],
  "metric": ["0–29", "30–49", "50–59", "60–79", "80–99", "100–119", "120–140"],
}
SPEED_OFFSET_UNITS = {"imperial": "mph", "metric": "km/h"}


def _speed_offset_runtime_strings():
  strings = []
  for mode, ranges in SPEED_OFFSET_RANGES.items():
    unit = SPEED_OFFSET_UNITS[mode]
    for band in ranges:
      strings.append(f"Speed Offset ({band} {unit})")
      strings.append(f"How much to offset posted speed limits between {band} {unit}.")
  return strings


def test_italian_catalog_covers_layout_and_settings_chrome():
  catalog = _catalog()
  missing = []
  still_english = []
  for source in _layout_strings() + sorted(CHROME):
    if source not in catalog:
      missing.append(source)
      continue
    if catalog[source] == source and source not in ALLOW_SAME:
      still_english.append(source)
  assert missing == []
  assert still_english == []



def test_italian_catalog_covers_runtime_speed_offset_bands():
  catalog = _catalog()
  missing = []
  still_english = []
  for source in _speed_offset_runtime_strings():
    if source not in catalog:
      missing.append(source)
      continue
    if catalog[source] == source:
      still_english.append(source)
  assert missing == []
  assert still_english == []
  assert catalog["Speed Offset (0–29 km/h)"] == "Offset velocità (0–29 km/h)"
  assert catalog["to"] == "a"

def test_italian_catalog_translates_reported_english_labels():
  catalog = _catalog()
  assert catalog["Car Make"] == "Marca dell'auto"
  assert catalog["Select your car make."] == "Seleziona la marca della tua auto."
  assert catalog["Choose an option"] == "Scegli un'opzione"
  assert catalog["Disable Automatic Fingerprint Detection"] == "Disattiva il rilevamento automatico dell'impronta"
  assert catalog["Right Hand Driving"] == "Guida a destra"
  assert "Ã" not in catalog["Start Speed"]
  assert catalog["Start Speed"] == "Velocità di partenza"
