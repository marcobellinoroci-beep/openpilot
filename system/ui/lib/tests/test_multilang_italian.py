from pathlib import Path

import pytest

from openpilot.system.ui.lib import multilang as multilang_mod
from openpilot.system.ui.lib.multilang import Multilang, multilang, tr


REPO_ROOT = Path(__file__).resolve().parents[4]
BUTTON_PY = REPO_ROOT / "selfdrive/ui/mici/widgets/button.py"
SLIDER_PY = REPO_ROOT / "system/ui/widgets/slider.py"
PO_PATH = REPO_ROOT / "selfdrive/ui/translations/app_it.po"

SAMPLE_EXPECT = {
  "toggles": "opzioni",
  "network": "rete",
  "device": "dispositivo",
  "right hand driving": "guida a destra",
  "disable auto fingerprint": "disattiva impronta automatica",
  "car make": "marca",
  "use metric units": "usa unità metriche",
  "experimental mode": "modalità sperimentale",
  "software": "software",
  "offroad": "fermo",
  "onroad": "in guida",
}


class FakeParams:
  def __init__(self, language="main_it"):
    self.values = {"LanguageSetting": language}

  def get(self, key):
    return self.values.get(key)

  def put(self, key, value):
    self.values[key] = value


def _activate(language):
  previous = (multilang._params, multilang._language, dict(multilang._translations),
              dict(multilang._plurals), multilang._plural_selector)
  multilang._params = FakeParams(language)
  multilang._load_languages()
  multilang.setup()
  return previous


def _restore(previous):
  (multilang._params, multilang._language, translations, plurals, selector) = previous
  multilang._translations = translations
  multilang._plurals = plurals
  multilang._plural_selector = selector


@pytest.fixture
def italian():
  previous = _activate("main_it")
  try:
    yield
  finally:
    _restore(previous)


@pytest.mark.parametrize("setting", ["main_it", b"main_it"])
def test_language_setting_main_it_loads_italian_po(setting):
  previous = _activate(setting)
  try:
    assert multilang.language == "it"
    assert tr("toggles") == "opzioni"
    assert tr("right hand driving") == "guida a destra"
    assert tr("disable auto fingerprint") == "disattiva impronta automatica"
    for source, expected in SAMPLE_EXPECT.items():
      assert tr(source) == expected, source
    # unknown strings pass through unchanged (e.g. SSIDs, dongle IDs)
    assert tr("MyHomeSSID") == "MyHomeSSID"
  finally:
    _restore(previous)


def test_strip_prefix_accepts_bytes_and_str():
  assert Multilang._strip_prefix("main_it") == "it"
  assert Multilang._strip_prefix(b"main_it") == "it"
  assert Multilang._strip_prefix(None) == ""
  assert Multilang._strip_prefix("it") == "it"


def test_po_path_fallback_to_basedir(monkeypatch, tmp_path):
  fallback = tmp_path / "selfdrive" / "ui" / "translations"
  fallback.mkdir(parents=True)
  (fallback / "app_xx.po").write_text('msgid "hello"\nmsgstr "ciao"\n', encoding="utf-8")

  monkeypatch.setattr(multilang_mod, "TRANSLATIONS_DIR_FALLBACK", fallback)
  monkeypatch.setattr(multilang_mod, "_is_file", lambda path: False)
  path = multilang_mod._po_path("xx")
  assert path == fallback / "app_xx.po"
  translations, _plurals = multilang_mod.load_translations(path)
  assert translations["hello"] == "ciao"


def test_mici_buttons_and_sliders_translate_at_display_time():
  button = BUTTON_PY.read_text(encoding="utf-8")
  slider = SLIDER_PY.read_text(encoding="utf-8")
  assert "from openpilot.system.ui.lib.multilang import tr" in button
  assert "lambda: tr(self.text) if self.text else \"\"" in button
  assert "lambda: tr(self.value) if self.value else \"\"" in button
  assert "from openpilot.system.ui.lib.multilang import tr" in slider
  assert "lambda: tr(self._title) if self._title else \"\"" in slider


def test_mici_settings_strings_are_in_app_it_po(italian):
  # Core hub/toggle/vehicle labels that were previously hardcoded English.
  for source, expected in SAMPLE_EXPECT.items():
    assert source in multilang._translations
    assert tr(source) == expected


def test_bytes_language_setting_on_fresh_multilang(monkeypatch):
  monkeypatch.setattr(multilang_mod, "Params", lambda: FakeParams(b"main_it"))
  fresh = Multilang()
  fresh.setup()
  assert fresh.language == "it"
  assert fresh.tr("toggles") == "opzioni"
