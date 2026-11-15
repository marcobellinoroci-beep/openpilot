from pathlib import Path

import pytest

from openpilot.system.ui.lib import multilang as multilang_mod
from openpilot.system.ui.lib.multilang import Multilang, multilang, tr


REPO_ROOT = Path(__file__).resolve().parents[4]
BUTTON_PY = REPO_ROOT / "selfdrive/ui/mici/widgets/button.py"
SLIDER_PY = REPO_ROOT / "system/ui/widgets/slider.py"


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


@pytest.mark.parametrize("setting", ["main_it", b"main_it", "it"])
def test_device_ui_stays_english_when_language_setting_is_italian(setting):
  previous = _activate(setting)
  try:
    assert multilang.language == "en"
    assert tr("toggles") == "toggles"
    assert tr("right hand driving") == "right hand driving"
    assert tr("Car Make") == "Car Make"
    assert tr("MyHomeSSID") == "MyHomeSSID"
  finally:
    _restore(previous)


def test_change_language_writes_setting_but_keeps_device_english():
  params = FakeParams("main_en")
  previous = (multilang._params, multilang._language, dict(multilang._translations),
              dict(multilang._plurals), multilang._plural_selector)
  try:
    multilang._params = params
    multilang.change_language("main_it")
    assert params.values["LanguageSetting"] == "main_it"
    assert multilang.language == "en"
    assert tr("toggles") == "toggles"
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


def test_mici_buttons_and_sliders_do_not_translate_at_render_time():
  button = BUTTON_PY.read_text(encoding="utf-8")
  slider = SLIDER_PY.read_text(encoding="utf-8")
  assert "from openpilot.system.ui.lib.multilang import tr" not in button
  assert "lambda: tr(" not in button
  assert "from openpilot.system.ui.lib.multilang import tr" not in slider
  assert "lambda: tr(" not in slider


def test_bytes_language_setting_does_not_switch_device_ui(monkeypatch):
  monkeypatch.setattr(multilang_mod, "Params", lambda: FakeParams(b"main_it"))
  fresh = Multilang()
  fresh.setup()
  assert fresh.language == "en"
  assert fresh.tr("toggles") == "toggles"
