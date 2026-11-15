"""Exercise the device Params overlay wrapper with a stale params_pyx stand-in.

This file must be collected in its own process (it installs a fake
openpilot.common.params_pyx before importing Params). Run with
pytest -c /dev/null --noconftest.
"""
from __future__ import annotations

import enum
import json
import re
import sys
import types
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
FORK_KEYS = ("AutoOvertake", "AutoOvertakeReturn", "OvertakeReturnDelay", "NavLaneChange")


def _install_fake_pyx():
  if "openpilot.common.params" in sys.modules:
    pytest.skip("openpilot.common.params already imported; run this file alone with --noconftest")

  repo = str(ROOT)
  if repo not in sys.path:
    sys.path.insert(0, repo)

  op = sys.modules.get("openpilot")
  if op is None:
    op = types.ModuleType("openpilot")
    op.__path__ = [repo]
    sys.modules["openpilot"] = op

  class UnknownKeyName(Exception):
    pass

  class ParamKeyType(enum.IntEnum):
    STRING = 0
    BOOL = 1
    INT = 2
    FLOAT = 3
    TIME = 4
    JSON = 5
    BYTES = 6

  class ParamKeyFlag(enum.IntFlag):
    PERSISTENT = 0x02
    CLEAR_ON_MANAGER_START = 0x04
    CLEAR_ON_ONROAD_TRANSITION = 0x08
    CLEAR_ON_OFFROAD_TRANSITION = 0x10
    DONT_LOG = 0x20
    DEVELOPMENT_ONLY = 0x40
    CLEAR_ON_IGNITION_ON = 0x80
    ALL = 0xFFFFFFFF

  header = (ROOT / "common" / "params_keys.h").read_text(encoding="utf-8")
  native_known = [
    m.group(1)
    for m in re.finditer(r'\{"([A-Za-z0-9_]+)"', header)
    if m.group(1) not in FORK_KEYS
  ]

  class FakeNative:
    known = set(native_known)
    stores: dict[int, dict] = {}

    def __init__(self, d="", *, memory=False, return_defaults=False):
      self.d = d
      self.m = memory
      self.return_defaults = return_defaults
      self._store = {}

    def check_key(self, key):
      name = key.decode() if isinstance(key, bytes) else key
      if name not in self.known:
        raise UnknownKeyName(name)
      return name.encode() if isinstance(key, str) else key

    def all_keys(self):
      return [k.encode() for k in self.known]

    def get_param_path(self, key=""):
      base = self.d or ""
      return str(Path(base) / "d" / key) if key else str(Path(base) / "d")

    def get(self, key, block=False, return_default=False):
      name = key.decode() if isinstance(key, bytes) else key
      self.check_key(name)
      return self._store.get(name)

    def get_bool(self, key, block=False):
      value = self.get(key, block=block)
      return bool(value)

    def put(self, key, dat):
      name = key.decode() if isinstance(key, bytes) else key
      self.check_key(name)
      self._store[name] = dat

    def put_bool(self, key, val):
      self.put(key, val)

    def put_nonblocking(self, key, dat):
      self.put(key, dat)

    def put_bool_nonblocking(self, key, val):
      self.put_bool(key, val)

    def remove(self, key):
      name = key.decode() if isinstance(key, bytes) else key
      self.check_key(name)
      self._store.pop(name, None)

    def get_type(self, key):
      self.check_key(key)
      return ParamKeyType.STRING

    def get_default_value(self, key):
      self.check_key(key)
      return None

    def cpp2python(self, key, value):
      self.check_key(key)
      return value

    def get_key_flag(self, key):
      self.check_key(key)
      return ParamKeyFlag.PERSISTENT

    def get_stock_value(self, key):
      self.check_key(key)
      return None

    def get_tuning_level(self, key):
      self.check_key(key)
      return 0

    def get_settings_tier(self, key):
      self.check_key(key)
      return 1

    def clear_all(self, tx_flag=0):
      self._store.clear()

  pyx = types.ModuleType("openpilot.common.params_pyx")
  pyx.Params = FakeNative
  pyx.UnknownKeyName = UnknownKeyName
  pyx.ParamKeyType = ParamKeyType
  pyx.ParamKeyFlag = ParamKeyFlag
  sys.modules["openpilot.common.params_pyx"] = pyx
  return FakeNative, UnknownKeyName


FakeNative, UnknownKeyName = _install_fake_pyx()

from openpilot.common.params import Params  # noqa: E402
import openpilot.common.params_declared as pd  # noqa: E402
from openpilot.common.params_declared import OVERLAY_FILENAME  # noqa: E402


@pytest.fixture
def param_root(tmp_path, monkeypatch):
  reads = {"n": 0}
  orig = pd._read_file

  def counting(path):
    reads["n"] += 1
    return orig(path)

  monkeypatch.setattr(pd, "_read_file", counting)
  FakeNative.known = set(FakeNative.known)  # keep class default
  return tmp_path, reads


def test_native_gets_do_not_reread_or_flock_the_overlay(param_root, monkeypatch):
  tmp_path, reads = param_root
  flocks = {"n": 0}
  orig_locked = pd.DeclaredKeyOverlay._locked

  def counting_locked(self):
    flocks["n"] += 1
    return orig_locked(self)

  monkeypatch.setattr(pd.DeclaredKeyOverlay, "_locked", counting_locked)

  params = Params(str(tmp_path))
  params.put_bool("AutoOvertake", True)
  reads["n"] = 0
  flocks["n"] = 0

  for _ in range(10000):
    assert params.get("IsMetric") is None
  # First native get may load the overlay once via contains_cached; never again.
  assert reads["n"] <= 1
  assert flocks["n"] == 0


def test_fork_key_check_is_cached(param_root):
  tmp_path, _reads = param_root
  params = Params(str(tmp_path))
  raises = {"n": 0}
  orig = FakeNative.check_key

  def counting(self, key):
    try:
      return orig(self, key)
    except Exception:
      raises["n"] += 1
      raise

  FakeNative.check_key = counting
  try:
    params.put_bool("AutoOvertake", True)
    raises["n"] = 0
    for _ in range(1000):
      assert params.get_bool("AutoOvertake") is True
    # First get may check once; later hits must use the per-instance cache.
    assert raises["n"] <= 1
  finally:
    FakeNative.check_key = orig


def test_overlay_migrates_once_when_new_instance_learns_the_key(param_root):
  tmp_path, _reads = param_root
  stale = Params(str(tmp_path))
  stale.put_bool("AutoOvertake", True)
  overlay = tmp_path / OVERLAY_FILENAME
  assert json.loads(overlay.read_text())["AutoOvertake"] == "1"

  FakeNative.known.add("AutoOvertake")
  try:
    upgraded = Params(str(tmp_path))
    assert upgraded._use_overlay("AutoOvertake") is False
    assert upgraded.get_bool("AutoOvertake") is True
    stored = json.loads(overlay.read_text()) if overlay.exists() else {}
    assert "AutoOvertake" not in stored
    assert upgraded._store.get("AutoOvertake") in (True, "1", 1)
  finally:
    FakeNative.known.discard("AutoOvertake")


def test_manager_style_all_keys_sync_is_not_quadratic(param_root, monkeypatch):
  tmp_path, reads = param_root
  params = Params(str(tmp_path))
  params.put_bool("NavLaneChange", True)
  reads["n"] = 0
  # Matches manager.py: for k in params.all_keys(): params.get(k)
  for key in params.all_keys():
    params.get(key)
  assert reads["n"] <= 1
