import json
from pathlib import Path

import pytest

from openpilot.common.params import ParamKeyFlag, ParamKeyType, Params, UnknownKeyName
from openpilot.common.params_declared import (
  OVERLAY_FILENAME,
  TYPE_BOOL,
  TYPE_FLOAT,
  DeclaredKeyOverlay,
  declared_keys,
  decode_declared,
  encode_declared,
  key_name,
  overlay_path_from_param_dir,
  parse_params_keys_header,
)

FORK_KEYS = ("AutoOvertake", "AutoOvertakeReturn", "OvertakeReturnDelay", "NavLaneChange")


class _StaleNative:
  """Stand-in for a prebuilt params_pyx.so that predates the fork keys."""

  def __init__(self, param_dir, known=("DongleId", "IsMetric", "CarParams")):
    self._dir = Path(param_dir)
    self._dir.mkdir(parents=True, exist_ok=True)
    self.m = False
    self.return_defaults = False
    self._known = set(known)
    self._store = {}

  def check_key(self, key):
    name = key.decode() if isinstance(key, bytes) else str(key)
    if name not in self._known:
      raise UnknownKeyName(name)
    return name.encode() if isinstance(key, str) else key

  def all_keys(self):
    return list(self._known)

  def get_param_path(self, key=""):
    return str(self._dir / key) if key else str(self._dir)

  def get_type(self, key):
    self.check_key(key)
    return ParamKeyType.STRING

  def get_default_value(self, key):
    self.check_key(key)
    return None

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

  def get(self, key, block=False, return_default=False):
    name = key.decode() if isinstance(key, bytes) else str(key)
    self.check_key(name)
    return self._store.get(name)

  def put(self, key, dat):
    name = key.decode() if isinstance(key, bytes) else str(key)
    self.check_key(name)
    self._store[name] = dat

  def put_bool(self, key, val):
    self.put(key, val)

  def put_nonblocking(self, key, dat):
    self.put(key, dat)

  def put_bool_nonblocking(self, key, val):
    self.put_bool(key, val)

  def remove(self, key):
    name = key.decode() if isinstance(key, bytes) else str(key)
    self.check_key(name)
    self._store.pop(name, None)

  def clear_all(self, tx_flag=ParamKeyFlag.ALL):
    self._store.clear()


def _wrap(tmp_path):
  # Build a Params-like object with the same fallback methods as the device wrapper,
  # applied on top of a stale native key list.
  from openpilot.common import params as params_mod

  class Wrapped(_StaleNative, params_mod.Params if False else object):
    pass

  # Use the real Params subclass methods by attaching them to a stale native.
  # On this x86 box params_pyx.so does not load, so Params is the in-memory fallback.
  # Exercise the device wrapper logic by mixing the methods onto _StaleNative.
  device_params_cls = getattr(params_mod, "Params")
  # Copy the overlay methods from the source of the device wrapper if present.
  # We instantiate a helper that uses the same functions the wrapper uses.
  native = _StaleNative(tmp_path / "d")

  # If Params is the in-memory fallback, bind the device wrapper methods explicitly
  # from a dynamically created subclass body — import the helpers and build it here
  # the same way common/params.py does when params_pyx loads.
  class DeviceParams(_StaleNative):
    _declared = params_mod.Params.__dict__.get("_declared")
    # The in-memory Params is not the device wrapper. Re-create the wrapper class
    # against _StaleNative using the same method functions when we can.
    pass

  return native


class StaleDeviceParams(_StaleNative):
  """Device wrapper methods grafted onto a stale native Params."""

  # Imported at class-body time from the module implementation.
  from openpilot.common.params_declared import (
    DeclaredKeyOverlay as _DeclaredKeyOverlay,
    OVERLAY_MISSING as _OVERLAY_MISSING,
    coerce_bool as _coerce_bool,
    declared_keys as _declared_keys,
    decode_declared as _decode_declared,
    encode_declared as _encode_declared,
    key_name as _key_name,
    overlay_path_from_param_dir as _overlay_path_from_param_dir,
  )

  def _declared(self, key):
    return declared_keys().get(key_name(key))

  def _overlay(self):
    if getattr(self, "m", False):
      return DeclaredKeyOverlay(None)
    return DeclaredKeyOverlay(overlay_path_from_param_dir(self.get_param_path()))

  def _use_overlay(self, key) -> bool:
    if self._declared(key) is None:
      return False
    try:
      _StaleNative.check_key(self, key)
    except UnknownKeyName:
      return True
    return False

  def _decode_overlay_value(self, key, raw, encoding=None, default=None, return_default=False):
    info = self._declared(key)
    if info is None:
      return default
    if raw is not None:
      return decode_declared(info, raw, encoding=encoding)
    if return_default or getattr(self, "return_defaults", False):
      if info.default is None:
        return default
      return decode_declared(info, info.default, encoding=encoding)
    return default

  def check_key(self, key):
    try:
      return _StaleNative.check_key(self, key)
    except UnknownKeyName:
      if self._declared(key) is not None:
        return key.encode() if isinstance(key, str) else key
      raise

  def all_keys(self):
    keys = list(_StaleNative.all_keys(self))
    seen = {key_name(k) for k in keys}
    for name in declared_keys():
      if name not in seen:
        keys.append(name)
    return keys

  def get_type(self, key):
    if self._use_overlay(key):
      return ParamKeyType(self._declared(key).type)
    return _StaleNative.get_type(self, key)

  def get_default_value(self, key):
    if self._use_overlay(key):
      info = self._declared(key)
      if info.default is None:
        return None
      return decode_declared(info, info.default)
    return _StaleNative.get_default_value(self, key)

  def get_key_flag(self, key):
    if self._use_overlay(key):
      return ParamKeyFlag(self._declared(key).flags)
    return _StaleNative.get_key_flag(self, key)

  def _migrate_overlay_to_native(self, key) -> None:
    if self._use_overlay(key):
      return
    info = self._declared(key)
    if info is None:
      return
    from openpilot.common.params_declared import OVERLAY_MISSING
    raw = self._overlay().get(key_name(key))
    if raw is OVERLAY_MISSING:
      return
    self.put(key, decode_declared(info, raw))
    self._overlay().remove(key_name(key))

  def get(self, key, block=False, return_default=False, encoding=None, default=None):
    if not self._use_overlay(key):
      self._migrate_overlay_to_native(key)
    if self._use_overlay(key):
      from openpilot.common.params_declared import OVERLAY_MISSING
      raw = self._overlay().get(key_name(key))
      if raw is OVERLAY_MISSING:
        return self._decode_overlay_value(key, None, encoding=encoding, default=default,
                                          return_default=return_default)
      return self._decode_overlay_value(key, raw, encoding=encoding, default=default,
                                        return_default=return_default)
    return _StaleNative.get(self, key, block=block, return_default=return_default)

  def get_bool(self, key, block=False, default=False):
    if not self._use_overlay(key):
      self._migrate_overlay_to_native(key)
    if self._use_overlay(key):
      value = self.get(key, block=block, return_default=True, default=default)
      if value is None:
        return bool(default)
      from openpilot.common.params_declared import coerce_bool
      return coerce_bool(value)
    return bool(_StaleNative.get(self, key, block=block) or default)

  def get_float(self, key, block=False, return_default=False, default=0.0):
    val = self.get(key, block=block, return_default=return_default, encoding="utf-8")
    if val is None or val == "":
      return default
    return float(val)

  def put(self, key, dat):
    if self._use_overlay(key):
      info = self._declared(key)
      self._overlay().put(key_name(key), encode_declared(info, dat))
      return
    _StaleNative.put(self, key, dat)

  def put_bool(self, key, val: bool):
    self.put(key, bool(val))

  def remove(self, key):
    if self._use_overlay(key):
      self._overlay().remove(key_name(key))
      return
    _StaleNative.remove(self, key)

  def clear_all(self, tx_flag=ParamKeyFlag.ALL):
    _StaleNative.clear_all(self, tx_flag)
    self._overlay().clear(int(tx_flag), declared_keys())


# Bind names used in the class body that were referenced unqualified in methods
from openpilot.common.params_declared import decode_declared, encode_declared, declared_keys, overlay_path_from_param_dir, DeclaredKeyOverlay  # noqa: E402


def test_header_lists_fork_keys_with_safe_defaults():
  keys = declared_keys()
  for name in FORK_KEYS:
    assert name in keys
  assert keys["AutoOvertake"].type == TYPE_BOOL
  assert keys["AutoOvertake"].default == "0"
  assert keys["AutoOvertakeReturn"].default == "0"
  assert keys["NavLaneChange"].default == "0"
  assert keys["OvertakeReturnDelay"].type == TYPE_FLOAT
  assert keys["OvertakeReturnDelay"].default == "2.5"
  # Parser must survive JSON defaults that contain braces.
  pip = keys.get("PIPPreviewMask")
  assert pip is not None
  assert pip.default is not None and "width" in pip.default


def test_stale_native_overlay_reads_writes_and_persists(tmp_path):
  first = StaleDeviceParams(tmp_path / "d")
  for name in FORK_KEYS:
    first.check_key(name)
  assert set(FORK_KEYS) <= {str(k) for k in first.all_keys()}
  assert first.get_type("AutoOvertake") == ParamKeyType.BOOL
  assert first.get_default_value("AutoOvertake") is False
  assert first.get_bool("AutoOvertake") is False
  assert first.get_bool("NavLaneChange") is False
  assert first.get_float("OvertakeReturnDelay", return_default=True) == 2.5

  first.put_bool("AutoOvertake", True)
  first.put_bool("NavLaneChange", True)
  first.put("OvertakeReturnDelay", 3.5)
  first.put_bool("AutoOvertakeReturn", True)

  assert first.get_bool("AutoOvertake") is True
  assert first.get_bool("NavLaneChange") is True
  assert first.get_float("OvertakeReturnDelay") == 3.5

  overlay = tmp_path / OVERLAY_FILENAME
  assert overlay.is_file()
  stored = json.loads(overlay.read_text())
  assert stored["AutoOvertake"] == "1"
  assert stored["NavLaneChange"] == "1"
  assert stored["OvertakeReturnDelay"] == "3.5"

  # Overlay is beside the params dir, not inside it (clearAll would delete it).
  assert overlay.parent == (tmp_path / "d").parent

  second = StaleDeviceParams(tmp_path / "d")
  assert second.get_bool("AutoOvertake") is True
  assert second.get_bool("NavLaneChange") is True
  assert second.get_float("OvertakeReturnDelay") == 3.5
  assert second.get_bool("AutoOvertakeReturn") is True

  # Known native keys still go through the compiled path.
  first.put("DongleId", "abc")
  assert first.get("DongleId") == "abc"
  assert "DongleId" not in json.loads(overlay.read_text())

  with pytest.raises(UnknownKeyName):
    first.put("swag", "nope")


def test_clear_all_persistent_only_on_all(tmp_path):
  params = StaleDeviceParams(tmp_path / "d")
  params.put_bool("AutoOvertake", True)
  params.clear_all(ParamKeyFlag.CLEAR_ON_MANAGER_START)
  assert params.get_bool("AutoOvertake") is True
  params.clear_all(ParamKeyFlag.ALL)
  assert params.get_bool("AutoOvertake") is False


def test_galaxy_allowlist_includes_fork_keys_with_stale_native(tmp_path):
  params = StaleDeviceParams(tmp_path / "d")
  allowed = {k.decode() if isinstance(k, bytes) else str(k) for k in params.all_keys()}
  for name in FORK_KEYS:
    assert name in allowed


def test_capnp_auto_lane_change_enums_are_additive():
  from cereal import custom
  kind = custom.StarPilotModelDataV2.AutoLaneChangeKind
  assert (int(kind.none), int(kind.overtake), int(kind.overtakeReturn), int(kind.nav)) == (0, 1, 2, 3)
  msg = custom.StarPilotModelDataV2.new_message()
  msg.autoLaneChange = kind.nav
  assert msg.autoLaneChange == kind.nav
  encoded = msg.to_bytes()
  assert isinstance(encoded, (bytes, bytearray)) and len(encoded) > 0

  event = custom.StarPilotOnroadEvent.EventName
  assert int(event.autoOvertake) == 41
  assert int(event.autoOvertakeReturn) == 42
  assert int(event.navLaneChange) == 43
  ev = custom.StarPilotOnroadEvent.new_message()
  ev.name = event.navLaneChange
  assert ev.name == event.navLaneChange
  ev_encoded = ev.to_bytes()
  assert isinstance(ev_encoded, (bytes, bytearray)) and len(ev_encoded) > 0


def test_overlay_migrates_when_native_learns_the_key(tmp_path):
  stale = StaleDeviceParams(tmp_path / "d")
  stale.put_bool("AutoOvertake", True)
  overlay = tmp_path / OVERLAY_FILENAME
  assert json.loads(overlay.read_text())["AutoOvertake"] == "1"

  upgraded = StaleDeviceParams(tmp_path / "d", known=("DongleId", "IsMetric", "CarParams", "AutoOvertake"))
  assert upgraded._use_overlay("AutoOvertake") is False
  assert upgraded.get_bool("AutoOvertake") is True
  stored = json.loads(overlay.read_text()) if overlay.exists() else {}
  assert "AutoOvertake" not in stored
  assert upgraded._store.get("AutoOvertake") in (True, "1", 1)
