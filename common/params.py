from __future__ import annotations

from enum import IntEnum, IntFlag
from pathlib import Path
import tempfile

from openpilot.common.params_declared import (
  DeclaredKeyOverlay,
  OVERLAY_MISSING,
  coerce_bool,
  declared_keys,
  decode_declared,
  encode_declared,
  key_name,
  overlay_path_from_param_dir,
)

SETTINGS_SIMPLE = 0
SETTINGS_ADVANCED = 1


def _load_settings_tiers() -> dict[str, int]:
  params_keys = Path(__file__).with_name("params_keys.h")
  if not params_keys.exists():
    return {}

  tiers = {}
  for line in params_keys.read_text(encoding="utf-8", errors="ignore").splitlines():
    if not line.lstrip().startswith('{"'):
      continue
    parts = line.split('"')
    if len(parts) >= 2:
      tiers[parts[1]] = SETTINGS_SIMPLE if "SETTINGS_SIMPLE" in line else SETTINGS_ADVANCED
  return tiers


_SETTINGS_TIERS = _load_settings_tiers()

try:
  from openpilot.common.params_pyx import Params as _Params, ParamKeyFlag, ParamKeyType, UnknownKeyName
except Exception:
  class UnknownKeyName(KeyError):
    pass

  class ParamKeyFlag(IntFlag):
    PERSISTENT = 0x02
    CLEAR_ON_MANAGER_START = 0x04
    CLEAR_ON_ONROAD_TRANSITION = 0x08
    CLEAR_ON_OFFROAD_TRANSITION = 0x10
    DONT_LOG = 0x20
    DEVELOPMENT_ONLY = 0x40
    CLEAR_ON_IGNITION_ON = 0x80
    ALL = 0xFFFFFFFF

  class ParamKeyType(IntEnum):
    STRING = 0
    BOOL = 1
    INT = 2
    FLOAT = 3
    TIME = 4
    JSON = 5
    BYTES = 6

  def _load_key_types() -> dict[str, ParamKeyType]:
    key_types: dict[str, ParamKeyType] = {}
    params_keys = Path(__file__).with_name("params_keys.h")
    if not params_keys.exists():
      return key_types

    for line in params_keys.read_text(encoding="utf-8", errors="ignore").splitlines():
      if not line.lstrip().startswith('{"'):
        continue

      parts = line.split('"')
      if len(parts) < 2:
        continue

      key = parts[1]
      for type_name in ParamKeyType.__members__:
        if f", {type_name}" in line:
          key_types[key] = ParamKeyType[type_name]
          break
      else:
        key_types[key] = ParamKeyType.STRING

    return key_types

  _KEY_TYPES = _load_key_types()
  _PERSISTENT_STORE: dict[str, object] = {}
  _MEMORY_STORE: dict[str, object] = {}

  class Params:
    def __init__(self, d: str | None = None, memory: bool = False, return_defaults: bool = False):
      self.d = d if d is not None else ""
      self.m = memory
      self.return_defaults = return_defaults
      self._store = _MEMORY_STORE if memory else _PERSISTENT_STORE

    def __reduce__(self):
      return type(self), (self.d, self.m, self.return_defaults)

    def clear_all(self, tx_flag=ParamKeyFlag.ALL):
      self._store.clear()

    def check_key(self, key):
      if isinstance(key, bytes):
        key = key.decode("utf-8")
      return str(key)

    def _coerce_bool(self, value) -> bool:
      if isinstance(value, bool):
        return value
      if isinstance(value, (int, float)):
        return bool(value)
      if isinstance(value, bytes):
        return value == b"1"
      if isinstance(value, str):
        return value.strip().lower() not in ("", "0", "false", "none", "null")
      return bool(value)

    def _coerce_value(self, key: str, value):
      value_type = self.get_type(key)
      if value_type == ParamKeyType.BOOL:
        return self._coerce_bool(value)
      if value_type == ParamKeyType.INT:
        return int(float(value))
      if value_type == ParamKeyType.FLOAT:
        return float(value)
      if value_type == ParamKeyType.BYTES and isinstance(value, str):
        return value.encode("utf-8")
      return value

    def get(self, key, block: bool = False, return_default: bool = False, encoding=None, default=None):
      key = self.check_key(key)
      value = self._store.get(key, default)
      if value is None:
        return default
      if encoding is not None and isinstance(value, bytes):
        try:
          return value.decode(encoding)
        except Exception:
          return value.decode("utf-8", errors="replace")
      return value

    def get_bool(self, key, block: bool = False, default: bool = False):
      value = self.get(key, block=block, return_default=True, default=default)
      return self._coerce_bool(value)

    def get_int(self, key, block: bool = False, return_default: bool = False, default: int = 0):
      value = self.get(key, block=block, return_default=return_default, encoding="utf-8", default=default)
      if value is None or value == "":
        return default
      try:
        return int(float(value))
      except (TypeError, ValueError):
        return default

    def get_float(self, key, block: bool = False, return_default: bool = False, default: float = 0.0):
      value = self.get(key, block=block, return_default=return_default, encoding="utf-8", default=default)
      if value is None or value == "":
        return default
      try:
        return float(value)
      except (TypeError, ValueError):
        return default

    def put(self, key, dat):
      key = self.check_key(key)
      self._store[key] = self._coerce_value(key, dat)

    def put_bool(self, key, val: bool):
      self.put(key, bool(val))

    def put_nonblocking(self, key, dat):
      self.put(key, dat)

    def put_bool_nonblocking(self, key, val: bool):
      self.put_bool(key, val)

    def put_int(self, key, val):
      self.put(key, int(val))

    def put_float(self, key, val):
      self.put(key, float(val))

    def remove(self, key):
      self._store.pop(self.check_key(key), None)

    def get_param_path(self, key: str = ""):
      base = Path(tempfile.gettempdir()) / ("params_memory" if self.m else "params")
      base.mkdir(parents=True, exist_ok=True)
      return str(base / key) if key else str(base)

    def get_type(self, key):
      return _KEY_TYPES.get(self.check_key(key), ParamKeyType.STRING)

    def all_keys(self):
      return list(_KEY_TYPES)

    def get_default_value(self, key):
      return None

    def cpp2python(self, key, value):
      return self._coerce_value(self.check_key(key), value)

    def get_key_flag(self, key):
      return ParamKeyFlag.PERSISTENT

    def get_stock_value(self, key):
      return None

    def get_tuning_level(self, key):
      return 0

    def get_settings_tier(self, key):
      return _SETTINGS_TIERS.get(self.check_key(key), SETTINGS_ADVANCED)

else:
  assert _Params
  assert ParamKeyFlag
  assert ParamKeyType
  assert UnknownKeyName

  class Params(_Params):
    def _declared(self, key):
      return declared_keys().get(key_name(key))

    def _overlay(self):
      cached = getattr(self, "_overlay_obj", None)
      if cached is not None:
        return cached
      if getattr(self, "m", False):
        obj = DeclaredKeyOverlay(None)
      else:
        try:
          param_dir = super().get_param_path()
        except Exception:
          param_dir = ""
        obj = DeclaredKeyOverlay(overlay_path_from_param_dir(param_dir))
      self._overlay_obj = obj
      return obj

    def _use_overlay(self, key, *, refresh=False) -> bool:
      name = key_name(key)
      cache = getattr(self, "_use_overlay_cache", None)
      if cache is None:
        self._use_overlay_cache = cache = {}
      if not refresh and name in cache:
        return cache[name]
      if self._declared(key) is None:
        cache[name] = False
        return False
      try:
        super().check_key(key)
      except UnknownKeyName:
        cache[name] = True
        return True
      cache[name] = False
      return False

    def _migrate_overlay_to_native(self, key) -> None:
      # When a future prebuilt .so learns these keys, copy any stored overlay
      # value into the official params file once, then drop it from the overlay.
      # Uses super().put/remove only — never self.get/self.put — so this cannot
      # recurse into get() -> migrate -> get().
      if self._use_overlay(key, refresh=True):
        return
      info = self._declared(key)
      if info is None:
        return
      raw = self._overlay().get(key_name(key))
      if raw is OVERLAY_MISSING:
        return
      try:
        super().put(key, decode_declared(info, raw))
        self._overlay().remove(key_name(key))
      except Exception:
        return

    def _maybe_migrate_key(self, key) -> None:
      # Native keys (IsMetric, etc.) must not touch the overlay on every get.
      # declared_keys() includes the whole params_keys.h (~800 keys), so the old
      # "migrate on every native get" path flocked and json.loaded the overlay
      # file on the manager boot all_keys() loop and on every process start.
      # Overlay-only keys (stale .so) stay on the overlay; a new Params instance
      # after the .so learns the key will see use_overlay=False and migrate then.
      if self._use_overlay(key):
        return
      if self._overlay().contains_cached(key_name(key)):
        self._migrate_overlay_to_native(key)

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
        return super().check_key(key)
      except UnknownKeyName:
        if self._declared(key) is not None:
          return key.encode() if isinstance(key, str) else key
        raise

    def all_keys(self):
      keys = list(super().all_keys())
      seen = {key_name(k) for k in keys}
      # The compiled Params.all_keys() returns bytes; callers such as
      # StarPilotVariables do key.decode(), so match the native element type.
      as_bytes = not keys or isinstance(keys[0], bytes)
      for name in declared_keys():
        if name not in seen:
          keys.append(name.encode("utf-8") if as_bytes else name)
      return keys

    def cpp2python(self, key, value):
      if self._use_overlay(key):
        return decode_declared(self._declared(key), value)
      return super().cpp2python(key, value)

    def get_type(self, key):
      if self._use_overlay(key):
        return ParamKeyType(self._declared(key).type)
      return super().get_type(key)

    def get_default_value(self, key):
      if self._use_overlay(key):
        info = self._declared(key)
        if info.default is None:
          return None
        return decode_declared(info, info.default)
      return super().get_default_value(key)

    def get_key_flag(self, key):
      if self._use_overlay(key):
        return ParamKeyFlag(self._declared(key).flags)
      return super().get_key_flag(key)

    def get_stock_value(self, key):
      if self._use_overlay(key):
        info = self._declared(key)
        if info.stock is None:
          return None
        return decode_declared(info, info.stock)
      return super().get_stock_value(key)

    def get_tuning_level(self, key):
      if self._use_overlay(key):
        return self._declared(key).tuning_level
      return super().get_tuning_level(key)

    def get_settings_tier(self, key):
      if self._use_overlay(key):
        return self._declared(key).settings_tier
      try:
        return super().get_settings_tier(key)
      except AttributeError:
        return _SETTINGS_TIERS.get(key_name(key), SETTINGS_ADVANCED)

    def get(self, key, block=False, return_default=False, encoding=None, default=None):
      self._maybe_migrate_key(key)
      if self._use_overlay(key):
        raw = self._overlay().get(key_name(key))
        if raw is OVERLAY_MISSING:
          return self._decode_overlay_value(key, None, encoding=encoding, default=default,
                                            return_default=return_default)
        return self._decode_overlay_value(key, raw, encoding=encoding, default=default,
                                          return_default=return_default)
      try:
        value = super().get(key, block=block, return_default=return_default)
      except UnknownKeyName:
        return default
      if value is None:
        return default
      if encoding is not None and isinstance(value, bytes):
        try:
          return value.decode(encoding)
        except Exception:
          return value.decode("utf-8", errors="replace")
      return value

    def get_bool(self, key, block=False, default=False):
      self._maybe_migrate_key(key)
      if self._use_overlay(key):
        value = self.get(key, block=block, return_default=True, default=default)
        if value is None:
          return bool(default)
        return coerce_bool(value)
      try:
        result = super().get(key, block=block, return_default=True)
        if result is None:
          return bool(default)
        return bool(result)
      except UnknownKeyName:
        return bool(default)

    def get_int(self, key, block=False, return_default=False, default=0):
      val = self.get(key, block=block, return_default=return_default, encoding="utf-8")
      if val is None or val == "":
        return default
      try:
        return int(float(val))
      except (TypeError, ValueError):
        return default

    def get_float(self, key, block=False, return_default=False, default=0.0):
      val = self.get(key, block=block, return_default=return_default, encoding="utf-8")
      if val is None or val == "":
        return default
      try:
        return float(val)
      except (TypeError, ValueError):
        return default

    def put(self, key, dat):
      if self._use_overlay(key):
        info = self._declared(key)
        self._overlay().put(key_name(key), encode_declared(info, dat))
        return
      super().put(key, dat)

    def put_bool(self, key, val: bool):
      if self._use_overlay(key):
        self.put(key, bool(val))
        return
      super().put_bool(key, val)

    def put_nonblocking(self, key, dat):
      if self._use_overlay(key):
        self.put(key, dat)
        return
      super().put_nonblocking(key, dat)

    def put_bool_nonblocking(self, key, val: bool):
      if self._use_overlay(key):
        self.put_bool(key, val)
        return
      super().put_bool_nonblocking(key, val)

    def put_int(self, key, val):
      t = self.get_type(key)
      if t == ParamKeyType.FLOAT:
        self.put(key, float(val))
      elif t == ParamKeyType.INT:
        self.put(key, int(val))
      elif t == ParamKeyType.BOOL:
        self.put(key, bool(val))
      else:
        self.put(key, str(int(val)))

    def put_float(self, key, val):
      t = self.get_type(key)
      if t == ParamKeyType.FLOAT:
        self.put(key, float(val))
      elif t == ParamKeyType.INT:
        self.put(key, int(val))
      elif t == ParamKeyType.BOOL:
        self.put(key, bool(val))
      else:
        self.put(key, str(float(val)))

    def remove(self, key):
      if self._use_overlay(key):
        self._overlay().remove(key_name(key))
        return
      super().remove(key)

    def clear_all(self, tx_flag=ParamKeyFlag.ALL):
      super().clear_all(tx_flag)
      self._overlay().clear(int(tx_flag), declared_keys())


if __name__ == "__main__":
  import sys

  params = Params()
  key = sys.argv[1]
  assert params.check_key(key), f"unknown param: {key}"

  if len(sys.argv) == 3:
    val = sys.argv[2]
    print(f"SET: {key} = {val}")
    params.put(key, val)
  elif len(sys.argv) == 2:
    print(f"GET: {key} = {params.get(key)}")
