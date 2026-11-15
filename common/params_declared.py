"""Declared params_keys.h metadata and an on-disk overlay for stale params_pyx.so.

The published StarPilot prebuilt params_pyx.so may lag keys added in
params_keys.h.  C++ Params::clearAll deletes unknown files in the params
directory, so this overlay lives *beside* that directory
(fork_key_overlay.json) and is used only for keys declared in the header
that the compiled .so does not know.
"""
from __future__ import annotations

from dataclasses import dataclass
import fcntl
import json
import os
import tempfile
import threading
from pathlib import Path
from typing import Any

OVERLAY_FILENAME = "fork_key_overlay.json"

# Mirrors ParamKeyFlag / ParamKeyType / ParamSettingsTier in params.h.
FLAG_PERSISTENT = 0x02
FLAG_CLEAR_ON_MANAGER_START = 0x04
FLAG_CLEAR_ON_ONROAD_TRANSITION = 0x08
FLAG_CLEAR_ON_OFFROAD_TRANSITION = 0x10
FLAG_DONT_LOG = 0x20
FLAG_DEVELOPMENT_ONLY = 0x40
FLAG_CLEAR_ON_IGNITION_ON = 0x80
FLAG_ALL = 0xFFFFFFFF

TYPE_STRING = 0
TYPE_BOOL = 1
TYPE_INT = 2
TYPE_FLOAT = 3
TYPE_TIME = 4
TYPE_JSON = 5
TYPE_BYTES = 6

SETTINGS_SIMPLE = 0
SETTINGS_ADVANCED = 1

_FLAG_NAMES = {
  "PERSISTENT": FLAG_PERSISTENT,
  "CLEAR_ON_MANAGER_START": FLAG_CLEAR_ON_MANAGER_START,
  "CLEAR_ON_ONROAD_TRANSITION": FLAG_CLEAR_ON_ONROAD_TRANSITION,
  "CLEAR_ON_OFFROAD_TRANSITION": FLAG_CLEAR_ON_OFFROAD_TRANSITION,
  "DONT_LOG": FLAG_DONT_LOG,
  "DEVELOPMENT_ONLY": FLAG_DEVELOPMENT_ONLY,
  "CLEAR_ON_IGNITION_ON": FLAG_CLEAR_ON_IGNITION_ON,
  "ALL": FLAG_ALL,
}

_TYPE_NAMES = {
  "STRING": TYPE_STRING,
  "BOOL": TYPE_BOOL,
  "INT": TYPE_INT,
  "FLOAT": TYPE_FLOAT,
  "TIME": TYPE_TIME,
  "JSON": TYPE_JSON,
  "BYTES": TYPE_BYTES,
}

_TIER_NAMES = {
  "SETTINGS_SIMPLE": SETTINGS_SIMPLE,
  "SETTINGS_ADVANCED": SETTINGS_ADVANCED,
}


@dataclass(frozen=True)
class DeclaredKey:
  name: str
  flags: int
  type: int
  default: str | None
  stock: str | None
  tuning_level: int
  settings_tier: int


def _balanced_inner(text: str) -> str | None:
  if not text.startswith("{"):
    return None
  depth = 0
  in_str = False
  escape = False
  for i, ch in enumerate(text):
    if in_str:
      if escape:
        escape = False
      elif ch == "\\":
        escape = True
      elif ch == '"':
        in_str = False
      continue
    if ch == '"':
      in_str = True
      continue
    if ch == "{":
      depth += 1
    elif ch == "}":
      depth -= 1
      if depth == 0:
        return text[1:i]
  return None


def _split_attr_tokens(attrs: str) -> list[str]:
  tokens: list[str] = []
  buf: list[str] = []
  in_str = False
  escape = False
  for ch in attrs:
    if in_str:
      buf.append(ch)
      if escape:
        escape = False
      elif ch == "\\":
        escape = True
      elif ch == '"':
        in_str = False
      continue
    if ch == '"':
      in_str = True
      buf.append(ch)
      continue
    if ch == ",":
      token = "".join(buf).strip()
      if token:
        tokens.append(token)
      buf = []
      continue
    buf.append(ch)
  token = "".join(buf).strip()
  if token:
    tokens.append(token)
  return tokens


def parse_params_keys_header(path: Path | None = None) -> dict[str, DeclaredKey]:
  header = path or Path(__file__).with_name("params_keys.h")
  declared: dict[str, DeclaredKey] = {}
  if not header.exists():
    return declared

  for line in header.read_text(encoding="utf-8", errors="ignore").splitlines():
    stripped = line.strip()
    if not stripped.startswith('{"'):
      continue
    try:
      key_end = stripped.find('"', 2)
      if key_end < 0:
        continue
      name = stripped[2:key_end]
      rest = stripped[key_end + 1:].lstrip()
      if not rest.startswith(","):
        continue
      rest = rest[1:].lstrip()
      attrs = _balanced_inner(rest)
      if attrs is None:
        continue
      tokens = _split_attr_tokens(attrs)
      if not tokens:
        continue

      flags = 0
      typ = TYPE_STRING
      strings: list[str] = []
      ints: list[int] = []
      tier = SETTINGS_ADVANCED
      type_seen = False
      for token in tokens:
        if token in _TIER_NAMES:
          tier = _TIER_NAMES[token]
          continue
        if "|" in token or token in _FLAG_NAMES:
          for part in token.split("|"):
            part = part.strip()
            if part in _FLAG_NAMES:
              flags |= _FLAG_NAMES[part]
          continue
        if token in _TYPE_NAMES:
          typ = _TYPE_NAMES[token]
          type_seen = True
          continue
        if token.startswith('"') and token.endswith('"') and len(token) >= 2:
          strings.append(bytes(token[1:-1], "utf-8").decode("unicode_escape") if "\\" in token else token[1:-1])
          continue
        try:
          ints.append(int(token, 0))
        except ValueError:
          pass
      if not type_seen:
        typ = TYPE_STRING
      default = strings[0] if strings else None
      stock = strings[1] if len(strings) > 1 else None
      tuning = ints[0] if ints else 0
      declared[name] = DeclaredKey(name, flags, typ, default, stock, tuning, tier)
    except Exception:
      continue
  return declared


_DECLARED: dict[str, DeclaredKey] | None = None


def declared_keys() -> dict[str, DeclaredKey]:
  global _DECLARED
  if _DECLARED is None:
    _DECLARED = parse_params_keys_header()
  return _DECLARED


def key_name(key: Any) -> str:
  if isinstance(key, bytes):
    return key.decode("utf-8")
  return str(key)


def coerce_bool(value: Any) -> bool:
  if isinstance(value, bool):
    return value
  if isinstance(value, (int, float)):
    return bool(value)
  if isinstance(value, bytes):
    return value == b"1"
  if isinstance(value, str):
    return value.strip().lower() not in ("", "0", "false", "none", "null")
  return bool(value)


def decode_declared(key_info: DeclaredKey, raw: Any, encoding: str | None = None) -> Any:
  if raw is None:
    return None
  typ = key_info.type
  if typ == TYPE_BOOL:
    return coerce_bool(raw)
  if typ == TYPE_INT:
    return int(float(raw))
  if typ == TYPE_FLOAT:
    return float(raw)
  if typ == TYPE_JSON:
    if isinstance(raw, (dict, list)):
      return raw
    if isinstance(raw, (bytes, bytearray)):
      raw = raw.decode("utf-8")
    return json.loads(raw)
  if typ == TYPE_BYTES:
    if isinstance(raw, bytes):
      return raw
    if isinstance(raw, list):
      return bytes(raw)
    if isinstance(raw, str):
      return raw.encode("utf-8")
    return raw
  if isinstance(raw, bytes):
    enc = encoding or "utf-8"
    try:
      return raw.decode(enc)
    except Exception:
      return raw.decode("utf-8", errors="replace")
  return str(raw)


def encode_declared(key_info: DeclaredKey, value: Any) -> Any:
  typ = key_info.type
  if typ == TYPE_BOOL:
    return "1" if coerce_bool(value) else "0"
  if typ == TYPE_INT:
    return str(int(float(value)))
  if typ == TYPE_FLOAT:
    return str(float(value))
  if typ == TYPE_JSON:
    if isinstance(value, (dict, list)):
      return value
    if isinstance(value, (bytes, bytearray)):
      return json.loads(value.decode("utf-8"))
    if isinstance(value, str):
      return json.loads(value)
    return value
  if typ == TYPE_BYTES:
    if isinstance(value, bytes):
      return list(value)
    if isinstance(value, str):
      return list(value.encode("utf-8"))
    return value
  if isinstance(value, bytes):
    return value.decode("utf-8", errors="replace")
  return str(value)


_io_lock = threading.RLock()
_memory_store: dict[str, Any] = {}


def overlay_path_from_param_dir(param_dir: str | os.PathLike[str] | None) -> Path | None:
  if not param_dir:
    return None
  path = Path(param_dir)
  # get_param_path() is the key directory (e.g. /data/params/d).  The overlay
  # must not live inside that directory: clearAll unlinks unknown names there.
  parent = path.parent if path.name else path
  return parent / OVERLAY_FILENAME


def _read_file(path: Path) -> dict[str, Any]:
  try:
    data = json.loads(path.read_text(encoding="utf-8"))
  except (OSError, TypeError, ValueError, json.JSONDecodeError):
    return {}
  return data if isinstance(data, dict) else {}


def _write_file(path: Path, data: dict[str, Any]) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  fd, tmp = tempfile.mkstemp(prefix=".fork_overlay_", dir=str(path.parent), text=True)
  try:
    with os.fdopen(fd, "w", encoding="utf-8") as handle:
      json.dump(data, handle, separators=(",", ":"))
      handle.flush()
      os.fsync(handle.fileno())
    os.replace(tmp, path)
  except Exception:
    try:
      os.unlink(tmp)
    except OSError:
      pass
    raise


class DeclaredKeyOverlay:
  """Process-safe JSON overlay. path=None keeps values in memory only.

  File contents are cached by (mtime_ns, size).  Reads do not take the
  exclusive flock; writers still lock and atomically replace the file.
  Native Params.get used to flock+json.loads on every call — that is the
  hot path at manager boot (all_keys loop over ~800 keys, then every
  process).
  """

  _file_cache: dict[str, tuple[int | None, int, dict[str, Any]]] = {}

  def __init__(self, path: Path | None):
    self.path = path
    self._cache_key = str(path) if path is not None else None
    self._loaded = False

  def _lock_path(self) -> Path:
    return self.path.with_name(self.path.name + ".lock")

  def _locked(self):
    class _Guard:
      def __init__(guard, path: Path):
        guard.path = path
        guard.fd = None
      def __enter__(guard):
        guard.path.parent.mkdir(parents=True, exist_ok=True)
        guard.fd = os.open(str(guard.path), os.O_CREAT | os.O_RDWR, 0o644)
        try:
          fcntl.flock(guard.fd, fcntl.LOCK_EX)
        except BaseException:
          os.close(guard.fd)
          guard.fd = None
          raise
        return guard
      def __exit__(guard, exc_type, exc, tb):
        if guard.fd is not None:
          try:
            fcntl.flock(guard.fd, fcntl.LOCK_UN)
          finally:
            os.close(guard.fd)
            guard.fd = None
        return False
    return _Guard(self._lock_path())

  def _stat(self) -> tuple[int, int] | None:
    try:
      st = self.path.stat()
      return st.st_mtime_ns, st.st_size
    except OSError:
      return None

  def _cached_data(self, *, refresh: bool) -> dict[str, Any]:
    if self.path is None:
      return _memory_store
    key = self._cache_key
    cached = self._file_cache.get(key)
    if not refresh and self._loaded and cached is not None:
      return cached[2]
    st = self._stat()
    if st is None:
      self._file_cache[key] = (None, 0, {})
      self._loaded = True
      return self._file_cache[key][2]
    if cached is not None and cached[0] == st[0] and cached[1] == st[1]:
      self._loaded = True
      return cached[2]
    data = _read_file(self.path)
    self._file_cache[key] = (st[0], st[1], data)
    self._loaded = True
    return data

  def _load(self) -> dict[str, Any]:
    # Under a write lock, always re-read so we do not clobber a concurrent writer.
    return self._cached_data(refresh=True)

  def _save(self, data: dict[str, Any]) -> None:
    if self.path is None:
      _memory_store.clear()
      _memory_store.update(data)
      return
    _write_file(self.path, data)
    st = self._stat()
    self._file_cache[self._cache_key] = (st[0] if st else None, st[1] if st else 0, dict(data))
    self._loaded = True

  def _with_locks(self):
    class _Both:
      def __init__(guard, overlay):
        guard.overlay = overlay
        guard.file_guard = None
      def __enter__(guard):
        _io_lock.acquire()
        if guard.overlay.path is not None:
          guard.file_guard = guard.overlay._locked()
          guard.file_guard.__enter__()
        return guard
      def __exit__(guard, exc_type, exc, tb):
        if guard.file_guard is not None:
          guard.file_guard.__exit__(exc_type, exc, tb)
        _io_lock.release()
        return False
    return _Both(self)

  def contains_cached(self, name: str) -> bool:
    """True if this key is in the last loaded overlay. Loads once; no re-stat."""
    return name in self._cached_data(refresh=False)

  def get(self, name: str) -> Any:
    if self.path is None:
      with _io_lock:
        return _memory_store.get(name, OVERLAY_MISSING)
    # Atomic replace makes a lock-free read safe; refresh on mtime/size.
    return self._cached_data(refresh=True).get(name, OVERLAY_MISSING)

  def put(self, name: str, value: Any) -> None:
    with self._with_locks():
      data = dict(self._load())
      data[name] = value
      self._save(data)

  def remove(self, name: str) -> None:
    with self._with_locks():
      data = dict(self._load())
      if name in data:
        del data[name]
        self._save(data)

  def clear(self, flag: int, declared: dict[str, DeclaredKey]) -> None:
    with self._with_locks():
      data = dict(self._load())
      changed = False
      for name in list(data):
        info = declared.get(name)
        if info is None or (info.flags & flag):
          del data[name]
          changed = True
      if changed:
        self._save(data)


OVERLAY_MISSING = object()
