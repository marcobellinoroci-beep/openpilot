"""
TeslaNavStatus: small status written by tesla_navd for the on-device settings row "Navigatore Tesla".

Kept free of messaging/params imports so both tesla_navd and the UI can use it.
The param is cleared on manager start and on each onroad transition, so a new drive starts from "In attesa…".
"""
STATUS_PARAM = "TeslaNavStatus"

STATE_WAITING = "waiting"
STATE_NOT_SEEN = "not_seen"
STATE_NO_ROUTE = "no_route"
STATE_ROUTE = "route"

STATUS_WRITE_MIN_PERIOD = 1.0  # s, and only when the status changes (limits flash writes)


def format_status(enabled: bool, status: dict | None, onroad: bool = True) -> str:
  if not enabled:
    return "Disattivato"
  state = status.get("state") if isinstance(status, dict) else None
  if state == STATE_NOT_SEEN:
    text = "Non rilevato (0x238 assente su bus 0 e 2)"
  elif state == STATE_NO_ROUTE:
    text = f"Rilevato (bus {status.get('bus', 0)}), nessun percorso"
  elif state == STATE_ROUTE:
    text = "Percorso attivo"
    side, dist = status.get("side"), status.get("dist")
    if side in ("left", "right") and isinstance(dist, (int, float)):
      text += f", uscita a {'sinistra' if side == 'left' else 'destra'} tra {int(round(dist))} m"
  else:
    return "In attesa…"
  if not onroad:
    text += " (ultimo viaggio)"
  return text


class StatusWriter:
  """Writes the status param only when it changed and at most once per STATUS_WRITE_MIN_PERIOD."""
  def __init__(self, put):
    self._put = put
    self._last = None
    self._last_write = None

  def update(self, status: dict, t: float) -> bool:
    if status == self._last:
      return False
    if self._last_write is not None and t - self._last_write < STATUS_WRITE_MIN_PERIOD:
      return False
    self._put(STATUS_PARAM, status)
    self._last, self._last_write = dict(status), t
    return True
