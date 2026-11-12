#!/usr/bin/env python3
"""
Tesla built-in navigator as navInstruction source (TeslaNavSource).

Read-only: listens to `can`, decodes UI_driverAssistMapData (0x238, Model 3/Y) and publishes
navInstruction so NavLaneChange can move toward the exit side. Nothing is ever sent on CAN.

Layout (joshwardell/model3dbc Model3CAN.dbc, Intel / little-endian, start bit = LSB):
  UI_navRouteActive          bit 29
  UI_nextBranchDist          bits 32-36, x10 m (0-300 m), raw 31 = SNA
  UI_nextBranchLeftOffRamp   bit 38
  UI_nextBranchRightOffRamp  bit 39
  UI_mapDataCounter          bits 52-55
  UI_mapDataChecksum         bits 56-63 (byte 7)

The UI sends it to the Autopilot computer, so it should be on the car side of the party bus (bus 0);
bus 2 (Autopilot side) is used as a fallback. Only one navInstruction publisher may run: when
TeslaNavSource is on, process_config does not start navd/mapsd.
"""
import time

import openpilot.cereal.messaging as messaging
from openpilot.common.swaglog import cloudlog

MAP_DATA_ADDR = 0x238
BUS_PRIMARY = 0
BUS_FALLBACK = 2
DIST_SNA = 31
DIST_SCALE = 10.0  # m per raw step

FRAME_TIMEOUT = 1.0  # s without an accepted frame -> no maneuver
BUS_FALLBACK_TIMEOUT = 3.0  # s without bus 0 frames before using bus 2
CHECKSUM_LEARN_FRAMES = 25  # frames used to decide whether the Tesla checksum applies to 0x238
CHECKSUM_MATCH_RATIO = 0.9
CONSISTENT_FRAMES = 3  # without a verified checksum: same route/side flags in this many consecutive frames
CONSISTENT_MAX_DIST_STEP = 20.0  # m between consecutive frames
NEW_EXIT_DIST_JUMP = 60.0  # m: distance growing by more than this means a new exit
NEW_EXIT_GAP = 3.0  # s without a maneuver means the next one is a new exit
NOT_SEEN_LOG_AFTER = 10.0  # s
PUBLISH_PERIOD = 0.25  # s (4 Hz)

SOURCE_TAG = "tesla"  # navInstruction.maneuverSecondaryText marks this source for NavLaneChange


def decode_map_data(dat: bytes) -> dict | None:
  if len(dat) < 8:
    return None
  v = int.from_bytes(bytes(dat[:8]), 'little')
  return {
    'route_active': bool((v >> 29) & 0x1),
    'dist_raw': (v >> 32) & 0x1F,
    'left': bool((v >> 38) & 0x1),
    'right': bool((v >> 39) & 0x1),
    'counter': (v >> 52) & 0xF,
    'checksum': (v >> 56) & 0xFF,
  }


def tesla_checksum(addr: int, dat: bytes, checksum_byte: int = 7) -> int:
  """Tesla CAN checksum as in opendbc safety/modes/tesla.h: addr bytes + all other data bytes."""
  s = (addr & 0xFF) + ((addr >> 8) & 0xFF)
  for i, b in enumerate(dat):
    if i != checksum_byte:
      s += b
  return s & 0xFF


class _BusState:
  def __init__(self):
    self.last_seen = None
    self.frames = 0
    self.checksum_matches = 0
    self.checksum_mode = None  # None = learning, True = verify checksum, False = consistency only
    self.consistent = 0
    self.prev = None


class TeslaNavTracker:
  def __init__(self, start_time: float = 0.0):
    self.start_time = start_time
    self.buses = {BUS_PRIMARY: _BusState(), BUS_FALLBACK: _BusState()}
    self.active_bus = BUS_PRIMARY
    self.latest = None  # last accepted decoded frame on the active bus
    self.latest_time = None
    self.exit_id = 0
    self.last_side = None
    self.last_dist = None
    self.last_maneuver_time = None
    self.logged_seen = set()
    self.logged_not_seen = False
    self.logged_route = False

  # --- input ---
  def on_frame(self, bus: int, dat: bytes, t: float) -> None:
    if bus not in self.buses:
      return
    d = decode_map_data(dat)
    if d is None:
      return
    st = self.buses[bus]
    st.last_seen = t
    st.frames += 1

    checksum_ok = d['checksum'] == tesla_checksum(MAP_DATA_ADDR, bytes(dat[:8]))
    if st.checksum_mode is None:
      st.checksum_matches += int(checksum_ok)
      if st.frames >= CHECKSUM_LEARN_FRAMES:
        st.checksum_mode = st.checksum_matches >= CHECKSUM_MATCH_RATIO * st.frames
        mode = 'verified' if st.checksum_mode else 'not matching, using consistency check'
        cloudlog.warning(f"tesla_navd: 0x238 bus {bus} checksum {mode} ({st.checksum_matches}/{st.frames})")
    if bus not in self.logged_seen:
      self.logged_seen.add(bus)
      cloudlog.warning(f"tesla_navd: UI_driverAssistMapData 0x238 seen on bus {bus}")

    accepted = False
    if st.checksum_mode:
      accepted = checksum_ok
    else:
      prev = st.prev
      dist = self._dist(d)
      prev_dist = self._dist(prev) if prev is not None else None
      same = prev is not None and (prev['route_active'], prev['left'], prev['right']) == (d['route_active'], d['left'], d['right']) and \
        ((dist is None) == (prev_dist is None)) and (dist is None or abs(dist - prev_dist) <= CONSISTENT_MAX_DIST_STEP)
      st.consistent = st.consistent + 1 if same else 1
      st.prev = d
      accepted = st.consistent >= CONSISTENT_FRAMES

    self._select_bus(t)
    if accepted and bus == self.active_bus:
      self.latest, self.latest_time = d, t
      if d['route_active'] and not self.logged_route:
        self.logged_route = True
        cloudlog.warning(f"tesla_navd: Tesla navigation route active (bus {bus})")

  def _select_bus(self, t: float) -> None:
    p, f = self.buses[BUS_PRIMARY], self.buses[BUS_FALLBACK]
    primary_fresh = p.last_seen is not None and t - p.last_seen < BUS_FALLBACK_TIMEOUT
    fallback_fresh = f.last_seen is not None and t - f.last_seen < BUS_FALLBACK_TIMEOUT
    new_bus = BUS_FALLBACK if (not primary_fresh and fallback_fresh) else BUS_PRIMARY
    if new_bus != self.active_bus:
      cloudlog.warning(f"tesla_navd: using bus {new_bus} for 0x238")
      self.active_bus = new_bus
      self.latest = self.latest_time = None

  @staticmethod
  def _dist(d: dict | None) -> float | None:
    if d is None or d['dist_raw'] == DIST_SNA:
      return None
    return d['dist_raw'] * DIST_SCALE

  def check_logs(self, t: float) -> None:
    if not self.logged_seen and not self.logged_not_seen and t - self.start_time > NOT_SEEN_LOG_AFTER:
      self.logged_not_seen = True
      cloudlog.warning(f"tesla_navd: UI_driverAssistMapData 0x238 not seen on bus 0 or 2 after {NOT_SEEN_LOG_AFTER:.0f} s")

  # --- output ---
  def maneuver(self, t: float):
    """(valid, side 'left'|'right'|None, distance m|None, exit_id)."""
    self._select_bus(t)
    if self.latest is None or self.latest_time is None or t - self.latest_time > FRAME_TIMEOUT:
      return False, None, None, self.exit_id
    d = self.latest
    dist = self._dist(d)
    side = None
    if d['route_active'] and dist is not None and d['left'] != d['right']:
      side = 'left' if d['left'] else 'right'
    if side is not None:
      new_exit = (self.last_side is None or side != self.last_side or
                  (self.last_dist is not None and dist > self.last_dist + NEW_EXIT_DIST_JUMP) or
                  (self.last_maneuver_time is not None and t - self.last_maneuver_time > NEW_EXIT_GAP))
      if new_exit:
        self.exit_id += 1
      self.last_side, self.last_dist, self.last_maneuver_time = side, dist, t
    return True, side, dist, self.exit_id

  def instruction_fields(self, t: float) -> tuple[bool, dict]:
    valid, side, dist, exit_id = self.maneuver(t)
    if side is None:
      return valid, {'maneuverType': '', 'maneuverModifier': '', 'maneuverDistance': 0.0,
                     'maneuverPrimaryText': '', 'maneuverSecondaryText': SOURCE_TAG}
    return valid, {
      'maneuverType': 'off ramp',
      'maneuverModifier': f'slight {side}',
      'maneuverDistance': float(dist),
      'maneuverPrimaryText': f"Tesla uscita #{exit_id} ({'sinistra' if side == 'left' else 'destra'})",
      'maneuverSecondaryText': SOURCE_TAG,
    }


def main():
  can_sock = messaging.sub_sock('can', timeout=100)
  pm = messaging.PubMaster(['navInstruction'])
  tracker = TeslaNavTracker(time.monotonic())
  cloudlog.warning("tesla_navd: started, listening for 0x238 on bus 0 (fallback bus 2)")
  last_pub = 0.0
  while True:
    msgs = messaging.drain_sock(can_sock, wait_for_one=True)
    t = time.monotonic()
    for m in msgs:
      for c in m.can:
        if c.address == MAP_DATA_ADDR and c.src in (BUS_PRIMARY, BUS_FALLBACK):
          tracker.on_frame(c.src, bytes(c.dat), t)
    tracker.check_logs(t)
    if t - last_pub >= PUBLISH_PERIOD:
      last_pub = t
      valid, fields = tracker.instruction_fields(t)
      msg = messaging.new_message('navInstruction', valid=valid)
      for k, v in fields.items():
        setattr(msg.navInstruction, k, v)
      pm.send('navInstruction', msg)


if __name__ == "__main__":
  main()
