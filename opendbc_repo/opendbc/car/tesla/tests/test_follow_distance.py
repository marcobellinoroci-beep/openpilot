from types import SimpleNamespace

from cereal import custom
from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.tesla.carstate import (
  CarState,
  UI_DRIVER_ASSIST_CONTROL_ADDR,
  tesla_follow_distance,
)
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CANBUS, CAR

ButtonType = structs.CarState.ButtonEvent.Type
PACKER = CANPacker("tesla_model3_party")


def _model3_params():
  fingerprint = gen_empty_fingerprint()
  toggles = SimpleNamespace(tesla_aol_screen_tap_requested=False, trailer_load_kg=0.0)
  return CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], True, False, False, toggles), toggles


def _follow_frame(distance: int, bus: int):
  return PACKER.make_can_msg("UI_driverAssistControl", bus, {"UI_followDistance": distance})


def _feed(parsers, t_ns, *, distance=None, bus=CANBUS.autopilot_party):
  msgs = []
  if distance is not None:
    msgs.append(_follow_frame(distance, bus))
  for parser in parsers.values():
    parser.update([[t_ns, msgs]])


def _events(cs, parsers, toggles):
  ret, _fp = cs.update(parsers, toggles)
  return [(be.type, be.pressed) for be in ret.buttonEvents]


def test_community_bit_layout_matches_dbc():
  # Community HW3/HW4: fd = (data[5] & 0b11100000) >> 5
  addr, dat, _bus = PACKER.make_can_msg(
    "UI_driverAssistControl", CANBUS.autopilot_party, {"UI_followDistance": 5},
  )
  assert addr == UI_DRIVER_ASSIST_CONTROL_ADDR
  assert (dat[5] & 0b11100000) >> 5 == 5

  parser = CANParser("tesla_model3_party", [("UI_driverAssistControl", 0)], CANBUS.autopilot_party)
  parser.update([[1_000_000_000, [(addr, dat, CANBUS.autopilot_party)]]])
  assert int(parser.vl["UI_driverAssistControl"]["UI_followDistance"]) == 5


def test_first_follow_distance_does_not_emit_button():
  cp, toggles = _model3_params()
  cs = CarState(cp, custom.StarPilotCarParams.new_message())
  parsers = CarState.get_can_parsers(cp)

  _feed(parsers, 1_000_000_000, distance=4)
  events = _events(cs, parsers, toggles)
  assert tesla_follow_distance(parsers) == 4
  assert cs.prev_follow_distance == 4
  assert (ButtonType.gapAdjustCruise, False) not in events
  assert (ButtonType.gapAdjustCruise, True) not in events


def test_follow_distance_change_emits_gap_adjust_cruise():
  cp, toggles = _model3_params()
  cs = CarState(cp, custom.StarPilotCarParams.new_message())
  parsers = CarState.get_can_parsers(cp)

  _feed(parsers, 1_000_000_000, distance=4)
  assert _events(cs, parsers, toggles) == []

  _feed(parsers, 2_000_000_000, distance=5)
  events = _events(cs, parsers, toggles)
  # Short press: press on the change, release on the next update, so
  # distancePressed does not stay high. selfdrived cycles personality on the release.
  assert events == [(ButtonType.gapAdjustCruise, True)]
  assert cs.prev_follow_distance == 5

  _feed(parsers, 3_000_000_000, distance=5)
  assert _events(cs, parsers, toggles) == [(ButtonType.gapAdjustCruise, False)]

  _feed(parsers, 4_000_000_000, distance=5)
  assert _events(cs, parsers, toggles) == []


def test_party_bus_fallback_and_zero_ignored():
  cp, toggles = _model3_params()
  cs = CarState(cp, custom.StarPilotCarParams.new_message())
  parsers = CarState.get_can_parsers(cp)

  # 0 is treated as unseen (SNA / missing field).
  _feed(parsers, 1_000_000_000, distance=0, bus=CANBUS.party)
  events = _events(cs, parsers, toggles)
  assert tesla_follow_distance(parsers) is None
  assert cs.prev_follow_distance is None
  assert events == []

  _feed(parsers, 2_000_000_000, distance=2, bus=CANBUS.party)
  assert _events(cs, parsers, toggles) == []
  assert tesla_follow_distance(parsers) == 2

  _feed(parsers, 3_000_000_000, distance=3, bus=CANBUS.party)
  events = _events(cs, parsers, toggles)
  assert events == [(ButtonType.gapAdjustCruise, True)]


def test_ap_party_preferred_over_party():
  cp, toggles = _model3_params()
  parsers = CarState.get_can_parsers(cp)
  _feed(parsers, 1_000_000_000, distance=2, bus=CANBUS.party)
  _feed(parsers, 1_000_000_000, distance=6, bus=CANBUS.autopilot_party)
  assert tesla_follow_distance(parsers) == 6


def test_missing_frame_does_not_clear_or_retrigger():
  cp, toggles = _model3_params()
  cs = CarState(cp, custom.StarPilotCarParams.new_message())
  parsers = CarState.get_can_parsers(cp)

  _feed(parsers, 1_000_000_000, distance=4)
  _events(cs, parsers, toggles)
  _feed(parsers, 2_000_000_000, distance=5)
  events = _events(cs, parsers, toggles)
  assert events == [(ButtonType.gapAdjustCruise, True)]

  # No new 0x3F8 this cycle: keep last setting, only the pending release.
  _feed(parsers, 3_000_000_000, distance=None)
  assert tesla_follow_distance(parsers) == 5
  assert _events(cs, parsers, toggles) == [(ButtonType.gapAdjustCruise, False)]
  assert cs.prev_follow_distance == 5

  _feed(parsers, 4_000_000_000, distance=None)
  assert _events(cs, parsers, toggles) == []
