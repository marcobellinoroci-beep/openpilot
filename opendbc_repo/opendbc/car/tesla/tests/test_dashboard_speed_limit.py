from types import SimpleNamespace

import pytest

from cereal import custom
from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.carstate import (
  CarState,
  tesla_dashboard_speed_limit,
  tesla_das_status_stale,
)
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CANBUS, CAR

PACKER = CANPacker("tesla_model3_party")


@pytest.mark.parametrize(
  ("physical", "units", "stale", "expected"),
  [
    (110, "KPH", False, 110 * CV.KPH_TO_MS),
    (70, "MPH", False, 70 * CV.MPH_TO_MS),
    (0, "KPH", False, 0.0),          # UNKNOWN_SNA
    (155, "KPH", False, 0.0),        # NONE (raw 31 * factor 5)
    (110, "KPH", True, 0.0),         # stale fails closed
    (110, None, False, 0.0),         # unknown cluster units
    (110, "KT", False, 0.0),
    (-5, "KPH", False, 0.0),
  ],
)
def test_tesla_dashboard_speed_limit_decode(physical, units, stale, expected):
  assert tesla_dashboard_speed_limit(physical, units, stale) == pytest.approx(expected)


def test_none_is_not_mistaken_for_a_real_limit():
  # Zoompilot looked up CANDefine.dv with the physical value; 155 would miss VAL_ 31 "NONE".
  assert tesla_dashboard_speed_limit(155, "KPH") == 0.0
  assert tesla_dashboard_speed_limit(155, "MPH") == 0.0


def _model3_params():
  fingerprint = gen_empty_fingerprint()
  toggles = SimpleNamespace(tesla_aol_screen_tap_requested=False, trailer_load_kg=0.0)
  return CarInterface.get_params(CAR.TESLA_MODEL_3, fingerprint, [], True, False, False, toggles), toggles


def _prime(cs, parsers, toggles):
  # First update registers lazily parsed messages (including DAS_status on ap_party).
  return cs.update(parsers, toggles)


def _feed(parsers, t_ns, *, fused=110, units="KPH", counter=1):
  unit_raw = 1 if units == "KPH" else 0
  msgs = [
    PACKER.make_can_msg("DI_state", CANBUS.party, {"DI_speedUnits": unit_raw, "DI_digitalSpeed": 80, "DI_cruiseState": 1}),
    PACKER.make_can_msg("DAS_status", CANBUS.autopilot_party, {"DAS_fusedSpeedLimit": fused, "DAS_statusCounter": counter}),
  ]
  for parser in parsers.values():
    parser.update([[t_ns, msgs]])


def test_carstate_publishes_dashboard_speed_limit_ms_for_model_3():
  cp, toggles = _model3_params()
  cs = CarState(cp, custom.StarPilotCarParams.new_message())
  parsers = CarState.get_can_parsers(cp)
  _prime(cs, parsers, toggles)
  _feed(parsers, 2_000_000_000, fused=110, units="KPH", counter=1)

  _, fp_ret = cs.update(parsers, toggles)

  assert not tesla_das_status_stale(parsers[Bus.ap_party])
  assert fp_ret.dashboardSpeedLimit == pytest.approx(110 * CV.KPH_TO_MS)


def test_carstate_mph_and_unavailable_and_stale():
  cp, toggles = _model3_params()
  cs = CarState(cp, custom.StarPilotCarParams.new_message())
  parsers = CarState.get_can_parsers(cp)
  _prime(cs, parsers, toggles)
  ap_party = parsers[Bus.ap_party]

  _feed(parsers, 2_000_000_000, fused=70, units="MPH", counter=1)
  _, fp_ret = cs.update(parsers, toggles)
  assert fp_ret.dashboardSpeedLimit == pytest.approx(70 * CV.MPH_TO_MS)

  _feed(parsers, 2_100_000_000, fused=155, units="MPH", counter=2)
  _, fp_ret = cs.update(parsers, toggles)
  assert fp_ret.dashboardSpeedLimit == 0.0

  _feed(parsers, 2_200_000_000, fused=0, units="MPH", counter=3)
  _, fp_ret = cs.update(parsers, toggles)
  assert fp_ret.dashboardSpeedLimit == 0.0

  # Last good value must not stick after DAS_status times out (fail closed).
  _feed(parsers, 3_000_000_000, fused=110, units="KPH", counter=4)
  _, fp_ret = cs.update(parsers, toggles)
  assert fp_ret.dashboardSpeedLimit == pytest.approx(110 * CV.KPH_TO_MS)

  # 20s later, no DAS_status: parser timeout (10s at the initial 1 Hz assumption).
  for parser in parsers.values():
    parser.update([[23_000_000_000, []]])
  assert tesla_das_status_stale(ap_party)
  _, fp_ret = cs.update(parsers, toggles)
  assert fp_ret.dashboardSpeedLimit == 0.0
