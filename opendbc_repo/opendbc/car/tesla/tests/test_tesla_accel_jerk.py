from types import SimpleNamespace

import pytest

from opendbc.can import CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.tesla.carcontroller import CarController, apply_tesla_accel_jerk_limit
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR, DBC, CarControllerParams


DT = CarControllerParams.ACCEL_CMD_DT
UP = CarControllerParams.ACCEL_JERK_LIMIT_UP
DOWN = CarControllerParams.ACCEL_JERK_LIMIT_DOWN
DOWN_URGENT = CarControllerParams.ACCEL_JERK_LIMIT_DOWN_URGENT


def test_comfort_up_jerk_is_limited():
  assert apply_tesla_accel_jerk_limit(2.0, 0.0, DT) == pytest.approx(UP * DT)


def test_mild_down_jerk_is_limited():
  assert apply_tesla_accel_jerk_limit(-1.0, 0.0, DT) == pytest.approx(-DOWN * DT)


def test_urgent_down_jerk_is_faster_but_still_limited():
  # Planner asks for -2.0, just below the urgent threshold, not yet unlimited.
  limited = apply_tesla_accel_jerk_limit(-2.0, 0.0, DT)
  assert limited == pytest.approx(-DOWN_URGENT * DT)
  assert limited > -2.0


def test_emergency_down_jerk_is_not_delayed():
  assert apply_tesla_accel_jerk_limit(-3.0, 0.5, DT) == pytest.approx(-3.0)


def test_unlimited_path_does_not_apply_when_releasing_brake():
  # Going from -3.0 toward 0 must still use the UP limiter.
  assert apply_tesla_accel_jerk_limit(0.0, -3.0, DT) == pytest.approx(-3.0 + UP * DT)


def _model3_long_controller():
  toggles = SimpleNamespace(tesla_cooperative_steering=False, trailer_load_kg=0.0)
  cp = CarInterface.get_params(CAR.TESLA_MODEL_3, gen_empty_fingerprint(), [], True, False, False, toggles)
  assert cp.openpilotLongitudinalControl
  state = SimpleNamespace(
    out=SimpleNamespace(
      vEgo=20.0, vEgoRaw=20.0, gasPressed=False,
      steeringAngleDeg=0.0, steeringDisengage=False, steeringTorque=0.0,
    ),
    hands_on_level=0,
  )
  return CarController(DBC[CAR.TESLA_MODEL_3], cp), state


def _set_accel(cc, accel, long_active=True, cancel=False):
  cc.longActive = long_active
  cc.cruiseControl.cancel = cancel
  cc.actuators.accel = accel
  return cc


def _sent_accel(controller, state, cc):
  _, sends = controller.update(cc.as_reader(), state, 0, None)
  long_msgs = [msg for msg in sends if msg[0] == 0x2B9]
  if not long_msgs:
    return None
  _, data, bus = long_msgs[0]
  parser = CANParser("tesla_model3_party", [(0x2B9, 0)], 0)
  parser.update([(1_000_000_000, [(0x2B9, data, bus)])])
  return parser.vl["DAS_control"]


def test_carcontroller_seeds_then_rate_limits_on_model3():
  controller, state = _model3_long_controller()
  state.out.vEgo = 20.0
  cc = structs.CarControl.new_message()

  decoded = _sent_accel(controller, state, _set_accel(cc, 1.5))
  assert decoded is not None
  assert decoded["DAS_accelMin"] == pytest.approx(1.5, abs=0.05)

  # 3 non-send frames, then the next 25 Hz command
  for _ in range(3):
    assert _sent_accel(controller, state, _set_accel(cc, 2.0)) is None
  decoded = _sent_accel(controller, state, _set_accel(cc, 2.0))
  assert decoded["DAS_accelMin"] == pytest.approx(1.5 + UP * DT, abs=0.05)
  assert decoded["DAS_accelMin"] < 2.0


def test_carcontroller_does_not_delay_emergency_brake():
  controller, state = _model3_long_controller()
  state.out.vEgo = 20.0
  cc = structs.CarControl.new_message()

  assert _sent_accel(controller, state, _set_accel(cc, 0.3))["DAS_accelMin"] == pytest.approx(0.3, abs=0.05)
  for _ in range(3):
    _sent_accel(controller, state, _set_accel(cc, 0.3))
  decoded = _sent_accel(controller, state, _set_accel(cc, -3.0))
  assert decoded["DAS_accelMin"] == pytest.approx(-3.0, abs=0.05)
