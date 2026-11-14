from types import SimpleNamespace
from time import monotonic

from cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import (
  AUTO_OVERTAKE_DELAY,
  AUTO_OVERTAKE_LEAD_DROPOUT_HOLD,
  AUTO_OVERTAKE_SPEED_MIN,
  DesireHelper,
  HIGHWAY_SPEED_LIMIT_MIN,
  LaneChangeDirection,
  LaneChangeState,
  STOPPED_LEAD_CONFIRM,
  StoppedLead,
  is_left_lane_clear,
  is_right_lane_clear,
  is_slower_lead,
  road_is_highway,
  stopped_lead_status,
)
from openpilot.selfdrive.controls.tests.test_navigation_desires import make_car_state, make_plan, make_toggles


HIGHWAY_V_EGO = 30.0  # m/s, well above 60 km/h
SET_SPEED_KPH = 130.0


def ao_toggles(**overrides):
  defaults = dict(
    auto_overtake=True,
    auto_overtake_return=False,
    overtake_return_delay=2.5,
    nudgeless=False,
  )
  defaults.update(overrides)
  return make_toggles(**defaults)


def step(helper, frames=1, *, carstate=None, plan=None, toggles=None, lateral_active=True,
         lane_change_prob=1.0, enabled=True, **ao):
  carstate = carstate or make_car_state(vEgo=HIGHWAY_V_EGO)
  plan = plan or make_plan()
  toggles = toggles or ao_toggles()
  ao.setdefault("slower_lead_detected", False)
  ao.setdefault("left_lane_clear", True)
  ao.setdefault("right_lane_clear", True)
  ao.setdefault("left_edge_detected", False)
  ao.setdefault("right_edge_detected", False)
  ao.setdefault("highway", True)
  ao.setdefault("stopped_lead", StoppedLead.NONE)
  for _ in range(frames):
    helper.update(
      carstate, lateral_active, lane_change_prob, plan, toggles, enabled, **ao,
    )
  return helper


def frames_for(seconds):
  return int(round(seconds / DT_MDL))


def start_overtake(helper=None, **kwargs):
  helper = helper or DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY), slower_lead_detected=True, **kwargs)
  # One more frame in preLaneChange to commit to starting
  step(helper, 1, slower_lead_detected=True, **kwargs)
  return helper


# --- helpers ---

def test_road_is_highway_requires_fresh_limit_at_least_110():
  assert road_is_highway(110 * CV.KPH_TO_MS, True)
  assert road_is_highway(130 * CV.KPH_TO_MS, True)
  assert not road_is_highway(109 * CV.KPH_TO_MS, True)
  assert not road_is_highway(90 * CV.KPH_TO_MS, True)
  assert not road_is_highway(110 * CV.KPH_TO_MS, False)
  assert not road_is_highway(0.0, True)
  assert not road_is_highway(0.0, False)
  assert not road_is_highway(float("nan"), True)
  assert not road_is_highway(None, True)
  assert road_is_highway(HIGHWAY_SPEED_LIMIT_MIN, True)


def test_is_slower_lead_thresholds():
  def leads(x=40.0, v=25.0, prob=0.8):
    return [SimpleNamespace(x=[x], v=[v], prob=prob)]

  # slightly slower than ego, within 70 m
  assert is_slower_lead(leads(v=HIGHWAY_V_EGO - 0.4), HIGHWAY_V_EGO, SET_SPEED_KPH)
  # matched speed, but at least 1 m/s under the set speed (130 kph ~ 36.1 m/s)
  assert is_slower_lead(leads(v=HIGHWAY_V_EGO), HIGHWAY_V_EGO, SET_SPEED_KPH)
  # too far
  assert not is_slower_lead(leads(x=80.0, v=20.0), HIGHWAY_V_EGO, SET_SPEED_KPH)
  # low confidence
  assert not is_slower_lead(leads(prob=0.4, v=20.0), HIGHWAY_V_EGO, SET_SPEED_KPH)
  # unset cruise: only relative-speed check
  assert not is_slower_lead(leads(v=HIGHWAY_V_EGO), HIGHWAY_V_EGO, 255.0)
  assert is_slower_lead(leads(v=HIGHWAY_V_EGO - 0.4), HIGHWAY_V_EGO, 255.0)


def test_stopped_lead_window_and_outside():
  def leads(x, v=0.2, prob=0.9):
    return [SimpleNamespace(x=[x], v=[v], prob=prob)]

  # at 30 m/s, window starts at max(30, 2*30)=60 m, up to 150 m
  assert stopped_lead_status(leads(80.0), HIGHWAY_V_EGO) == StoppedLead.WINDOW
  assert stopped_lead_status(leads(40.0), HIGHWAY_V_EGO) == StoppedLead.OUTSIDE
  assert stopped_lead_status(leads(200.0), HIGHWAY_V_EGO) == StoppedLead.OUTSIDE
  assert stopped_lead_status(leads(80.0, v=5.0), HIGHWAY_V_EGO) == StoppedLead.NONE
  assert stopped_lead_status(leads(80.0, prob=0.55), HIGHWAY_V_EGO) == StoppedLead.OUTSIDE


def test_left_and_right_lane_clear():
  def model(left_y=(-3.8, -1.8), right_y=(1.8, 3.8), probs=(0.8, 0.8, 0.8, 0.8), leads=None):
    lines = [SimpleNamespace(y=[y]) for y in (*left_y, *right_y)]
    return SimpleNamespace(laneLineProbs=list(probs), laneLines=lines, leadsV3=leads or [])

  assert is_left_lane_clear(model())
  assert is_right_lane_clear(model())
  # occupied left lane
  occ = [SimpleNamespace(prob=0.8, x=[30.0], y=[-2.8])]
  assert not is_left_lane_clear(model(leads=occ))
  assert is_right_lane_clear(model(leads=occ))
  # low line confidence
  assert not is_left_lane_clear(model(probs=(0.2, 0.8, 0.8, 0.8)))
  assert not is_right_lane_clear(model(probs=(0.8, 0.8, 0.2, 0.8)))


# --- state machine: trigger ---

def test_slower_lead_triggers_left_overtake_after_two_seconds():
  helper = DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) - 1, slower_lead_detected=True)
  assert helper.lane_change_state == LaneChangeState.off
  step(helper, 1, slower_lead_detected=True)
  assert helper.lane_change_state == LaneChangeState.preLaneChange
  assert helper.lane_change_direction == LaneChangeDirection.left
  assert helper.auto_overtake_active
  step(helper, 1, slower_lead_detected=True)
  assert helper.lane_change_state == LaneChangeState.laneChangeStarting
  assert helper.desire == log.Desire.laneChangeLeft


def test_stopped_lead_in_window_triggers_after_one_second():
  helper = DesireHelper()
  step(helper, frames_for(STOPPED_LEAD_CONFIRM) - 1, stopped_lead=StoppedLead.WINDOW)
  assert helper.lane_change_state == LaneChangeState.off
  step(helper, 1, stopped_lead=StoppedLead.WINDOW)
  assert helper.lane_change_state == LaneChangeState.preLaneChange
  step(helper, 1, stopped_lead=StoppedLead.WINDOW)
  assert helper.lane_change_state == LaneChangeState.laneChangeStarting
  assert helper.auto_overtake_active
  assert helper.lane_change_direction == LaneChangeDirection.left


def test_brief_lead_dropout_does_not_reset_overtake_timer():
  helper = DesireHelper()
  step(helper, frames_for(1.5), slower_lead_detected=True)
  step(helper, frames_for(AUTO_OVERTAKE_LEAD_DROPOUT_HOLD) - 1, slower_lead_detected=False)
  step(helper, frames_for(0.6), slower_lead_detected=True)
  assert helper.lane_change_state in (LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting)


# --- blocking conditions ---

def test_auto_overtake_off_does_not_trigger():
  helper = DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) + 5, slower_lead_detected=True, toggles=ao_toggles(auto_overtake=False))
  assert helper.lane_change_state == LaneChangeState.off
  assert not helper.auto_overtake_active


def test_lane_changes_disabled_blocks_overtake():
  helper = DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) + 5, slower_lead_detected=True, toggles=ao_toggles(lane_changes=False))
  assert helper.lane_change_state == LaneChangeState.off


def test_below_auto_overtake_speed_blocks():
  helper = DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) + 5, slower_lead_detected=True,
       carstate=make_car_state(vEgo=AUTO_OVERTAKE_SPEED_MIN - 0.5))
  assert helper.lane_change_state == LaneChangeState.off


def test_minimum_lane_change_speed_setting_is_respected():
  helper = DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) + 5, slower_lead_detected=True,
       carstate=make_car_state(vEgo=HIGHWAY_V_EGO),
       toggles=ao_toggles(minimum_lane_change_speed=HIGHWAY_V_EGO + 1.0))
  assert helper.lane_change_state == LaneChangeState.off


def test_left_blinker_blocks_overtake():
  helper = DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) + 5, slower_lead_detected=True,
       carstate=make_car_state(vEgo=HIGHWAY_V_EGO, leftBlinker=True))
  assert not helper.auto_overtake_active


def test_right_blinker_blocks_overtake():
  helper = DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) + 5, slower_lead_detected=True,
       carstate=make_car_state(vEgo=HIGHWAY_V_EGO, rightBlinker=True))
  assert not helper.auto_overtake_active


def test_left_bsm_blocks_overtake():
  helper = DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) + 5, slower_lead_detected=True,
       carstate=make_car_state(vEgo=HIGHWAY_V_EGO, leftBlindspot=True))
  assert helper.lane_change_state == LaneChangeState.off


def test_left_lane_not_clear_blocks_overtake():
  helper = DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) + 5, slower_lead_detected=True, left_lane_clear=False)
  assert helper.lane_change_state == LaneChangeState.off


def test_left_road_edge_blocks_overtake():
  helper = DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) + 5, slower_lead_detected=True, left_edge_detected=True)
  assert helper.lane_change_state == LaneChangeState.off


def test_stopped_lead_outside_window_does_not_overtake():
  helper = DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) + 5, slower_lead_detected=True, stopped_lead=StoppedLead.OUTSIDE)
  assert helper.lane_change_state == LaneChangeState.off


def test_lateral_inactive_blocks_overtake():
  helper = DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) + 5, slower_lead_detected=True, lateral_active=False)
  assert helper.lane_change_state == LaneChangeState.off


# --- highway gate fail closed ---

def test_highway_gate_false_blocks_overtake():
  helper = DesireHelper()
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) + 5, slower_lead_detected=True, highway=False)
  assert helper.lane_change_state == LaneChangeState.off
  assert helper.overtake_timer == 0.0


def test_highway_gate_helper_fail_closed_when_stale_or_zero():
  # modeld uses this helper; stale or 0 must never count as highway
  assert not road_is_highway(130 * CV.KPH_TO_MS, False)
  assert not road_is_highway(0, True)


# --- nudgeless and nav unchanged ---

def test_nudgeless_still_requires_blinker_when_auto_overtake_is_off():
  helper = DesireHelper()
  toggles = ao_toggles(auto_overtake=False, nudgeless=True, lane_change_delay=0.0)
  # no blinker, slower lead on a highway: must not start a lane change
  step(helper, frames_for(AUTO_OVERTAKE_DELAY) + 5, slower_lead_detected=True, toggles=toggles)
  assert helper.lane_change_state == LaneChangeState.off

  # blinker edge still starts a nudgeless change
  helper = DesireHelper()
  step(helper, 1, toggles=toggles, slower_lead_detected=False, highway=False,
       carstate=make_car_state(vEgo=HIGHWAY_V_EGO, leftBlinker=False))
  step(helper, 1, toggles=toggles, slower_lead_detected=False, highway=False,
       carstate=make_car_state(vEgo=HIGHWAY_V_EGO, leftBlinker=True))
  # wait timer accumulates; with delay 0 and width ok, next frames should start
  step(helper, 3, toggles=toggles, slower_lead_detected=False, highway=False,
       carstate=make_car_state(vEgo=HIGHWAY_V_EGO, leftBlinker=True))
  assert helper.lane_change_state == LaneChangeState.laneChangeStarting
  assert not helper.auto_overtake_active
  assert helper.lane_change_direction == LaneChangeDirection.left


def test_nudgeless_blinker_path_still_works_with_auto_overtake_enabled():
  helper = DesireHelper()
  toggles = ao_toggles(auto_overtake=True, nudgeless=True, lane_change_delay=0.0)
  step(helper, 1, toggles=toggles, carstate=make_car_state(vEgo=HIGHWAY_V_EGO, leftBlinker=False))
  step(helper, 3, toggles=toggles, carstate=make_car_state(vEgo=HIGHWAY_V_EGO, leftBlinker=True))
  assert helper.lane_change_state == LaneChangeState.laneChangeStarting
  assert not helper.auto_overtake_active


def test_nav_keep_left_unchanged_when_overtake_conditions_are_absent():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {"valid": True, "maneuverModifier": "slightLeft", "updatedAtMonotonic": monotonic()}
  toggles = ao_toggles(auto_overtake=True, nudgeless=True)
  helper.update(
    make_car_state(vEgo=20.0, steeringPressed=True, steeringTorque=1.0),
    True,
    0.0,
    make_plan(laneWidthLeft=4.2),
    toggles,
    slower_lead_detected=False,
    highway=False,
  )
  assert helper.desire == log.Desire.keepLeft
  assert helper.lane_change_state == LaneChangeState.off


def test_nav_desires_do_not_override_an_active_overtake():
  helper = start_overtake()
  assert helper.lane_change_state == LaneChangeState.laneChangeStarting
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {"valid": True, "maneuverModifier": "slightLeft", "updatedAtMonotonic": monotonic()}
  step(helper, 1, slower_lead_detected=True, lane_change_prob=1.0,
       carstate=make_car_state(vEgo=HIGHWAY_V_EGO, steeringPressed=True, steeringTorque=1.0),
       toggles=ao_toggles(nudgeless=True))
  assert helper.desire == log.Desire.laneChangeLeft
  assert helper.lane_change_state == LaneChangeState.laneChangeStarting


# --- return path ---

from openpilot.selfdrive.controls.lib.desire_helper import (
  AUTO_OVERTAKE_RETURN_CONFIRM,
  AUTO_OVERTAKE_RETURN_SETTLE,
  AUTO_OVERTAKE_RETURN_TIMEOUT,
  AutoLaneChangeKind,
)


def finish_active_lane_change(helper, **kwargs):
  kwargs.setdefault("lane_change_prob", 0.0)
  kwargs.setdefault("slower_lead_detected", False)
  saw_finishing = False
  for _ in range(frames_for(2.5) + 5):
    if helper.lane_change_state == LaneChangeState.off and saw_finishing:
      break
    if helper.lane_change_state == LaneChangeState.laneChangeFinishing:
      saw_finishing = True
    step(helper, 1, **kwargs)
    if helper.lane_change_state == LaneChangeState.off and saw_finishing:
      break
  assert saw_finishing
  assert helper.lane_change_state == LaneChangeState.off


def complete_left_overtake(helper=None, **kwargs):
  helper = start_overtake(helper, **kwargs)
  finish_active_lane_change(helper, **kwargs)
  assert helper.return_pending
  return helper


def test_left_overtake_arms_a_return_to_the_right():
  helper = complete_left_overtake()
  assert helper.return_pending
  assert helper.lane_change_state == LaneChangeState.off
  assert not helper.auto_overtake_active


def test_blinker_return_starts_after_delay_and_settle():
  helper = complete_left_overtake()
  # Wait the settle period with no blinker; the 2.5 s delay is shorter than settle.
  step(helper, frames_for(AUTO_OVERTAKE_RETURN_SETTLE),
       carstate=make_car_state(vEgo=HIGHWAY_V_EGO))
  assert helper.lane_change_state == LaneChangeState.off
  assert helper.return_pending
  # Rising edge of the right blinker starts the owed return without a nudge.
  step(helper, 1, carstate=make_car_state(vEgo=HIGHWAY_V_EGO, rightBlinker=True))
  assert helper.lane_change_state == LaneChangeState.preLaneChange
  assert helper.lane_change_direction == LaneChangeDirection.right
  assert helper.auto_overtake_active
  step(helper, 1, carstate=make_car_state(vEgo=HIGHWAY_V_EGO, rightBlinker=True))
  assert helper.lane_change_state == LaneChangeState.laneChangeStarting
  assert helper.desire == log.Desire.laneChangeRight


def test_early_right_blinker_is_held_until_return_delay():
  helper = complete_left_overtake()
  # Signal during the delay: do not start yet, then start once 2.5 s have elapsed.
  step(helper, 1, carstate=make_car_state(vEgo=HIGHWAY_V_EGO, rightBlinker=True))
  step(helper, frames_for(2.5) - 2, carstate=make_car_state(vEgo=HIGHWAY_V_EGO, rightBlinker=True))
  assert helper.lane_change_state == LaneChangeState.off
  assert helper.return_pending
  step(helper, 4, carstate=make_car_state(vEgo=HIGHWAY_V_EGO, rightBlinker=True))
  assert helper.lane_change_state in (LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting)
  assert helper.lane_change_direction == LaneChangeDirection.right


def test_auto_return_starts_without_blinker_when_enabled():
  helper = complete_left_overtake(toggles=ao_toggles(auto_overtake_return=True))
  step(helper, frames_for(AUTO_OVERTAKE_RETURN_SETTLE) + frames_for(AUTO_OVERTAKE_RETURN_CONFIRM),
       toggles=ao_toggles(auto_overtake_return=True), slower_lead_detected=False, highway=True)
  assert helper.lane_change_state in (LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting)
  assert helper.auto_return_active or helper.auto_overtake_active
  assert helper.lane_change_direction == LaneChangeDirection.right
  step(helper, 1, toggles=ao_toggles(auto_overtake_return=True), highway=True)
  assert helper.lane_change_state == LaneChangeState.laneChangeStarting
  assert helper.auto_return_active
  assert helper.auto_lane_change_kind == AutoLaneChangeKind.RETURN
  assert helper.desire == log.Desire.laneChangeRight


def test_auto_return_requires_highway():
  helper = complete_left_overtake(toggles=ao_toggles(auto_overtake_return=True))
  step(helper, frames_for(AUTO_OVERTAKE_RETURN_SETTLE) + frames_for(AUTO_OVERTAKE_RETURN_CONFIRM) + 5,
       toggles=ao_toggles(auto_overtake_return=True), highway=False)
  assert helper.lane_change_state == LaneChangeState.off
  assert helper.return_pending


def test_auto_return_blocked_by_right_bsm():
  helper = complete_left_overtake(toggles=ao_toggles(auto_overtake_return=True))
  step(helper, frames_for(AUTO_OVERTAKE_RETURN_SETTLE) + frames_for(AUTO_OVERTAKE_RETURN_CONFIRM) + 5,
       toggles=ao_toggles(auto_overtake_return=True),
       carstate=make_car_state(vEgo=HIGHWAY_V_EGO, rightBlindspot=True))
  assert helper.lane_change_state == LaneChangeState.off
  assert helper.return_pending


def test_auto_return_blocked_when_right_lane_not_clear():
  helper = complete_left_overtake(toggles=ao_toggles(auto_overtake_return=True))
  step(helper, frames_for(AUTO_OVERTAKE_RETURN_SETTLE) + frames_for(AUTO_OVERTAKE_RETURN_CONFIRM) + 5,
       toggles=ao_toggles(auto_overtake_return=True), right_lane_clear=False)
  assert helper.lane_change_state == LaneChangeState.off
  assert helper.return_pending


def test_auto_return_blocked_by_right_road_edge():
  helper = complete_left_overtake(toggles=ao_toggles(auto_overtake_return=True))
  step(helper, frames_for(AUTO_OVERTAKE_RETURN_SETTLE) + frames_for(AUTO_OVERTAKE_RETURN_CONFIRM) + 5,
       toggles=ao_toggles(auto_overtake_return=True), right_edge_detected=True)
  assert helper.lane_change_state == LaneChangeState.off
  assert helper.return_pending


def test_return_cancelled_by_timeout_or_left_input():
  helper = complete_left_overtake()
  step(helper, frames_for(AUTO_OVERTAKE_RETURN_TIMEOUT) + 1)
  assert not helper.return_pending

  helper = complete_left_overtake()
  step(helper, 2, carstate=make_car_state(vEgo=HIGHWAY_V_EGO, leftBlinker=True))
  assert not helper.return_pending

  helper = complete_left_overtake()
  step(helper, 2, carstate=make_car_state(vEgo=HIGHWAY_V_EGO, steeringPressed=True, steeringTorque=1.0))
  assert not helper.return_pending
