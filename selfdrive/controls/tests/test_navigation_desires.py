from types import SimpleNamespace
from time import monotonic

from cereal import log

from openpilot.selfdrive.controls.lib.desire_helper import AutoLaneChangeKind, DesireHelper, LaneChangeDirection, LaneChangeState


def make_car_state(**overrides):
  defaults = {
    "vEgo": 20.0,
    "leftBlinker": False,
    "rightBlinker": False,
    "leftBlindspot": False,
    "rightBlindspot": False,
    "steeringPressed": False,
    "steeringTorque": 0.0,
    "standstill": False,
    "cruiseState": SimpleNamespace(enabled=True),
  }
  defaults.update(overrides)
  return SimpleNamespace(**defaults)


def make_toggles(**overrides):
  defaults = {
    "lane_changes": True,
    "lane_change_delay": 0.0,
    "lane_detection_width": 3.0,
    "minimum_lane_change_speed": 10.0,
    "nudgeless": True,
    "nudgeless_lane_change_only_when_engaged": False,
    "one_lane_change": False,
    "use_turn_desires": False,
    "lane_changes_require_cruise": False,
    "nav_desires_allowed": True,
    "nav_lane_positioning_allowed": True,
    "nav_lane_change": False,
  }
  defaults.update(overrides)
  return SimpleNamespace(**defaults)


def make_plan(**overrides):
  defaults = {
    "laneWidthLeft": 4.0,
    "laneWidthRight": 4.0,
  }
  defaults.update(overrides)
  return SimpleNamespace(**defaults)

def test_nav_desires_keep_left_when_route_requests_it():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {"valid": True, "maneuverModifier": "slightLeft", "updatedAtMonotonic": monotonic()}

  helper.update(
    make_car_state(vEgo=20.0, steeringPressed=True, steeringTorque=1.0),
    True,
    0.0,
    make_plan(laneWidthLeft=4.2),
    make_toggles(nudgeless=True),
  )

  assert helper.desire == log.Desire.keepLeft


def test_nav_desires_turn_right_below_lane_change_speed():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True, "maneuverType": "turn", "maneuverModifier": "right",
    "maneuverDistance": 10.0, "updatedAtMonotonic": monotonic(),
  }

  helper.update(
    make_car_state(vEgo=5.0, rightBlinker=True),
    True,
    0.0,
    make_plan(),
    make_toggles(minimum_lane_change_speed=10.0, nav_lane_positioning_allowed=False),
  )

  assert helper.desire == log.Desire.turnRight


def test_nav_desires_turn_preview_starts_before_last_second():
  helper = DesireHelper()
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True, "maneuverType": "turn", "maneuverModifier": "right",
    "maneuverDistance": 50.0, "updatedAtMonotonic": monotonic(),
  }

  helper.update(
    make_car_state(vEgo=10.5, rightBlinker=True),
    True,
    0.0,
    make_plan(),
    make_toggles(minimum_lane_change_speed=11.1),
  )

  assert helper.desire == log.Desire.turnRight
  assert helper.lane_change_state == LaneChangeState.off


def test_routed_turn_replays_signaled_approach_independently_of_lane_change_setting():
  samples = (
    (82.6, 13.64, True, log.Desire.none),
    (69.1, 13.15, True, log.Desire.turnRight),
    (56.3, 13.18, True, log.Desire.turnRight),
    (43.1, 13.00, False, log.Desire.none),
  )
  for lane_change_speed in (2.777777, 11.1):
    helper = DesireHelper()
    helper._update_nav_params = lambda: None
    toggles = make_toggles(minimum_lane_change_speed=lane_change_speed)
    for distance, speed, right_blinker, expected in samples:
      helper._nav_instruction_state = {
        "valid": True,
        "maneuverType": "turn",
        "maneuverModifier": "right",
        "maneuverDistance": distance,
        "updatedAtMonotonic": monotonic(),
      }
      helper.update(
        make_car_state(vEgo=speed, rightBlinker=right_blinker),
        True,
        0.0,
        make_plan(),
        toggles,
      )
      assert helper.desire == expected
      assert helper.lane_change_state == LaneChangeState.off
      assert helper.turn_direction == expected


def test_stale_route_instruction_cannot_request_turn_or_suppress_lane_change():
  helper = DesireHelper()
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True,
    "maneuverType": "turn",
    "maneuverModifier": "right",
    "maneuverDistance": 55.0,
    "updatedAtMonotonic": monotonic() - 10.0,
  }
  helper.update(
    make_car_state(vEgo=13.0, rightBlinker=True),
    True,
    0.0,
    make_plan(),
    make_toggles(minimum_lane_change_speed=2.777777),
  )

  assert helper.desire == log.Desire.none
  assert helper.lane_change_state == LaneChangeState.preLaneChange


def test_nav_desires_turn_preview_is_bounded_and_requires_matching_signal():
  for distance, blinker, speed, maneuver_type in (
    (65.0, True, 10.5, "turn"),
    (-1.0, True, 10.5, "turn"),
    (50.0, False, 10.5, "turn"),
    (50.0, True, 14.2, "turn"),
    (50.0, True, 10.5, "arrive"),
  ):
    helper = DesireHelper()
    helper._update_nav_params = lambda: None
    helper._nav_instruction_state = {
      "valid": True, "maneuverType": maneuver_type, "maneuverModifier": "right",
      "maneuverDistance": distance, "updatedAtMonotonic": monotonic(),
    }

    helper.update(
      make_car_state(vEgo=speed, rightBlinker=blinker),
      True,
      0.0,
      make_plan(),
      make_toggles(minimum_lane_change_speed=11.1),
    )

    assert helper.desire == log.Desire.none


def test_nav_desires_turn_preview_respects_stop_hold():
  helper = DesireHelper()
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True, "maneuverType": "turn", "maneuverModifier": "right",
    "maneuverDistance": 20.0, "updatedAtMonotonic": monotonic(),
  }

  helper.update(
    make_car_state(vEgo=5.0, rightBlinker=True),
    True,
    0.0,
    make_plan(redLight=True),
    make_toggles(minimum_lane_change_speed=11.1),
  )

  assert helper.turn_stop_hold
  assert helper.desire == log.Desire.none


def test_nav_desires_turn_requires_matching_blinker():
  for modifier, opposite_blinker in (("left", "rightBlinker"), ("right", "leftBlinker")):
    helper = DesireHelper()
    helper.nav_desires_allowed = True
    helper._update_nav_params = lambda: None
    helper._nav_instruction_state = {
      "valid": True, "maneuverType": "turn", "maneuverModifier": modifier,
      "maneuverDistance": 10.0, "updatedAtMonotonic": monotonic(),
    }

    helper.update(
      make_car_state(vEgo=5.0, **{opposite_blinker: True}),
      True,
      0.0,
      make_plan(),
      make_toggles(minimum_lane_change_speed=10.0, nav_lane_positioning_allowed=False),
    )

    assert helper.desire == log.Desire.none


def test_nav_desires_turn_right_waits_until_turn_is_close():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True, "maneuverType": "turn", "maneuverModifier": "right",
    "maneuverDistance": 300.0, "updatedAtMonotonic": monotonic(),
  }

  helper.update(
    make_car_state(vEgo=5.0),
    True,
    0.0,
    make_plan(),
    make_toggles(minimum_lane_change_speed=10.0),
  )

  assert helper.desire == log.Desire.none


def test_matching_routed_turn_does_not_start_lane_change_above_threshold():
  helper = DesireHelper()
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True,
    "updatedAtMonotonic": monotonic(),
    "maneuverType": "turn",
    "maneuverModifier": "right",
    "maneuverDistance": 111.0,
  }

  helper.update(
    make_car_state(vEgo=16.0, rightBlinker=True),
    True,
    0.0,
    make_plan(),
    make_toggles(minimum_lane_change_speed=11.1),
  )

  assert helper.lane_change_state == LaneChangeState.off
  assert helper.lane_change_direction == LaneChangeDirection.none
  assert helper.desire == log.Desire.none


def test_distant_routed_turn_does_not_block_lane_change():
  helper = DesireHelper()
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True,
    "updatedAtMonotonic": monotonic(),
    "maneuverType": "turn",
    "maneuverModifier": "right",
    "maneuverDistance": 794.0,
  }

  helper.update(
    make_car_state(vEgo=16.0, rightBlinker=True),
    True,
    0.0,
    make_plan(),
    make_toggles(minimum_lane_change_speed=11.1),
  )

  assert helper.lane_change_state == LaneChangeState.preLaneChange
  assert helper.lane_change_direction == LaneChangeDirection.right


def test_matching_routed_turn_cancels_pending_lane_change_before_it_starts():
  helper = DesireHelper()
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True,
    "updatedAtMonotonic": monotonic(),
    "maneuverType": "turn",
    "maneuverModifier": "left",
    "maneuverDistance": 125.0,
  }
  helper.lane_change_state = LaneChangeState.preLaneChange
  helper.lane_change_direction = LaneChangeDirection.left
  helper.prev_one_blinker = True

  helper.update(
    make_car_state(vEgo=11.0, leftBlinker=True),
    True,
    0.0,
    make_plan(),
    make_toggles(minimum_lane_change_speed=10.0),
  )

  assert helper.lane_change_state == LaneChangeState.off
  assert helper.lane_change_direction == LaneChangeDirection.none


def test_nav_desires_off_ramp_lane_guidance_becomes_keep_right():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True,
    "updatedAtMonotonic": monotonic(),
    "maneuverType": "off ramp",
    "maneuverModifier": "right",
    "activeLaneDirection": "slightRight",
    "maneuverDistance": 120.0,
  }

  helper.update(
    make_car_state(vEgo=22.5, steeringPressed=True, steeringTorque=-1.0),
    True,
    0.0,
    make_plan(laneWidthRight=4.2),
    make_toggles(nudgeless=True),
  )

  assert helper.desire == log.Desire.keepRight


def test_nav_desires_off_ramp_lane_guidance_waits_until_split_is_close():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True,
    "updatedAtMonotonic": monotonic(),
    "maneuverType": "off ramp",
    "maneuverModifier": "right",
    "activeLaneDirection": "slightRight",
    "maneuverDistance": 300.0,
  }

  helper.update(
    make_car_state(vEgo=22.5),
    True,
    0.0,
    make_plan(laneWidthRight=4.2),
    make_toggles(nudgeless=True),
  )

  assert helper.desire == log.Desire.none


def test_nav_desires_ambiguous_off_ramp_waits_longer_before_keep_right():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True,
    "updatedAtMonotonic": monotonic(),
    "maneuverType": "off ramp",
    "maneuverModifier": "right",
    "activeLaneDirection": "slightRight",
    "sameSideLaneCount": 3,
    "maneuverDistance": 120.0,
  }

  helper.update(
    make_car_state(vEgo=22.5),
    True,
    0.0,
    make_plan(laneWidthRight=4.2),
    make_toggles(nudgeless=True),
  )

  assert helper.desire == log.Desire.none


def test_nav_desires_edge_exit_lane_with_shared_transition_lane_does_not_keep_right():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True,
    "updatedAtMonotonic": monotonic(),
    "maneuverType": "off ramp",
    "maneuverModifier": "right",
    "activeLaneDirection": "slightRight",
    "laneCount": 3,
    "sameSideLaneCount": 2,
    "activeLaneAtRoadEdge": True,
    "hasSharedSameSideLane": True,
    "maneuverDistance": 10.0,
  }

  helper.update(
    make_car_state(vEgo=22.5),
    True,
    0.0,
    make_plan(laneWidthRight=4.2),
    make_toggles(nudgeless=True),
  )

  assert helper.desire == log.Desire.none


def test_nav_desires_wide_highway_edge_exit_lane_keeps_right():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True,
    "updatedAtMonotonic": monotonic(),
    "maneuverType": "off ramp",
    "maneuverModifier": "right",
    "activeLaneDirection": "slightRight",
    "laneCount": 7,
    "sameSideLaneCount": 2,
    "activeLaneAtRoadEdge": True,
    "hasSharedSameSideLane": True,
    "maneuverDistance": 105.0,
  }

  helper.update(
    make_car_state(vEgo=19.0, steeringPressed=True, steeringTorque=-1.0),
    True,
    0.0,
    make_plan(laneWidthRight=4.2),
    make_toggles(nudgeless=True),
  )

  assert helper.desire == log.Desire.keepRight


def test_nav_desires_shared_transition_lane_keeps_when_active_lane_is_not_outermost():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True,
    "updatedAtMonotonic": monotonic(),
    "maneuverType": "off ramp",
    "maneuverModifier": "right",
    "activeLaneDirection": "slightRight",
    "sameSideLaneCount": 2,
    "activeLaneAtRoadEdge": False,
    "hasSharedSameSideLane": True,
    "maneuverDistance": 10.0,
  }

  helper.update(
    make_car_state(vEgo=22.5, steeringPressed=True, steeringTorque=-1.0),
    True,
    0.0,
    make_plan(laneWidthRight=4.2),
    make_toggles(nudgeless=True),
  )

  assert helper.desire == log.Desire.keepRight


def test_nav_desires_ambiguous_fork_slight_right_only_keeps_close_to_split():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True,
    "updatedAtMonotonic": monotonic(),
    "maneuverType": "fork",
    "maneuverModifier": "slightRight",
    "activeLaneDirection": "slightRight",
    "sameSideLaneCount": 3,
    "maneuverDistance": 60.0,
  }

  helper.update(
    make_car_state(vEgo=22.5, steeringPressed=True, steeringTorque=-1.0),
    True,
    0.0,
    make_plan(laneWidthRight=4.2),
    make_toggles(nudgeless=True),
  )

  assert helper.desire == log.Desire.keepRight


def test_nav_desires_ambiguous_fork_slight_right_does_not_nudge_too_early():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True,
    "updatedAtMonotonic": monotonic(),
    "maneuverType": "fork",
    "maneuverModifier": "slightRight",
    "activeLaneDirection": "slightRight",
    "sameSideLaneCount": 3,
    "maneuverDistance": 120.0,
  }

  helper.update(
    make_car_state(vEgo=22.5),
    True,
    0.0,
    make_plan(laneWidthRight=4.2),
    make_toggles(nudgeless=True),
  )

  assert helper.desire == log.Desire.none


def test_nav_desires_fork_with_active_straight_lane_does_not_turn_left():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {
    "valid": True,
    "updatedAtMonotonic": monotonic(),
    "maneuverType": "fork",
    "maneuverModifier": "left",
    "activeLaneDirection": "straight",
    "maneuverDistance": 15.0,
  }

  helper.update(
    make_car_state(vEgo=5.0),
    True,
    0.0,
    make_plan(laneWidthLeft=4.2),
    make_toggles(nudgeless=True, minimum_lane_change_speed=10.0),
  )

  assert helper.desire == log.Desire.none


def test_nav_desires_do_not_override_lane_change_state_machine():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {"valid": True, "maneuverModifier": "slightRight", "updatedAtMonotonic": monotonic()}
  helper.lane_change_state = LaneChangeState.laneChangeStarting
  helper.lane_change_direction = LaneChangeDirection.left
  helper.lane_change_ll_prob = 0.5

  helper.update(
    make_car_state(vEgo=25.0, leftBlinker=True),
    True,
    0.5,
    make_plan(),
    make_toggles(),
  )

  assert helper.desire == log.Desire.laneChangeLeft


def test_lane_changes_require_cruise_blocks_blinker_lane_change_without_cruise():
  helper = DesireHelper()

  helper.update(
    make_car_state(leftBlinker=True, cruiseState=SimpleNamespace(enabled=False)),
    True,
    0.0,
    make_plan(),
    make_toggles(lane_changes_require_cruise=True),
  )

  assert helper.lane_change_state == LaneChangeState.off
  assert helper.lane_change_direction == LaneChangeDirection.none
  assert helper.desire == log.Desire.none


def test_lane_changes_require_cruise_allows_blinker_lane_change_with_cruise():
  helper = DesireHelper()

  helper.update(
    make_car_state(leftBlinker=True, cruiseState=SimpleNamespace(enabled=True)),
    True,
    0.0,
    make_plan(),
    make_toggles(lane_changes_require_cruise=True),
  )

  assert helper.lane_change_state == LaneChangeState.preLaneChange
  assert helper.lane_change_direction == LaneChangeDirection.left


def test_lane_changes_without_cruise_requirement_keep_existing_behavior():
  helper = DesireHelper()

  helper.update(
    make_car_state(leftBlinker=True, cruiseState=SimpleNamespace(enabled=False)),
    True,
    0.0,
    make_plan(),
    make_toggles(lane_changes_require_cruise=False),
  )

  assert helper.lane_change_state == LaneChangeState.preLaneChange
  assert helper.lane_change_direction == LaneChangeDirection.left


def test_nudgeless_only_when_engaged_allows_automatic_lane_change_when_engaged():
  helper = DesireHelper()

  for _ in range(2):
    helper.update(
      make_car_state(leftBlinker=True),
      True,
      0.0,
      make_plan(),
      make_toggles(nudgeless_lane_change_only_when_engaged=True),
      controls_enabled=True,
    )

  assert helper.lane_change_state == LaneChangeState.laneChangeStarting
  assert helper.lane_change_direction == LaneChangeDirection.left


def test_nudgeless_only_when_engaged_requires_nudge_when_aol_only():
  helper = DesireHelper()
  toggles = make_toggles(nudgeless_lane_change_only_when_engaged=True)

  for _ in range(2):
    helper.update(
      make_car_state(leftBlinker=True),
      True,
      0.0,
      make_plan(),
      toggles,
      controls_enabled=False,
    )

  assert helper.lane_change_state == LaneChangeState.preLaneChange
  assert helper.lane_change_direction == LaneChangeDirection.left

  helper.update(
    make_car_state(leftBlinker=True, steeringPressed=True, steeringTorque=1.0),
    True,
    0.0,
    make_plan(),
    toggles,
    controls_enabled=False,
  )

  assert helper.lane_change_state == LaneChangeState.laneChangeStarting
  assert helper.lane_change_direction == LaneChangeDirection.left


def test_nav_desires_nudgeless_only_when_engaged_blocks_keep_when_aol_only():
  helper = DesireHelper()
  helper.nav_desires_allowed = True
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {"valid": True, "maneuverModifier": "slightLeft", "updatedAtMonotonic": monotonic()}

  helper.update(
    make_car_state(vEgo=20.0),
    True,
    0.0,
    make_plan(laneWidthLeft=4.2),
    make_toggles(nudgeless=True, nudgeless_lane_change_only_when_engaged=True),
    controls_enabled=False,
  )

  assert helper.desire == log.Desire.none

  helper.update(
    make_car_state(vEgo=20.0, steeringPressed=True, steeringTorque=-1.0),
    True,
    0.0,
    make_plan(laneWidthRight=4.2),
    make_toggles(nav_desires_allowed=True, nav_lane_positioning_allowed=False, nudgeless=True),
  )

  assert helper.desire == log.Desire.none


def test_turn_desire_fires_below_lane_change_speed_when_no_stop():
  helper = DesireHelper()

  helper.update(
    make_car_state(vEgo=5.0, rightBlinker=True),
    True,
    0.0,
    make_plan(),
    make_toggles(use_turn_desires=True, minimum_lane_change_speed=10.0),
  )

  assert helper.desire == log.Desire.turnRight


def test_turn_desire_held_while_stopping_for_red_light():
  helper = DesireHelper()

  helper.update(
    make_car_state(vEgo=5.0, rightBlinker=True),
    True,
    0.0,
    make_plan(redLight=True),
    make_toggles(use_turn_desires=True, minimum_lane_change_speed=10.0),
  )

  assert helper.turn_stop_hold
  assert helper.desire == log.Desire.none


def test_turn_desire_released_after_stop_completes():
  helper = DesireHelper()
  toggles = make_toggles(use_turn_desires=True, minimum_lane_change_speed=10.0)

  helper.update(make_car_state(vEgo=5.0, rightBlinker=True), True, 0.0, make_plan(redLight=True), toggles)
  assert helper.desire == log.Desire.none

  helper.update(make_car_state(vEgo=0.0, rightBlinker=True, standstill=True), True, 0.0, make_plan(redLight=True), toggles)
  assert not helper.turn_stop_hold

  helper.update(make_car_state(vEgo=2.0, rightBlinker=True), True, 0.0, make_plan(), toggles)
  assert helper.desire == log.Desire.turnRight


def test_nav_desires_disabled_leave_desire_unchanged():
  helper = DesireHelper()
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {"valid": True, "maneuverModifier": "left", "updatedAtMonotonic": monotonic()}

  helper.update(
    make_car_state(vEgo=5.0),
    True,
    0.0,
    make_plan(),
    make_toggles(minimum_lane_change_speed=10.0, nav_desires_allowed=False),
  )

  assert helper.desire == log.Desire.none


def test_disabling_nav_desires_clears_active_route_desire_immediately():
  helper = DesireHelper()
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {"valid": True, "maneuverModifier": "slightRight", "updatedAtMonotonic": monotonic()}
  car_state = make_car_state(vEgo=20.0, steeringPressed=True, steeringTorque=-1.0)
  plan = make_plan(laneWidthRight=4.2)

  helper.update(car_state, True, 0.0, plan, make_toggles(nav_desires_allowed=True, nav_lane_positioning_allowed=True))
  assert helper.desire == log.Desire.keepRight

  helper.update(car_state, True, 0.0, plan, make_toggles(nav_desires_allowed=False, nav_lane_positioning_allowed=True))
  assert helper.desire == log.Desire.none


def test_nav_lane_positioning_requires_driver_confirmation():
  helper = DesireHelper()
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = {"valid": True, "maneuverModifier": "slightRight", "updatedAtMonotonic": monotonic()}

  helper.update(
    make_car_state(vEgo=20.0),
    True,
    0.0,
    make_plan(laneWidthRight=4.2),
    make_toggles(nav_desires_allowed=True, nav_lane_positioning_allowed=True, nudgeless=True),
  )

  assert helper.desire == log.Desire.none

  helper.update(
    make_car_state(vEgo=20.0, steeringPressed=True, steeringTorque=-1.0),
    True,
    0.0,
    make_plan(laneWidthRight=4.2),
    make_toggles(nav_desires_allowed=True, nav_lane_positioning_allowed=False, nudgeless=True),
  )

  assert helper.desire == log.Desire.none


from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import NAV_LANE_CHANGE_DELAY, NAV_LANE_CHANGE_MAX_PER_MANEUVER, NAV_LANE_CHANGE_MIN_DISTANCE, NAV_LANE_CHANGE_SETTLE


def _frames_for(seconds):
  return int(round(seconds / DT_MDL))


def make_nav_instruction(**overrides):
  state = {
    "valid": True,
    "updatedAtMonotonic": monotonic(),
    "maneuverType": "off ramp",
    "maneuverModifier": "right",
    "maneuverDistance": 300.0,
    "activeLaneDirection": "slightRight",
    "maneuverPrimaryText": "Exit 12",
  }
  state.update(overrides)
  return state


def arm_nav(helper, **overrides):
  helper._update_nav_params = lambda: None
  helper._nav_instruction_state = make_nav_instruction(**overrides)
  return helper


def nav_toggles(**overrides):
  return make_toggles(nav_lane_change=True, **overrides)


def step_nav(helper, frames=1, *, carstate=None, plan=None, toggles=None, lateral_active=True,
             lane_change_prob=1.0, **kwargs):
  carstate = carstate or make_car_state(vEgo=30.0)
  plan = plan or make_plan()
  toggles = toggles or nav_toggles()
  kwargs.setdefault("slower_lead_detected", False)
  kwargs.setdefault("left_lane_clear", True)
  kwargs.setdefault("right_lane_clear", True)
  kwargs.setdefault("left_edge_detected", False)
  kwargs.setdefault("right_edge_detected", False)
  kwargs.setdefault("right_lane_oncoming", False)
  kwargs.setdefault("highway", True)
  for _ in range(frames):
    helper.update(carstate, lateral_active, lane_change_prob, plan, toggles, True, **kwargs)
  return helper


def start_nav_lane_change(helper=None, **kwargs):
  helper = helper or DesireHelper()
  nav = kwargs.pop("nav", {})
  arm_nav(helper, **nav)
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY), **kwargs)
  # One more frame in preLaneChange to commit to starting
  step_nav(helper, 1, **kwargs)
  return helper


def test_nav_lane_change_off_does_not_trigger():
  helper = arm_nav(DesireHelper())
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) + 5, toggles=make_toggles(nav_lane_change=False))
  assert helper.lane_change_state == LaneChangeState.off
  assert not helper.auto_nav_active


def test_nav_lane_change_off_ramp_right_starts_blinkerless_change():
  helper = start_nav_lane_change()
  assert helper.lane_change_state == LaneChangeState.laneChangeStarting
  assert helper.lane_change_direction == LaneChangeDirection.right
  assert helper.auto_nav_active
  assert helper.auto_overtake_active
  assert helper.desire == log.Desire.laneChangeRight
  assert helper.auto_lane_change_kind == AutoLaneChangeKind.NAV
  assert not helper.return_pending


def test_nav_lane_change_waits_for_confirm_delay():
  helper = arm_nav(DesireHelper())
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) - 1)
  assert helper.lane_change_state == LaneChangeState.off
  step_nav(helper, 1)
  assert helper.lane_change_state == LaneChangeState.preLaneChange
  assert helper.auto_nav_active


def test_nav_lane_change_continue_keep_left():
  helper = start_nav_lane_change(nav={
    "maneuverType": "continue",
    "maneuverModifier": "slightLeft",
    "activeLaneDirection": "",
    "maneuverPrimaryText": "Keep left to stay on A1",
  })
  assert helper.lane_change_direction == LaneChangeDirection.left
  assert helper.desire == log.Desire.laneChangeLeft
  assert helper.auto_nav_active


def test_nav_lane_change_notification_keep_right():
  helper = start_nav_lane_change(nav={
    "maneuverType": "notification",
    "maneuverModifier": "right",
    "activeLaneDirection": "",
    "maneuverPrimaryText": "Keep right",
  })
  assert helper.lane_change_direction == LaneChangeDirection.right
  assert helper.desire == log.Desire.laneChangeRight
  assert helper.auto_nav_active


def test_nav_lane_change_fork_left_with_lane_guidance():
  helper = start_nav_lane_change(nav={
    "maneuverType": "fork",
    "maneuverModifier": "left",
    "activeLaneDirection": "slightLeft",
    "maneuverDistance": 280.0,
  })
  assert helper.lane_change_direction == LaneChangeDirection.left
  assert helper.auto_nav_active


def test_nav_lane_change_ignores_city_turn_and_roundabout():
  for maneuver_type, modifier in (("turn", "right"), ("roundabout", "left"), ("rotary", "right"), ("exit roundabout", "left")):
    helper = arm_nav(DesireHelper(), maneuverType=maneuver_type, maneuverModifier=modifier, activeLaneDirection="")
    step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) + 5)
    assert helper.lane_change_state == LaneChangeState.off, maneuver_type
    assert not helper.auto_nav_active


def test_nav_lane_change_ignores_too_close_and_too_far():
  helper = arm_nav(DesireHelper(), maneuverDistance=NAV_LANE_CHANGE_MIN_DISTANCE - 1.0)
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) + 5)
  assert helper.lane_change_state == LaneChangeState.off

  helper = arm_nav(DesireHelper(), maneuverDistance=500.0)
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) + 5, carstate=make_car_state(vEgo=30.0))
  assert helper.lane_change_state == LaneChangeState.off


def test_nav_lane_change_requires_highway_and_min_speed():
  helper = arm_nav(DesireHelper())
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) + 5, highway=False)
  assert helper.lane_change_state == LaneChangeState.off

  helper = arm_nav(DesireHelper())
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) + 5, carstate=make_car_state(vEgo=15.0))
  assert helper.lane_change_state == LaneChangeState.off


def test_nav_lane_change_blocked_by_bsm_edge_and_occupied_lane():
  helper = arm_nav(DesireHelper())
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) + 5, carstate=make_car_state(vEgo=30.0, rightBlindspot=True))
  assert helper.lane_change_state == LaneChangeState.off

  helper = arm_nav(DesireHelper())
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) + 5, right_edge_detected=True)
  assert helper.lane_change_state == LaneChangeState.off

  helper = arm_nav(DesireHelper())
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) + 5, right_lane_clear=False)
  assert helper.lane_change_state == LaneChangeState.off


def test_nav_lane_change_blocked_by_blinker_or_stale_instruction():
  helper = arm_nav(DesireHelper())
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) + 5, carstate=make_car_state(vEgo=30.0, leftBlinker=True))
  assert helper.lane_change_state == LaneChangeState.off or not helper.auto_nav_active

  helper = arm_nav(DesireHelper(), updatedAtMonotonic=monotonic() - 10.0)
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) + 5)
  assert helper.lane_change_state == LaneChangeState.off
  assert not helper.auto_nav_active


def test_nav_lane_change_straight_active_lane_does_not_trigger():
  helper = arm_nav(DesireHelper(), maneuverType="fork", maneuverModifier="left", activeLaneDirection="straight")
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) + 5)
  assert helper.lane_change_state == LaneChangeState.off


def test_finished_left_nav_lane_change_does_not_arm_overtake_return():
  helper = start_nav_lane_change(nav={
    "maneuverType": "continue",
    "maneuverModifier": "slightLeft",
    "activeLaneDirection": "",
  })
  assert helper.lane_change_state == LaneChangeState.laneChangeStarting
  saw_finishing = False
  for _ in range(_frames_for(2.5) + 5):
    if helper.lane_change_state == LaneChangeState.laneChangeFinishing:
      saw_finishing = True
    step_nav(helper, 1, lane_change_prob=0.0)
    if helper.lane_change_state == LaneChangeState.off and saw_finishing:
      break
  assert saw_finishing
  assert helper.lane_change_state == LaneChangeState.off
  assert not helper.return_pending
  assert not helper.auto_nav_active


def finish_nav_lane_change(helper, **kwargs):
  saw_finishing = False
  for _ in range(_frames_for(2.5) + 5):
    if helper.lane_change_state == LaneChangeState.laneChangeFinishing:
      saw_finishing = True
    step_nav(helper, 1, lane_change_prob=0.0, **kwargs)
    if helper.lane_change_state == LaneChangeState.off and saw_finishing:
      break
  assert saw_finishing
  assert helper.lane_change_state == LaneChangeState.off
  assert not helper.auto_nav_active
  return helper


def test_nav_lane_change_waits_to_settle_before_a_second_change():
  helper = start_nav_lane_change()
  finish_nav_lane_change(helper)
  assert helper.nav_lane_change_count == 1

  step_nav(helper, _frames_for(NAV_LANE_CHANGE_SETTLE) - 1)
  assert helper.lane_change_state == LaneChangeState.off
  assert not helper.auto_nav_active

  step_nav(helper, 1 + _frames_for(NAV_LANE_CHANGE_DELAY) + 1)
  assert helper.auto_nav_active
  assert helper.lane_change_direction == LaneChangeDirection.right
  assert helper.lane_change_state in (LaneChangeState.preLaneChange, LaneChangeState.laneChangeStarting)


def test_nav_lane_change_caps_at_two_changes_per_maneuver():
  helper = start_nav_lane_change()
  finish_nav_lane_change(helper)
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_SETTLE) + _frames_for(NAV_LANE_CHANGE_DELAY) + 2)
  assert helper.auto_nav_active
  finish_nav_lane_change(helper)
  assert helper.nav_lane_change_count == NAV_LANE_CHANGE_MAX_PER_MANEUVER

  step_nav(helper, _frames_for(NAV_LANE_CHANGE_SETTLE) + _frames_for(NAV_LANE_CHANGE_DELAY) + 5)
  assert helper.lane_change_state == LaneChangeState.off
  assert not helper.auto_nav_active
  assert helper.nav_lane_change_count == 2


def test_nav_lane_change_count_resets_on_new_maneuver():
  helper = start_nav_lane_change()
  finish_nav_lane_change(helper)
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_SETTLE) + _frames_for(NAV_LANE_CHANGE_DELAY) + 2)
  finish_nav_lane_change(helper)
  assert helper.nav_lane_change_count == 2

  arm_nav(helper, maneuverType="off ramp", maneuverModifier="right", activeLaneDirection="slightRight",
          maneuverDistance=300.0, maneuverPrimaryText="Exit 13")
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_DELAY) + 2)
  assert helper.auto_nav_active
  assert helper.nav_lane_change_count == 0 or helper.lane_change_state != LaneChangeState.off


def test_nav_lane_change_second_change_stops_on_road_edge_or_missing_lane():
  helper = start_nav_lane_change()
  finish_nav_lane_change(helper)

  step_nav(helper, _frames_for(NAV_LANE_CHANGE_SETTLE) + _frames_for(NAV_LANE_CHANGE_DELAY) + 5, right_edge_detected=True)
  assert helper.lane_change_state == LaneChangeState.off
  assert not helper.auto_nav_active

  helper = start_nav_lane_change()
  finish_nav_lane_change(helper)
  step_nav(helper, _frames_for(NAV_LANE_CHANGE_SETTLE) + _frames_for(NAV_LANE_CHANGE_DELAY) + 5, right_lane_clear=False)
  assert helper.lane_change_state == LaneChangeState.off
  assert not helper.auto_nav_active
