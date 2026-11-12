from openpilot.cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = custom.ModelDataV2SP.TurnDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS  # 20 mph == 8.9408 m/s
LANE_CHANGE_TIME_MAX = 10.
LANE_CHANGE_START_TIME = 0.5
AUTO_OVERTAKE_DELAY = 2.0
AUTO_OVERTAKE_SPEED_MIN = 60 * CV.KPH_TO_MS  # 60 km/h == 16.67 m/s
# Settled in the passing lane, then give up if the right lane never opens.
AUTO_OVERTAKE_RETURN_SETTLE = 5.0  # s
AUTO_OVERTAKE_RETURN_TIMEOUT = 30.0  # s

# Lead qualification. x is meters ahead, v is absolute speed in m/s.
AUTO_OVERTAKE_LEAD_PROB = 0.5
AUTO_OVERTAKE_LEAD_MAX_X = 70.0  # m; ignore leads farther than this
# Slightly slower than ego. -1.0 m/s never fired once longitudinal control had
# matched the lead (v_rel ~ 0) or when the lead was only a little slower.
AUTO_OVERTAKE_LEAD_VREL = -0.3  # m/s; v_lead - v_ego must be below this
# Used only when the caller omits v_cruise_kph, so that path stays unchanged.
AUTO_OVERTAKE_LEAD_VREL_LEGACY = -1.0  # m/s
# carState.vCruise is kph. 255 is the unset sentinel; <= 0 is also unset.
AUTO_OVERTAKE_VCRUISE_UNSET_KPH = 255.0
AUTO_OVERTAKE_LEAD_BELOW_SET_SPEED = 1.0  # m/s; v_lead must be at least this far under the set speed
# One model frame is 0.05 s. A dropout shorter than this must not zero overtake_timer.
AUTO_OVERTAKE_LEAD_DROPOUT_HOLD = 0.5  # s

# Left adjacent lane. laneLineProbs[0] is the far-left line, [1] is the ego-left line.
# Both have to be confident or the lane is not treated as present.
LEFT_LANE_LINE_PROB_MIN = 0.5
# modelV2 has no adjacent-object list. leadsV3 is the only cheap occupancy signal:
# a confident lead whose current y lies between those two lines occupies the lane.
# Model y is positive to the right, so the left lane is the more-negative band.
# The inset keeps a vehicle riding the shared line from counting as in-lane.
LEFT_LANE_OCCUPANCY_MARGIN_M = 0.4  # m
LEFT_LANE_LEAD_PROB_MIN = 0.5

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}


def _cruise_set_speed_ms(v_cruise_kph):
  """Cruise set speed in m/s, or None when it is unset.

  carState.vCruise is kph. <= 0 or >= 255 (the unset sentinel) means no setpoint.
  """
  try:
    v_kph = float(v_cruise_kph)
  except (TypeError, ValueError):
    return None
  if v_kph <= 0.0 or v_kph >= AUTO_OVERTAKE_VCRUISE_UNSET_KPH:
    return None
  return v_kph * CV.KPH_TO_MS


def is_slower_lead(leads, v_ego, v_cruise_kph=None):
  """True if the current model lead is one auto-overtake should consider.

  Confident (prob > 0.5) and ahead. When v_cruise_kph is omitted the legacy
  predicate is kept (v_lead - v_ego < -1 m/s, no distance cap, no set-speed
  check) so existing callers do not change.

  When v_cruise_kph is passed (carState.vCruise, kph) the lead must also be
  within AUTO_OVERTAKE_LEAD_MAX_X and either slightly slower than ego
  (v_lead - v_ego < -0.3 m/s) or more than ~1 m/s below the cruise set speed.
  A matched lead (v_rel ~ 0) still counts in that second case. 255 or <= 0
  means the setpoint is unset and only the relative-speed check applies.
  """
  if not leads:
    return False
  lead = leads[0]
  try:
    v_lead = float(lead.v[0])
    x = float(lead.x[0])
    prob = float(lead.prob)
    v_ego = float(v_ego)
  except (IndexError, AttributeError, TypeError, ValueError):
    return False

  if not (prob > AUTO_OVERTAKE_LEAD_PROB and x > 0.0):
    return False

  v_rel = v_lead - v_ego
  if v_cruise_kph is None:
    return v_rel < AUTO_OVERTAKE_LEAD_VREL_LEGACY

  if x >= AUTO_OVERTAKE_LEAD_MAX_X:
    return False
  if v_rel < AUTO_OVERTAKE_LEAD_VREL:
    return True

  v_set = _cruise_set_speed_ms(v_cruise_kph)
  if v_set is None:
    return False
  return v_lead < (v_set - AUTO_OVERTAKE_LEAD_BELOW_SET_SPEED)


def is_left_lane_clear(model_v2):
  """True when the left adjacent lane exists and no model lead is sitting in it.

  Existence: far-left laneLineProbs[0] and ego-left laneLineProbs[1] both above
  LEFT_LANE_LINE_PROB_MIN, with the far line actually to the left of the near one.
  Occupancy: leadsV3 are time-offset lead hypotheses, not a dedicated object list,
  but a confident lead whose current (x, y) falls strictly inside that band is
  treated as occupying the lane. Missing lane data returns False.
  """
  try:
    probs = model_v2.laneLineProbs
    lines = model_v2.laneLines
    if len(probs) < 2 or len(lines) < 2:
      return False
    if not (probs[0] > LEFT_LANE_LINE_PROB_MIN and probs[1] > LEFT_LANE_LINE_PROB_MIN):
      return False
    y_far = float(lines[0].y[0])
    y_near = float(lines[1].y[0])
  except (IndexError, AttributeError, TypeError, ValueError):
    return False

  # Far-left line must sit to the left of the ego-left line.
  if not y_far < y_near:
    return False
  y_lo = y_far + LEFT_LANE_OCCUPANCY_MARGIN_M
  y_hi = y_near - LEFT_LANE_OCCUPANCY_MARGIN_M
  if y_lo >= y_hi:
    return False

  try:
    leads = model_v2.leadsV3
  except (AttributeError, TypeError):
    return True

  for lead in leads:
    try:
      if not (float(lead.prob) > LEFT_LANE_LEAD_PROB_MIN):
        continue
      x = float(lead.x[0])
      y = float(lead.y[0])
    except (IndexError, AttributeError, TypeError, ValueError):
      continue
    if x <= 0.0 or x >= AUTO_OVERTAKE_LEAD_MAX_X:
      continue
    if y_lo < y < y_hi:
      return False
  return True


def is_right_lane_clear(model_v2):
  """True when the right adjacent lane exists and no model lead is sitting in it.

  Mirror of is_left_lane_clear. laneLineProbs[2] is the ego-right line and [3] is
  the far-right line. Model y is positive to the right, so the right lane is the
  more-positive band. Same probability and occupancy thresholds as the left lane.
  Missing lane data returns False.
  """
  try:
    probs = model_v2.laneLineProbs
    lines = model_v2.laneLines
    if len(probs) < 4 or len(lines) < 4:
      return False
    if not (probs[2] > LEFT_LANE_LINE_PROB_MIN and probs[3] > LEFT_LANE_LINE_PROB_MIN):
      return False
    y_near = float(lines[2].y[0])
    y_far = float(lines[3].y[0])
  except (IndexError, AttributeError, TypeError, ValueError):
    return False

  # Far-right line must sit to the right of the ego-right line.
  if not y_near < y_far:
    return False
  y_lo = y_near + LEFT_LANE_OCCUPANCY_MARGIN_M
  y_hi = y_far - LEFT_LANE_OCCUPANCY_MARGIN_M
  if y_lo >= y_hi:
    return False

  try:
    leads = model_v2.leadsV3
  except (AttributeError, TypeError):
    return True

  for lead in leads:
    try:
      if not (float(lead.prob) > LEFT_LANE_LEAD_PROB_MIN):
        continue
      x = float(lead.x[0])
      y = float(lead.y[0])
    except (IndexError, AttributeError, TypeError, ValueError):
      continue
    if x <= 0.0 or x >= AUTO_OVERTAKE_LEAD_MAX_X:
      continue
    if y_lo < y < y_hi:
      return False
  return True


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.alc = AutoLaneChangeController(self)
    self.lane_turn_controller = LaneTurnController(self)
    self.lane_turn_direction = TurnDirection.none
    self.params = Params()
    self.auto_overtake = False
    self.auto_overtake_active = False
    self.overtake_timer = 0.0
    self.lead_dropout_timer = 0.0
    # One right-lane return owed after an auto-overtake finishes to the left.
    # Armed after left auto-overtake; starts only when the driver uses the right blinker.
    self.return_pending = False
    self.return_wait_timer = 0.0
    self.return_settle_timer = 0.0

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob, left_edge_detected=False, right_edge_detected=False, slower_lead_detected=False, left_lane_clear=True, right_lane_clear=True, right_lane_oncoming=False):
    self.alc.update_params()
    self.lane_turn_controller.update_params()
    self.auto_overtake = self.params.get_bool("AutoOvertake")
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # Lane turn controller update
    self.lane_turn_controller.update_lane_turn(blindspot_left=carstate.leftBlindspot, blindspot_right=carstate.rightBlindspot,
                                               left_blinker=carstate.leftBlinker, right_blinker=carstate.rightBlinker, v_ego=v_ego)
    self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()

    # --- Auto Overtake (left only, no blinker required) ---
    # Count only while the state machine is off. On a 2s hold, enter preLaneChange
    # left and let AutoLaneChangeController / the existing machine finish the change.
    # left_lane_clear defaults True so callers that do not compute it are unchanged.
    # A lead dropout shorter than AUTO_OVERTAKE_LEAD_DROPOUT_HOLD does not zero the timer.
    overtake_ready = (
      self.auto_overtake and
      lateral_active and
      not below_lane_change_speed and
      v_ego >= AUTO_OVERTAKE_SPEED_MIN and
      not carstate.leftBlinker and
      not carstate.rightBlinker and
      not carstate.leftBlindspot and
      not left_edge_detected and
      left_lane_clear and
      not self.return_pending and
      self.lane_change_state == LaneChangeState.off and
      self.alc.lane_change_set_timer != AutoLaneChangeMode.OFF
    )
    if overtake_ready and slower_lead_detected:
      self.overtake_timer += DT_MDL
      self.lead_dropout_timer = 0.0
    elif overtake_ready and self.overtake_timer > 0.0 and self.lead_dropout_timer < AUTO_OVERTAKE_LEAD_DROPOUT_HOLD:
      self.lead_dropout_timer += DT_MDL
      self.overtake_timer += DT_MDL
    else:
      self.overtake_timer = 0.0
      self.lead_dropout_timer = 0.0

    # One return to the right lane after the left auto-overtake finishes.
    # More prudent: does not start by itself. Stays pending until the driver
    # turns on the right blinker (and the safety checks still pass).
    # right_lane_oncoming defaults to clear. Tesla Model 3 HW4 has no adjacent-lane
    # object feed: radarTracks is empty (radarUnavailable; the Continental radar is
    # not on HW4), liveTracks is deprecated and unpublished, and leadsV3 is ego-path
    # hypotheses with unsigned norm speed, so it cannot see oncoming traffic.
    if self.return_pending:
      self.return_wait_timer += DT_MDL
      if self.lane_change_state == LaneChangeState.off:
        self.return_settle_timer += DT_MDL
      # Positive steering torque is a left input, opposite a return to the right.
      # Right blinker is the confirmation signal, so it does not cancel the pending return.
      opposite_torque = carstate.steeringPressed and carstate.steeringTorque > 0
      if (carstate.leftBlinker or opposite_torque or
          not self.auto_overtake or not lateral_active or
          self.return_wait_timer >= AUTO_OVERTAKE_RETURN_TIMEOUT):
        self.return_pending = False
        self.return_wait_timer = 0.0
        self.return_settle_timer = 0.0

    return_ready = (
      self.return_pending and
      self.return_settle_timer >= AUTO_OVERTAKE_RETURN_SETTLE and
      not slower_lead_detected and
      right_lane_clear and
      not right_lane_oncoming and
      not carstate.rightBlindspot and
      not right_edge_detected and
      carstate.rightBlinker and
      not carstate.leftBlinker and
      lateral_active and
      v_ego >= AUTO_OVERTAKE_SPEED_MIN and
      self.auto_overtake and
      self.lane_change_state == LaneChangeState.off and
      self.alc.lane_change_set_timer != AutoLaneChangeMode.OFF
    )

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX + self.alc.lane_change_time_extra or \
       self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self.auto_overtake_active = False
    else:
      auto_overtake_trigger = self.overtake_timer >= AUTO_OVERTAKE_DELAY

      if self.lane_change_state == LaneChangeState.off and \
         ((one_blinker and not self.prev_one_blinker) or auto_overtake_trigger or return_ready) and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        if return_ready:
          self.lane_change_direction = LaneChangeDirection.right
          self.auto_overtake_active = True
          self.return_pending = False
          self.return_wait_timer = 0.0
          self.return_settle_timer = 0.0
        elif auto_overtake_trigger:
          self.lane_change_direction = LaneChangeDirection.left
          self.auto_overtake_active = True
          self.overtake_timer = 0.0
        else:
          # Initialize lane change direction to prevent UI alert flicker
          self.lane_change_direction = self.get_lane_change_direction(carstate)
          self.auto_overtake_active = False

      elif self.lane_change_state == LaneChangeState.preLaneChange:
        if not self.auto_overtake or not self.auto_overtake_active:
          self.auto_overtake_active = False
          self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        edge_detected = ((left_edge_detected and self.lane_change_direction == LaneChangeDirection.left) or
                         (right_edge_detected and self.lane_change_direction == LaneChangeDirection.right))

        self.alc.update_lane_change(blindspot_detected, edge_detected, carstate.brakePressed, torque_applied)

        # Auto-overtake may remain in preLaneChange without a blinker.
        blinker_ok = one_blinker or self.auto_overtake_active

        if not blinker_ok or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self.lane_change_timer = 0.0
          self.auto_overtake_active = False
        elif (torque_applied or self.alc.auto_lane_change_allowed or self.auto_overtake_active) and not self.alc.lane_change_blocked:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_timer = 0.0

      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        self.lane_change_timer += DT_MDL

        if lane_change_prob < 0.02 and self.lane_change_timer >= LANE_CHANGE_START_TIME:
          # This tree finishes a lane change back to off (it does not enter laneChangeFinishing).
          finished_auto_left = (
            self.auto_overtake_active and
            self.lane_change_direction == LaneChangeDirection.left and
            not one_blinker
          )
          self.lane_change_timer = 0.0
          self.auto_overtake_active = False
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none
            if finished_auto_left:
              self.return_pending = True
              self.return_wait_timer = 0.0
              self.return_settle_timer = 0.0

    self.prev_one_blinker = one_blinker and lateral_active

    if self.lane_turn_direction != TurnDirection.none:
      self.desire = TURN_DESIRES[self.lane_turn_direction]
    else:
      self.desire = log.Desire.none
      if self.lane_change_state == LaneChangeState.laneChangeStarting:
        if self.lane_change_direction == LaneChangeDirection.left:
          self.desire = log.Desire.laneChangeLeft
        elif self.lane_change_direction == LaneChangeDirection.right:
          self.desire = log.Desire.laneChangeRight

    self.alc.update_state()
