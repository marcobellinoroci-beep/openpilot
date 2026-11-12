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
AUTO_OVERTAKE_LEAD_VREL = -1.0  # m/s; lead must be at least this much slower than ego
AUTO_OVERTAKE_LEAD_PROB = 0.5

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}


def is_slower_lead(leads, v_ego):
  """True if the current model lead is ahead, confident, and slower than ego."""
  if not leads:
    return False
  lead = leads[0]
  try:
    v_rel = lead.v[0] - v_ego
    return lead.prob > AUTO_OVERTAKE_LEAD_PROB and lead.x[0] > 0.0 and v_rel < AUTO_OVERTAKE_LEAD_VREL
  except (IndexError, AttributeError, TypeError):
    return False


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

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob, left_edge_detected=False, right_edge_detected=False, slower_lead_detected=False):
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
    overtake_conditions = (
      self.auto_overtake and
      lateral_active and
      not below_lane_change_speed and
      not carstate.leftBlinker and
      not carstate.rightBlinker and
      not carstate.leftBlindspot and
      not left_edge_detected and
      self.lane_change_state == LaneChangeState.off and
      self.alc.lane_change_set_timer != AutoLaneChangeMode.OFF and
      slower_lead_detected
    )
    if overtake_conditions:
      self.overtake_timer += DT_MDL
    else:
      self.overtake_timer = 0.0

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX + self.alc.lane_change_time_extra or \
       self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self.auto_overtake_active = False
    else:
      auto_overtake_trigger = self.overtake_timer >= AUTO_OVERTAKE_DELAY

      if self.lane_change_state == LaneChangeState.off and \
         ((one_blinker and not self.prev_one_blinker) or auto_overtake_trigger) and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_timer = 0.0
        if auto_overtake_trigger:
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
          self.lane_change_timer = 0.0
          self.auto_overtake_active = False
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_direction = self.get_lane_change_direction(carstate)
          else:
            self.lane_change_state = LaneChangeState.off
            self.lane_change_direction = LaneChangeDirection.none

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
