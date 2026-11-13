"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import messaging, custom

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

GREEN_LIGHT_X_THRESHOLD = 30
LEAD_DEPART_DIST_THRESHOLD = 1.0
TRIGGER_TIMER_THRESHOLD = 0.3

# ConfirmStartFromStop: engaged with openpilot longitudinal, after a real stop the car waits for a touch of
# the gas; one chime when the model wants to go
START_STANDSTILL_SPEED = 0.1  # m/s
START_STANDSTILL_TIME = 1.0  # s stopped before the hold starts (not while crawling)
START_GO_TIME = 0.1  # s the model must keep asking to go before the chime (2 frames)
START_HOLD_MAX_SPEED = 0.5  # m/s: never hold a car that is already rolling
START_REARM_SPEED = 2.0  # m/s: moving this fast ends the episode

# RoundaboutAssist: engaged with openpilot longitudinal and a Mapbox route, a roundabout as the next maneuver within
# ROUNDABOUT_ZONE_DIST: one chime, a gentle speed cap, and a stop in the zone is held until the gas (the
# ConfirmStartFromStop hold). It does not look for traffic: the driver still gives way.
# Mapbox maneuver types for roundabouts (navd does not request roundabout_exits, so no 'exit roundabout' steps)
ROUNDABOUT_TYPES = ('roundabout', 'rotary', 'roundabout turn')
ROUNDABOUT_ZONE_DIST = 150.0  # m to the maneuver point (the entry)
ROUNDABOUT_NAV_TIMEOUT = 2.0  # s; navd publishes at 1 Hz, older than this is stale
ROUNDABOUT_V_CAP = 7.5  # m/s (27 km/h)
ROUNDABOUT_A_MIN = -1.0  # m/s^2: the cap never asks for harder braking than this
ROUNDABOUT_CAP_MARGIN = 10.0  # m before the maneuver point where the cap should be reached
ROUNDABOUT_CAP_GAIN = 0.5  # 1/s: below the cap, accelerate at most (cap - v) * gain
ROUNDABOUT_NEW_JUMP = 50.0  # m; the distance growing this much means a new maneuver


def roundabout_accel_cap(v_ego: float, dist: float) -> float:
  """Highest accel allowed toward a roundabout dist m ahead: the gentlest decel that reaches ROUNDABOUT_V_CAP
  ROUNDABOUT_CAP_MARGIN before it, never below ROUNDABOUT_A_MIN; under the cap, a gentle approach to it."""
  if v_ego <= ROUNDABOUT_V_CAP:
    return (ROUNDABOUT_V_CAP - v_ego) * ROUNDABOUT_CAP_GAIN
  d = max(dist - ROUNDABOUT_CAP_MARGIN, 1.0)
  return max((ROUNDABOUT_V_CAP ** 2 - v_ego ** 2) / (2.0 * d), ROUNDABOUT_A_MIN)


class E2EStates:
  INACTIVE = 0
  ARMED = 1
  CONSUMED = 2


class E2EAlertsHelper:
  def __init__(self):
    self._params = Params()
    self.frame = -1
    self.green_light_state = E2EStates.INACTIVE
    self.prev_green_light_state = E2EStates.INACTIVE
    self.lead_depart_state = E2EStates.INACTIVE
    self.prev_lead_depart_state = E2EStates.INACTIVE

    self.green_light_alert = False
    self.green_light_alert_enabled = self._params.get_bool("GreenLightAlert")
    self.lead_depart_alert = False
    self.lead_depart_alert_enabled = self._params.get_bool("LeadDepartAlert")

    self.green_light_trigger_timer = 0
    self.lead_depart_trigger_timer = 0
    self.last_lead_distance = -1
    self.last_moving_frame = -1

    self.allowed = False
    self.last_allowed = False
    self.has_lead = False

    self.lead_depart_arm_timer = 0
    self.lead_depart_confirmed_lead = False
    self.lead_depart_armed = False

    self.confirm_start_enabled = self._params.get_bool("ConfirmStartFromStop")
    self.openpilot_long = False  # set by the planner from CarParams
    self.start_chime = False
    self.start_hold = False  # the planner must keep asking to stop
    self.start_standstill_timer = 0.0
    self.start_armed = False  # a real stop happened this episode: hold until gas
    self.start_released = False  # the driver touched the gas this episode
    self.start_go_timer = 0.0
    self.start_chimed = False
    self.start_roundabout = False  # this hold was armed in a roundabout zone

    self.roundabout_enabled = self._params.get_bool("RoundaboutAssist")
    self.roundabout_chime = False
    self.roundabout_zone = False  # a roundabout within ROUNDABOUT_ZONE_DIST (arms the stop hold)
    self.roundabout_a_cap: float | None = None  # accel cap for the planner, None when not capping
    self.nav_age = float('inf')  # s since the last navInstruction
    self.nav_travel = 0.0  # m driven since the last navInstruction (navd is 1 Hz: dead-reckon in between)
    self.roundabout_key = None  # the roundabout maneuver being approached
    self.roundabout_last_dist = 0.0
    self.roundabout_chimed = False
    self.roundabout_released = False  # the driver pressed the gas for this roundabout

  def _read_params(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.green_light_alert_enabled = self._params.get_bool("GreenLightAlert")
      self.lead_depart_alert_enabled = self._params.get_bool("LeadDepartAlert")
      self.confirm_start_enabled = self._params.get_bool("ConfirmStartFromStop")
      self.roundabout_enabled = self._params.get_bool("RoundaboutAssist")

  def update_alert_trigger(self, sm: messaging.SubMaster):
    CS = sm['carState']
    CC = sm['carControl']

    model_x = sm['modelV2'].position.x
    max_idx = len(model_x) - 1
    self.has_lead = sm['radarState'].leadOne.present
    lead_dRel = sm['radarState'].leadOne.dRel

    standstill = CS.standstill
    moving = not standstill and CS.vEgo > 0.1

    if moving:
      self.last_moving_frame = self.frame
    recent_moving = self.last_moving_frame == -1 or (self.frame - self.last_moving_frame) * DT_MDL < 2.0

    self.allowed = not moving and not CS.gasPressed and not CC.enabled and not recent_moving

    # Green Light Alert
    green_light_trigger = False
    if self.green_light_state == E2EStates.ARMED:
      if model_x[max_idx] > GREEN_LIGHT_X_THRESHOLD:
        self.green_light_trigger_timer += 1
      else:
        self.green_light_trigger_timer = 0

      if self.green_light_trigger_timer * DT_MDL > TRIGGER_TIMER_THRESHOLD:
        green_light_trigger = True
    elif self.green_light_state != E2EStates.ARMED:
      self.green_light_trigger_timer = 0

    # Lead Departure Alert
    close_lead_valid = self.has_lead and lead_dRel < 8.0
    if self.allowed and not self.last_allowed and close_lead_valid:
      self.lead_depart_confirmed_lead = True
    elif not self.allowed:
      self.lead_depart_confirmed_lead = False

    if self.allowed and self.lead_depart_confirmed_lead and close_lead_valid:
      self.lead_depart_arm_timer += 1

      if self.lead_depart_arm_timer * DT_MDL >= 1.0:
        self.lead_depart_armed = True
    else:
      self.lead_depart_arm_timer = 0
      self.lead_depart_armed = False

    lead_depart_trigger = False
    if self.lead_depart_state == E2EStates.ARMED:
      if self.last_lead_distance == -1 or lead_dRel < self.last_lead_distance:
        self.last_lead_distance = lead_dRel

      if self.last_lead_distance != -1 and (lead_dRel - self.last_lead_distance > LEAD_DEPART_DIST_THRESHOLD):
        self.lead_depart_trigger_timer += 1
      else:
        self.lead_depart_trigger_timer = 0

      if self.lead_depart_trigger_timer * DT_MDL > TRIGGER_TIMER_THRESHOLD:
        lead_depart_trigger = True
    elif self.lead_depart_state != E2EStates.ARMED:
      self.last_lead_distance = -1
      self.lead_depart_trigger_timer = 0

    self.last_allowed = self.allowed

    return green_light_trigger, lead_depart_trigger

  def _reset_start(self) -> None:
    self.start_hold = False
    self.start_standstill_timer = self.start_go_timer = 0.0
    self.start_armed = self.start_released = self.start_chimed = self.start_roundabout = False

  def update_confirm_start(self, sm: messaging.SubMaster) -> bool:
    """ConfirmStartFromStop. Sets start_hold (the planner then adds a stop request) and returns True on the one
    frame the chime plays. Engaged with openpilot longitudinal, after >= 1 s stopped: hold until any gas press;
    chime once when the model asks to go. Off, disengaged, gas, or rolling: no hold."""
    CS = sm['carState']
    CC = sm['carControl']
    # CC.enabled stays on through a gas override (longActive drops), so a gas tap releases the episode instead
    # of starting a new one
    engaged = CC.enabled and self.openpilot_long
    # RoundaboutAssist arms the same hold in a roundabout zone, and keeps a hold it armed until the gas
    roundabout = self.roundabout_enabled and (self.roundabout_zone or self.start_roundabout)

    if not (self.confirm_start_enabled or roundabout) or not engaged:
      self._reset_start()
      return False

    if CS.vEgo > START_REARM_SPEED:
      self._reset_start()
      return False

    if CS.gasPressed:
      self.start_released = True  # the driver starts: release now, for the rest of this episode

    if CS.vEgo < START_STANDSTILL_SPEED:
      self.start_standstill_timer += DT_MDL
      if self.start_standstill_timer >= START_STANDSTILL_TIME and not self.start_released:
        self.start_armed = True
        self.start_roundabout = self.start_roundabout or self.roundabout_zone
    else:
      self.start_standstill_timer = 0.0

    self.start_hold = self.start_armed and not self.start_released and CS.vEgo < START_HOLD_MAX_SPEED

    chime = False
    if self.start_hold and not sm['modelV2'].action.shouldStop:
      self.start_go_timer += DT_MDL
      if self.start_go_timer >= START_GO_TIME and not self.start_chimed:
        self.start_chimed = chime = True
    else:
      self.start_go_timer = 0.0
    return chime

  def roundabout_ahead(self, sm: messaging.SubMaster) -> tuple[tuple[str, str, str], float] | None:
    """(maneuver key, distance) of a roundabout that is the next maneuver of a fresh, valid route, else None."""
    if self.nav_age > ROUNDABOUT_NAV_TIMEOUT or not sm.valid.get('navInstruction', False):
      return None
    nav = sm['navInstruction']
    m_type = str(nav.maneuverType).strip().lower()
    if m_type not in ROUNDABOUT_TYPES:
      return None
    return (m_type, str(nav.maneuverModifier), str(nav.maneuverPrimaryText)), float(nav.maneuverDistance)

  def update_roundabout(self, sm: messaging.SubMaster) -> bool:
    """RoundaboutAssist. Sets roundabout_zone and roundabout_a_cap; True on the one frame the chime plays."""
    CS = sm['carState']
    if sm.updated.get('navInstruction', False):
      self.nav_age = self.nav_travel = 0.0
    else:
      self.nav_age += DT_MDL
      self.nav_travel += max(CS.vEgo, 0.0) * DT_MDL
    self.roundabout_zone = False
    self.roundabout_a_cap = None

    ahead = self.roundabout_ahead(sm)
    if ahead is None:
      return False
    key, dist = ahead
    dist -= self.nav_travel
    if key != self.roundabout_key or dist > self.roundabout_last_dist + ROUNDABOUT_NEW_JUMP:
      self.roundabout_key = key  # a new roundabout: chime and cap again
      self.roundabout_chimed = self.roundabout_released = False
    self.roundabout_last_dist = dist

    engaged = sm['carControl'].enabled and self.openpilot_long
    if not self.roundabout_enabled or not engaged or not 0.0 <= dist <= ROUNDABOUT_ZONE_DIST:
      return False  # off, disengaged, too far, or passed

    self.roundabout_zone = True
    if CS.gasPressed:
      self.roundabout_released = True  # the driver takes over the speed for this roundabout
    if not self.roundabout_released:
      self.roundabout_a_cap = roundabout_accel_cap(CS.vEgo, dist)
    chime = not self.roundabout_chimed
    self.roundabout_chimed = True
    return chime

  @staticmethod
  def update_state_machine(state: int, enabled: bool, allowed: bool, triggered: bool) -> tuple[int, bool]:
    if state != E2EStates.INACTIVE:
      if not allowed or not enabled:
        state = E2EStates.INACTIVE

      else:
        if state == E2EStates.ARMED:
          if triggered:
            state = E2EStates.CONSUMED

        elif state == E2EStates.CONSUMED:
          pass

    elif state == E2EStates.INACTIVE:
      if allowed and enabled:
        state = E2EStates.ARMED

    return state, triggered

  def update(self, sm: messaging.SubMaster, events_sp: EventsSP) -> None:
    self._read_params()

    green_light_trigger, lead_depart_trigger = self.update_alert_trigger(sm)

    self.prev_green_light_state = self.green_light_state
    self.prev_lead_depart_state = self.lead_depart_state

    self.green_light_state, self.green_light_alert = self.update_state_machine(
      self.green_light_state,
      self.green_light_alert_enabled,
      self.allowed and not self.has_lead,
      green_light_trigger
    )

    self.lead_depart_state, self.lead_depart_alert = self.update_state_machine(
      self.lead_depart_state,
      self.lead_depart_alert_enabled,
      self.allowed and self.lead_depart_armed,
      lead_depart_trigger
    )

    self.roundabout_chime = self.update_roundabout(sm)
    self.start_chime = self.update_confirm_start(sm)

    if self.green_light_alert or self.lead_depart_alert or self.start_chime or self.roundabout_chime:
      events_sp.add(custom.OnroadEventSP.EventName.e2eChime)

    self.frame += 1
