"""
Copyright (c) 2026-, zoompilot contributors.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

FollowRightLine: keep the car at a fixed distance from the RIGHT reference only, ignoring the left line.

Reference: the ego-right lane line (laneLines[2]) when it is confident, else the right road edge (roadEdges[1]).
Target: car centre 1.8 m (half a lane) from the right line, or 1.45 m (half car width + 0.5 m) from the right edge,
both moved by FollowRightMargin (cm, + = further from the right). Never closer than EDGE_HARD_MIN to the right edge.

Safety cancel: when the model is going around something (Blendful's flag, or its plan leaving the reference),
or a confident model lead is close ahead and not clearly to the left, the shift ramps out fast and stays off for
HOLD_TIME after the condition clears. openpilot does not reliably see oncoming traffic or cyclists outside the path.

It does not steer by itself: it adds an extra camera offset (CameraOffsetHelper.extra_offset), so the driving
model still plans the path and goes around obstacles. Geometry (CameraOffsetHelper shear): a camera offset o
moves every point the model sees by -o laterally, so with the model planning to be d_perc from the reference
in its own (shifted) view, the car really ends up d_perc + o from it. The offset that puts the car at D is
therefore D - d_perc - base_offset. y is positive to the right (laneLines[1] < laneLines[2]).
"""
import numpy as np

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL

LINE_TARGET = 1.8  # m, car centre to right lane line (half of a 3.6 m lane)
EDGE_TARGET = 1.45  # m, car centre to right road edge (half car width ~0.95 m + 0.5 m margin)
EDGE_HARD_MIN = 1.25  # m, car centre to right road edge, whatever the margin (half car width + 0.3 m)
MARGIN_RANGE_CM = (-30, 50)  # FollowRightMargin, cm added to both targets
MAX_TOTAL_OFFSET = 0.35  # m, CameraOffset range: base + extra stays within it

MIN_SPEED = 30 * CV.KPH_TO_MS
MAX_SPEED = 110 * CV.KPH_TO_MS
MAX_LAT_ACCEL = 1.5  # m/s^2

LINE_MIN_PROB = 0.6
LINE_MAX_STD = 0.3
EDGE_MAX_STD = 0.5
LINE_DIST_RANGE = (0.6, 4.0)  # m, plausible distance to the right line
EDGE_DIST_RANGE = (0.6, 6.0)  # m, plausible distance to the right edge
MEASURE_X = (6.0, 20.0)  # m ahead, where the reference is compared with the model's plan

ENGAGE_TIME = 1.0  # s every condition must hold before shifting
TARGET_RC = 1.0  # s, smoothing of the measured target
RAMP_IN_RATE = MAX_TOTAL_OFFSET / 3.0  # m/s: full shift in ~3 s
RAMP_OUT_RATE = MAX_TOTAL_OFFSET / 1.0  # m/s: back to 0 in ~1 s
FAST_OUT_RATE = MAX_TOTAL_OFFSET / 0.4  # m/s: safety cancel, back to 0 in ~0.4 s
HOLD_TIME = 3.0  # s off after a safety cancel clears

LEAD_MIN_PROB = 0.5
LEAD_MIN_DIST = 25.0  # m
LEAD_TIME = 2.0  # s: leads within max(LEAD_MIN_DIST, LEAD_TIME * v_ego) cancel
LEAD_LEFT_Y = -1.0  # m: a lead further left than this (y < -1 m) does not cancel
DEVIATION_X = (5.0, 40.0)  # m ahead where the plan is compared with the reference shape
DEVIATION_SPREAD = 0.5  # m: plan-to-reference gap varying more than this along DEVIATION_X = going around
DEVIATION_JUMP = 0.4  # m: gap moving this far from its slow average = going around
DEVIATION_RC = 5.0  # s

REF_NONE, REF_LINE, REF_EDGE = 0, 1, 2


def margin_from_param(value) -> float:
  """FollowRightMargin (cm) as metres, clamped to MARGIN_RANGE_CM."""
  try:
    cm = float(value)
  except (TypeError, ValueError):
    cm = 0.0
  if not np.isfinite(cm):
    cm = 0.0
  return float(np.clip(cm, *MARGIN_RANGE_CM)) / 100.0


def lead_close(leads, v_ego: float) -> bool:
  """A confident model lead within max(25 m, 2 s) that is not clearly to the left (or whose side is unknown)."""
  limit = max(LEAD_MIN_DIST, LEAD_TIME * v_ego)
  for lead in leads:
    if lead.prob <= LEAD_MIN_PROB or len(lead.x) == 0 or not 0.0 <= lead.x[0] <= limit:
      continue
    if len(lead.y) == 0 or not np.isfinite(lead.y[0]) or lead.y[0] > LEAD_LEFT_Y:
      return True
  return False


def plan_gap_spread(ref_x, ref_y, plan_x, plan_y) -> float:
  """How much the plan-to-reference gap varies along DEVIATION_X: ~0 when the plan follows the reference's shape."""
  if len(ref_x) < 2 or len(plan_x) < 2:
    return 0.0
  xs = np.linspace(DEVIATION_X[0], DEVIATION_X[1], 8)
  g = np.interp(xs, ref_x, ref_y) - np.interp(xs, plan_x, plan_y)
  return float(np.ptp(g)) if np.all(np.isfinite(g)) else 0.0


def _mean_dist(ref_x, ref_y, plan_x, plan_y) -> float | None:
  """Mean lateral distance from the model's planned position to the reference, MEASURE_X ahead."""
  if len(ref_x) < 2 or len(ref_y) < 2 or len(plan_x) < 2 or len(plan_y) < 2:
    return None
  xs = np.linspace(MEASURE_X[0], MEASURE_X[1], 8)
  d = np.interp(xs, ref_x, ref_y) - np.interp(xs, plan_x, plan_y)
  return float(np.mean(d)) if np.all(np.isfinite(d)) else None


class FollowRightLine:
  def __init__(self):
    self.enabled = False
    self.margin = 0.0  # m, FollowRightMargin
    self.offset = 0.0  # extra camera offset applied now (m)
    self.hazard = False
    self.hold_timer = 0.0
    self.gap_slow = FirstOrderFilter(0.0, DEVIATION_RC, DT_MDL, initialized=False)
    self._slow_reference = REF_NONE
    self.reference = REF_NONE
    self.active = False
    self.engage_timer = 0.0
    self.target = FirstOrderFilter(0.0, TARGET_RC, DT_MDL, initialized=False)

  def reset(self) -> None:
    self.active = False
    self.engage_timer = 0.0
    self.reference = REF_NONE
    self.target.initialized = False

  def targets(self) -> tuple[float, float]:
    """(line target, edge target) for the car centre, margin applied, edge never below EDGE_HARD_MIN."""
    return LINE_TARGET + self.margin, max(EDGE_TARGET + self.margin, EDGE_HARD_MIN)

  def deviating(self, modelv2, reference: int, dist: float | None) -> bool:
    """The model's plan is leaving the reference: going around something."""
    if reference == REF_NONE or dist is None:
      self.gap_slow.initialized = False
      self._slow_reference = REF_NONE
      return False
    ref = modelv2.laneLines[2] if reference == REF_LINE else modelv2.roadEdges[1]
    spread = plan_gap_spread(list(ref.x), list(ref.y), list(modelv2.position.x), list(modelv2.position.y))
    if reference != self._slow_reference:
      self.gap_slow.initialized = False
      self._slow_reference = reference
    jump = abs(dist - self.gap_slow.x) if self.gap_slow.initialized else 0.0
    deviating = spread > DEVIATION_SPREAD or jump > DEVIATION_JUMP
    if not deviating:
      self.gap_slow.update(dist)
    return deviating

  def measure(self, modelv2) -> tuple[int, float | None, float | None]:
    """(reference, distance to it, distance to the right edge if confident) in the model's view."""
    plan_x, plan_y = list(modelv2.position.x), list(modelv2.position.y)
    edge_d = None
    if len(modelv2.roadEdges) > 1 and len(modelv2.roadEdgeStds) > 1 and modelv2.roadEdgeStds[1] < EDGE_MAX_STD:
      d = _mean_dist(list(modelv2.roadEdges[1].x), list(modelv2.roadEdges[1].y), plan_x, plan_y)
      if d is not None and EDGE_DIST_RANGE[0] <= d <= EDGE_DIST_RANGE[1]:
        edge_d = d
    if len(modelv2.laneLines) > 2 and len(modelv2.laneLineProbs) > 2 and len(modelv2.laneLineStds) > 2 and \
       modelv2.laneLineProbs[2] > LINE_MIN_PROB and modelv2.laneLineStds[2] < LINE_MAX_STD:
      d = _mean_dist(list(modelv2.laneLines[2].x), list(modelv2.laneLines[2].y), plan_x, plan_y)
      if d is not None and LINE_DIST_RANGE[0] <= d <= LINE_DIST_RANGE[1]:
        return REF_LINE, d, edge_d
    if edge_d is not None:
      return REF_EDGE, edge_d, edge_d
    return REF_NONE, None, None

  def update(self, modelv2, car_state, lat_active: bool, desire, lane_change_state, v_ego: float, curvature: float,
             base_offset: float = 0.0, going_around: bool = False) -> float:
    """Returns the extra camera offset (m) to add to CameraOffset. 0 when off or not applicable."""
    if not self.enabled:
      self.reset()
      self.hazard, self.hold_timer = False, 0.0
      self.gap_slow.initialized = False
      self.offset = float(np.clip(0.0, self.offset - RAMP_OUT_RATE * DT_MDL, self.offset + RAMP_OUT_RATE * DT_MDL))
      return self.offset

    target = 0.0
    reference, dist, edge_dist = self.measure(modelv2)
    line_target, edge_target = self.targets()

    # safety cancel: fast out, then HOLD_TIME off after it clears
    self.hazard = bool(going_around or lead_close(modelv2.leadsV3, v_ego) or self.deviating(modelv2, reference, dist))
    if self.hazard:
      self.hold_timer = HOLD_TIME
    else:
      self.hold_timer = max(self.hold_timer - DT_MDL, 0.0)

    conditions = (lat_active and reference != REF_NONE and
                  MIN_SPEED <= v_ego <= MAX_SPEED and
                  desire == log.Desire.none and lane_change_state == log.LaneChangeState.off and
                  not car_state.leftBlinker and not car_state.rightBlinker and not car_state.steeringPressed and
                  abs(curvature) * v_ego ** 2 < MAX_LAT_ACCEL)
    if conditions:
      self.engage_timer = min(self.engage_timer + DT_MDL, ENGAGE_TIME)
      self.reference = reference
      desired = (line_target if reference == REF_LINE else edge_target) - dist - base_offset
      low, high = -MAX_TOTAL_OFFSET - base_offset, MAX_TOTAL_OFFSET - base_offset
      if edge_dist is not None:  # never closer than the edge target (>= EDGE_HARD_MIN) to the right edge
        low = max(low, edge_target - edge_dist - base_offset)
      desired = float(np.clip(desired, low, high)) if low <= high else high
      smoothed = self.target.update(desired)
      self.active = self.engage_timer >= ENGAGE_TIME and self.hold_timer <= 0.0
      if self.active:
        target = float(np.clip(smoothed, low, high)) if low <= high else high
    else:
      self.reset()

    if self.hold_timer > 0.0:
      rate = FAST_OUT_RATE
    elif self.active and abs(target) > abs(self.offset) and target * self.offset >= 0:
      rate = RAMP_IN_RATE
    else:
      rate = RAMP_OUT_RATE
    step = rate * DT_MDL
    self.offset = float(np.clip(target, self.offset - step, self.offset + step))
    return self.offset
