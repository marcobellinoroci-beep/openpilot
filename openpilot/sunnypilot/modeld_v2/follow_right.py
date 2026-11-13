"""
Copyright (c) 2026-, zoompilot contributors.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

FollowRightLine: keep the car at a fixed distance from the RIGHT reference only, ignoring the left line.

Reference: the ego-right lane line (laneLines[2]) when it is confident, else the right road edge (roadEdges[1]).
Target: car centre 1.8 m (half a lane) from the right line, or 1.45 m (half car width + 0.5 m) from the right edge.

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

REF_NONE, REF_LINE, REF_EDGE = 0, 1, 2


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
    self.offset = 0.0  # extra camera offset applied now (m)
    self.reference = REF_NONE
    self.active = False
    self.engage_timer = 0.0
    self.target = FirstOrderFilter(0.0, TARGET_RC, DT_MDL, initialized=False)

  def reset(self) -> None:
    self.active = False
    self.engage_timer = 0.0
    self.reference = REF_NONE
    self.target.initialized = False

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
             base_offset: float = 0.0) -> float:
    """Returns the extra camera offset (m) to add to CameraOffset. 0 when off or not applicable."""
    target = 0.0
    reference, dist, edge_dist = self.measure(modelv2) if self.enabled else (REF_NONE, None, None)
    conditions = (self.enabled and lat_active and reference != REF_NONE and
                  MIN_SPEED <= v_ego <= MAX_SPEED and
                  desire == log.Desire.none and lane_change_state == log.LaneChangeState.off and
                  not car_state.leftBlinker and not car_state.rightBlinker and not car_state.steeringPressed and
                  abs(curvature) * v_ego ** 2 < MAX_LAT_ACCEL)
    if conditions:
      self.engage_timer = min(self.engage_timer + DT_MDL, ENGAGE_TIME)
      self.reference = reference
      desired = (LINE_TARGET if reference == REF_LINE else EDGE_TARGET) - dist - base_offset
      low, high = -MAX_TOTAL_OFFSET - base_offset, MAX_TOTAL_OFFSET - base_offset
      if edge_dist is not None:  # never closer than EDGE_TARGET to the right edge
        low = max(low, EDGE_TARGET - edge_dist - base_offset)
      desired = float(np.clip(desired, low, high)) if low <= high else high
      smoothed = self.target.update(desired)
      self.active = self.engage_timer >= ENGAGE_TIME
      if self.active:
        target = float(np.clip(smoothed, low, high)) if low <= high else high
    else:
      self.reset()

    rate = RAMP_IN_RATE if self.active and abs(target) > abs(self.offset) and target * self.offset >= 0 else RAMP_OUT_RATE
    if not self.active:
      rate = RAMP_OUT_RATE
    step = rate * DT_MDL
    self.offset = float(np.clip(target, self.offset - step, self.offset + step))
    return self.offset
