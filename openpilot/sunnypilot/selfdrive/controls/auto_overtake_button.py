"""
Long-press detection to toggle the AutoOvertake param.

Button choice (see comments in AO_TOGGLE_BUTTON_NAMES): mainCruise. SET/RESUME
and gapAdjustCruise already have long-press actions in this codebase.
"""
from __future__ import annotations

AO_LONG_PRESS_S = 2.0

# Names only, resolved lazily so unit tests can inject their own types
# without importing opendbc.
AO_TOGGLE_BUTTON_NAMES = ("mainCruise",)


def resolve_ao_toggle_buttons():
  from opendbc.car.structs import car
  ButtonType = car.CarState.ButtonEvent.Type
  return tuple(getattr(ButtonType, name) for name in AO_TOGGLE_BUTTON_NAMES)


class AutoOvertakeLongPress:
  """Track press/release edges and fire once when a watched button is held ≥ 2 s."""

  def __init__(self, button_types=None, hold_s: float = AO_LONG_PRESS_S):
    self.button_types = button_types
    self.hold_s = hold_s
    self._pressed = False
    self._held_s = 0.0
    self._fired = False

  def _types(self):
    if self.button_types is None:
      self.button_types = resolve_ao_toggle_buttons()
    return self.button_types

  def update(self, button_events, dt: float) -> bool:
    """Return True exactly once per qualifying hold, when the timer first reaches hold_s."""
    types = self._types()
    for be in button_events or ():
      if be.type not in types:
        continue
      if be.pressed:
        self._pressed = True
        self._held_s = 0.0
        self._fired = False
      else:
        self._pressed = False
        self._held_s = 0.0
        self._fired = False

    if not self._pressed:
      return False

    self._held_s += dt
    if not self._fired and self._held_s >= self.hold_s:
      self._fired = True
      return True
    return False
