"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Manual-steering-override ("human turn") detection for Ford angle control, ported from
BluePilot bp-7.0.

While the driver holds a real turn, angle control forces lateral inactive (mode 0, all signals
zero on the wire) rather than winding path_angle into a stale command the PSCM has to reconcile
on release -- on the Mach-E's PSCM that reconciliation cost 2-3 s of dead time before control
resumed. Mode 0 needs no safety bypass: every check in safety/modes/ford.h has a legitimate
!steer_control_enabled branch.
"""
from opendbc.car import DT_CTRL
from opendbc.car.ford.values import CarControllerParams

# Require sustained hands-on AND a large angle (avoids resetting on small wheel nudges in a curve).
HUMAN_TURN_ANGLE_DEG = 45.0
HUMAN_TURN_HOLD_S = 1.5
# When the wheel was ALREADY past HUMAN_TURN_ANGLE_DEG at first contact -- lateral control had it
# turned mid-curve -- the angle condition is pre-satisfied, so a brief corrective nudge would latch
# after only HUMAN_TURN_HOLD_S of light contact and kill steering mid-curve. Require a longer hold
# there before reading it as an intentional takeover.
HUMAN_TURN_HOLD_PRETURNED_S = 3.0

_STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL  # 20 Hz lateral tick


class HumanTurnDetector:
  """Latches ``active`` once the driver holds real steering pressure AND ``|wheel angle|`` >
  ``HUMAN_TURN_ANGLE_DEG`` continuously for the hold time -- long enough to tell an intentional
  turn from a brief nudge.

  Call ``update`` once per lateral tick while control is active; call ``reset`` on the inactive
  path to zero the timer.
  """

  def __init__(self):
    self.hold_timer_s = 0.0
    self.active = False
    self._active_last = False
    self._pressed_last = False
    self._press_started_preturned = False

  def update(self, enabled: bool, steering_pressed: bool, steering_angle_deg: float) -> bool:
    self._active_last = self.active
    # Was the wheel already past the angle threshold when this press began? If so the driver is
    # touching a wheel that lateral control turned (mid-curve nudge), not driving a turn -- hold
    # the longer HUMAN_TURN_HOLD_PRETURNED_S before latching.
    if steering_pressed and not self._pressed_last:
      self._press_started_preturned = abs(steering_angle_deg) > HUMAN_TURN_ANGLE_DEG
    self._pressed_last = steering_pressed

    if not enabled:
      self.hold_timer_s = 0.0
    elif steering_pressed and abs(steering_angle_deg) > HUMAN_TURN_ANGLE_DEG:
      self.hold_timer_s += _STEER_DT
    else:
      self.hold_timer_s = 0.0

    hold_req = HUMAN_TURN_HOLD_PRETURNED_S if self._press_started_preturned else HUMAN_TURN_HOLD_S
    self.active = self.hold_timer_s >= hold_req
    return self.active

  @property
  def just_released(self) -> bool:
    return self._active_last and not self.active

  def reset(self) -> None:
    self.hold_timer_s = 0.0
    self.active = False
    self._active_last = False
    self._pressed_last = False
    self._press_started_preturned = False
