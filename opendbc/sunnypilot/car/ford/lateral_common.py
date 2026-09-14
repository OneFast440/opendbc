"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Shared pieces of the two BluePilot Ford lateral strategies.
"""
from collections import namedtuple

# The four polynomial signals plus the two mode bits, for one lateral frame. Both strategies
# return this and the car controller packs it identically, so the only difference on the wire is
# which signals are non-zero.
FordLateralResult = namedtuple('FordLateralResult', [
  'apply_curvature',   # c2
  'curvature_rate',    # c3
  'path_offset',       # c0
  'path_angle',        # c1
  'ramp_type',
  'precision_type',
  'lat_inactive',      # True -> car controller must send mode 0 for this frame
])

# Mode 0 with every signal at its inactive sentinel. Used by both strategies whenever lateral
# hands back to the driver: on disengage, during a human-turn override, and during angle mode's
# stall blip. Every check in safety/modes/ford.h has a legitimate !steer_control_enabled branch,
# so these frames need no safety bypass of any kind.
INACTIVE_RESULT = FordLateralResult(
  apply_curvature=0.0,
  curvature_rate=0.0,
  path_offset=0.0,
  path_angle=0.0,
  ramp_type=0,
  precision_type=1,
  lat_inactive=True,
)


def get_current_curvature(CS) -> float:
  """Measured curvature of the car right now, in openpilot's sign convention.

  Derived from the RCM yaw rate, the same source safety/modes/ford.h builds its measured
  curvature from. Any value judged against that check has to come from the same measurement as
  the check's own reference, so every consumer reads it through here.
  """
  return -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
