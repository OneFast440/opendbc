"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ford lateral/longitudinal values, ported from BluePilot bp-7.0.

Ford's PSCM does not take a steering command. It takes a third-order polynomial describing the
road centerline and runs its own lateral planner on it:

    y(x) = c0 + c1*x + 1/2*c2*x^2 + 1/6*c3*x^3

  c0  path_offset     lateral offset from the centerline
  c1  path_angle      heading angle to the centerline
  c2  curvature       curvature of the centerline
  c3  curvature_rate  rate of change of that curvature

Upstream openpilot drives c2 alone. BluePilot offers two strategies instead:

  curvature  all four signals, with model-blended curvature, a derived curvature rate and a PID
             lane-centering term on c1 (lateral_curv_ext.py)
  angle      c1 alone, because the PSCM low-pass filters c2 for up to a second and keeps acting
             on a stale command (lateral_angle_ext.py)

Background: https://bluepilot.dev/announcements/
"""

from collections import namedtuple
from enum import IntEnum

from opendbc.car import structs
from opendbc.car.ford.values import CAR
from opendbc.car.lateral import AngleSteeringLimits

ButtonType = structs.CarState.ButtonEvent.Type
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])


class PrimaryLateralControl(IntEnum):
  """Which lateral strategy runs. Selected by the FordPrefLateralControl param and carried on
  CP_SP.fordLateralTuning.primaryControl. Read once at car init, because the panda safety mode
  is derived from the same read and the two layers must never disagree.

  Defaults to stock, so updating does not change how anyone's car drives until they opt in.
  """
  stock = 0      # upstream openpilot: curvature only, every other signal at its sentinel
  curvature = 1  # BluePilot: all four signals, curvature-primary
  angle = 2      # BluePilot: path_angle-primary


class FordSafetyFlagsSP:
  """sunnypilot-level safety parameters for Ford.

  Carried in CP_SP.safetyParam and delivered to the safety firmware as current_safety_param_sp
  (the separate SP uint16, USB control 0xdf) -- NOT safetyConfigs[].safetyParam. ford_init reads
  it with GET_FLAG/masking, same pattern as Subaru STOP_AND_GO. Plain int constants, not IntFlag:
  CP_SP.safetyParam must stay a plain int through capnp serialization.

  Bits 0-1 carry the PrimaryLateralControl value, so the panda knows which signal is the actuator
  and which signals must stay at their inactive sentinels.
  """
  LATERAL_MODE_MASK = 0x3


# Ford cruise control buttons live in Steering_Data_FD1 (CAN id 131) as 1-bit flags.
# Some are combo buttons that emit two ButtonEvent types; carstate_ext picks which one to emit
# based on whether cruise is currently engaged.
BUTTONS = [
  Button(ButtonType.accelCruise, "Steering_Data_FD1", "CcAslButtnSetIncPress", [1]),
  Button(ButtonType.setCruise, "Steering_Data_FD1", "CcAslButtnSetIncPress", [1]),
  Button(ButtonType.decelCruise, "Steering_Data_FD1", "CcAslButtnSetDecPress", [1]),
  Button(ButtonType.setCruise, "Steering_Data_FD1", "CcAslButtnSetDecPress", [1]),
  Button(ButtonType.cancel, "Steering_Data_FD1", "CcAslButtnCnclResPress", [1]),
  Button(ButtonType.resumeCruise, "Steering_Data_FD1", "CcAslButtnCnclResPress", [1]),
  Button(ButtonType.mainCruise, "Steering_Data_FD1", "CcButtnOnOffPress", [1]),
]


# Model time index breakpoints, matching selfdrive/modeld/constants.py ModelConstants.T_IDXS.
# Duplicated here because opendbc must not import openpilot; CarControlSP.FordLateral's model
# arrays are sampled on exactly this grid.
T_IDXS = [
  0.0, 0.009765625, 0.0390625, 0.087890625, 0.15625, 0.244140625, 0.3515625, 0.478515625,
  0.625, 0.791015625, 0.9765625, 1.181640625, 1.40625, 1.650390625, 1.9140625, 2.197265625,
  2.5, 2.822265625, 3.1640625, 3.525390625, 3.90625, 4.306640625, 4.7265625, 5.166015625,
  5.625, 6.103515625, 6.6015625, 7.119140625, 7.65625, 8.212890625, 8.7890625, 9.384765625, 10.0,
]

# DBC LatCtlCurv_No_Actl magnitude limit (1/m). Also the panda's FORD_STEERING_LIMITS.max_curvature.
CURVATURE_MAX = 0.02
# DBC LatCtlCurv_NoRate_Actl / LatCtlCrv_NoRate2_Actl magnitude limit (1/m^2).
CURVATURE_RATE_MAX = 0.001023
# DBC LatCtlPath_An_Actl range (rad). The panda mirror lives in safety/modes/ford.h; the PSCM
# enforces the same range in firmware.
FORD_DBC_PATH_ANGLE_MIN = -0.5
FORD_DBC_PATH_ANGLE_MAX = 0.5235
# Curvature mode only trims lane position with c1, so it keeps a much tighter cap than the DBC's.
# Mirrored by the panda, which allows the full DBC range only in angle mode.
CURV_MODE_PATH_ANGLE_MAX = 0.25

# Curvature rate limits for the curvature strategy. Three breakpoints rather than upstream's two:
# higher rates at low speed for responsiveness, lower at mid-speed for comfort, very low at
# highway speed for stability. The control side uses a stricter wind-up table than unwind so
# openpilot stays inside the panda's symmetric limits.
_BP_ANGLE_RATE_UP = ([5., 16., 25.], [0.0025, 0.0012, 0.00008])
_BP_ANGLE_RATE_DOWN = ([5., 16., 25.], [0.0025, 0.0014, 0.00018])
BP_ANGLE_LIMITS = AngleSteeringLimits(CURVATURE_MAX, _BP_ANGLE_RATE_UP, _BP_ANGLE_RATE_DOWN)

# User-tunable values. (default, min, max) -- the single source of truth for the defaults and
# clamps used by the settings UI, the sunnylink schema, and the control code.
# angle mode
LOW_SPEED_FACTOR_RANGE = (1.0, 0.5, 1.5)
HIGH_SPEED_FACTOR_RANGE = (1.0, 0.5, 1.5)
HIGH_SPEED_DAMPENING_RANGE = (1.0, 0.25, 1.25)
LANE_CHANGE_FACTOR_RANGE = (1.0, 0.85, 1.5)
# curvature mode
LANE_CHANGE_FACTOR_CURV_RANGE = (0.85, 0.5, 1.0)
BLEND_RATIO_RANGE = (0.4, 0.0, 1.0)
PATH_OFFSET_RANGE = (0.0, -1.0, 1.0)
LANE_POSITIONING_GAIN_RANGE = (3.0, 0.0, 20.0)

# Hard-coded per-platform gain defaults for angle mode: (low-curvature gain, high-curvature gain),
# both applied only at high speed. The PSCM compensates path_angle against a factory model of the
# vehicle (trim, suspension, weight distribution), so the same commanded angle produces different
# steering across platforms -- and across trims within a platform, which is what the two user
# factors above are for.
_GAIN_CAN = (1.00, 1.15)         # CAN vehicles (Escape Mk4, Bronco Sport, Explorer, Maverick, Edge)
_GAIN_CANFD_BOF = (0.95, 0.95)   # CAN FD body-on-frame trucks (F-150, Lightning, Expedition, Ranger)
_GAIN_CANFD_SUV = (1.00, 1.05)   # CAN FD unibody SUVs (Mustang Mach-E, Escape Mk4.5)

_CANFD_BOF_CARS = frozenset({
  CAR.FORD_F_150_MK14,
  CAR.FORD_F_150_LIGHTNING_MK1,
  CAR.FORD_EXPEDITION_MK4,
  CAR.FORD_RANGER_MK2,
})
_CANFD_SUV_CARS = frozenset({
  CAR.FORD_MUSTANG_MACH_E_MK1,
  CAR.FORD_ESCAPE_MK4_5,
})


def platform_path_angle_gains(car_fingerprint) -> tuple[float, float]:
  """(low-curvature gain, high-curvature gain) defaults for a platform."""
  if car_fingerprint in _CANFD_BOF_CARS:
    return _GAIN_CANFD_BOF
  if car_fingerprint in _CANFD_SUV_CARS:
    return _GAIN_CANFD_SUV
  return _GAIN_CAN
