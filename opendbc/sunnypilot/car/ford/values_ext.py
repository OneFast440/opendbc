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

# *** PSCM firmware ground truth ***
#
# Read out of the PSCM firmware ML3V-14D003-BC (EXE) and its calibration PL3V-14D004-AA (DATA),
# cal base VBF 0x101C0470, reached at runtime as control_block(_DAT_febe4238) -> +0x98 -> cal.
# Exec addresses are relative to base 0x00040000.
#
# Provenance: this is the calibration off a 2023 F-150 Raptor R, dumped from that vehicle's own
# module. It is not a reference platform or a related trim. Note that such a truck fingerprints as
# CAR.FORD_F_150_MK14, which is a broad capture covering every F-150 trim, so the fingerprint says
# nothing about which PSCM calibration is actually in the vehicle: a base F-150, a Lightning and a
# Raptor R all land on the same platform enum. Treat these values as confirmed for a Raptor R and
# unverified for anything else on that enum.
#
# Everything here is informational: it documents what the actuator does so the control constants
# below can be reasoned about instead of purely road-fitted. Nothing in this block clamps a
# command, and no control constant is derived from one. The internal units of the cal values were
# never pinned to physical units, so they describe envelope *shape and ordering*, not calibrated
# limits, and that is true whether or not the vehicle matches. Pinning the units needs a sweep on
# the truck, not more decompilation.
#
# Control loop
PSCM_INTERNAL_RATE_HZ = 250.0     # FUN_0005dfb2, dt = 0.004. openpilot transmits at 20 Hz.
# Output saturation, mode-independent fixed literals in FUN_0005e2a6. The primary command int16 at
# gp-0xe474 clamps to +-1440; +-8.8 is cal +0x00; a secondary angle path (FUN_0005f13e) clamps to
# +-8.0. These are where the actuator stops responding to a larger command, and they bite well
# before the fault thresholds below.
PSCM_OUTPUT_SAT_PRIMARY = 1440
PSCM_OUTPUT_SAT_CAL = 8.8
PSCM_OUTPUT_SAT_SECONDARY = 10.0
PSCM_OUTPUT_SAT_ANGLE_PATH = 8.0
# Slew limiter, FUN_0005dfb2: rate = (1 / cal) * 0.004 per 250 Hz tick. cal +0x40 = 1.8 winding up,
# cal +0x44 = 1.3 unwinding. Which one applies is gated by an internal mode/direction state
# (DAT_febed36b, written by the mode arbiter FUN_0005df34), NOT by the ramp_type we transmit.
PSCM_SLEW_CAL_UP = 1.8
PSCM_SLEW_CAL_DOWN = 1.3
PSCM_SLEW_PER_S_UP = (1.0 / PSCM_SLEW_CAL_UP) * 0.004 * PSCM_INTERNAL_RATE_HZ      # 0.5556 /s
PSCM_SLEW_PER_S_DOWN = (1.0 / PSCM_SLEW_CAL_DOWN) * 0.004 * PSCM_INTERNAL_RATE_HZ  # 0.7692 /s
# The one structural fact that survives the unit gap: the module unwinds 38% faster than it winds
# up. Any rate limit on our side that is tighter going down than coming up is fighting the
# hardware's own asymmetry, not matching it.
PSCM_SLEW_DOWN_UP_RATIO = PSCM_SLEW_PER_S_DOWN / PSCM_SLEW_PER_S_UP                # 1.385
# Deadband, FUN_0005d7ca: fixed cal +0x2c / +0x30. Not gated on the precision_type we transmit.
PSCM_DEADBAND_CAL = (110.0, 90.0)
# Fault thresholds. Exceeding any trips a Dem fault and drops assist, so this is the outer
# envelope, not a working limit. FUN_0005e042 (cal +0x08, two inputs), FUN_0005e0c4 (cal +0x10),
# FUN_0005db6c (cal +0x20), FUN_0005dd1a (cal +0x1c).
PSCM_FAULT_THRESHOLDS = (0.73, 7.0, 19.0, 20.0)
# FUN_00093b2a forms the angle domain as curvature * speed * 572.95776e-6, and
# 572.95776 = 10 * (180/pi). It scales with speed, which a steering-wheel angle would not
# (delta ~= L*kappa), so path_angle is a path heading swept over a lookahead (kappa*v*t). This is
# the firmware's own confirmation that `path_angle = kappa * v_ego * gain` in lateral_angle_ext is
# the right shape: the gain absorbs the lookahead time and the per-platform compensation.
PSCM_ANGLE_FROM_KAPPA_V = 572.95776e-6
# ramp_type and precision_type do not retune the actuator in Limited mode: every consumer we read
# uses fixed cal (slew above, deadband above), and the 0-3 enum that looked like a received
# ramp_type (0xFEBE4916) is an outbound status the PSCM transmits, packed with a rolling counter
# and checksum by FUN_000b1c96. Limited vs Extended is selected internally by FUN_000a7204 from
# two RTE mode-condition flags, not from the LatCtl_D2_Rq we send, so Extended is not reachable
# from the wire. We keep sending the same values we always have; nothing should be tuned on them.
PSCM_RAMP_PRECISION_INERT_IN_LIMITED = True
# The PSCM broadcasts LatCtlLim_D_Stat on Lane_Assist_Data3_FD1 (0x3CC), but on CAN FD in angle
# mode it does not fire even while the module is attenuating delivery. That is why saturation has
# to be observed from delivered-vs-commanded motion rather than read off the bus.
PSCM_LIM_STAT_UNRELIABLE_CANFD = True

# User-tunable values. (default, min, max) -- the single source of truth for the defaults and
# clamps used by the settings UI, the sunnylink schema, and the control code.
# angle mode
# Accelerator pedal position, percent, above which the driver counts as overriding.
#
# CarState sets gasPressed from ApedPos_Pc_ActlArb > 1e-6, so the lightest imaginable touch
# reads as a full override and the whole ACCDATA goes inactive. If the driver is asking for
# less than cruise already was, the truck then slows down under a pedal they just put their
# foot on, which is the opposite of what touching the accelerator is for.
#
# Measured over six of the owner's drives, a deliberate press sits at 7-31% with a 16% median
# and only 0.1% of pressed frames below 2%, so a threshold in this range costs nothing on a
# real press and only filters the feather.
#
# It is a tuning value rather than a constant because how far it can be raised is a question
# about the PCM, not about the pedal. Staying active at a percent or two of pedal is the same
# thing as the normal cruising state. Staying active at ten is openpilot commanding propulsion
# while the driver is also asking for some, which is the arrangement that was denied when
# engaging with the pedal already down. Whether it is also denied once already engaged is
# untested, and raising this is how to find out.
PEDAL_OVERRIDE_RANGE = (2.0, 0.0, 25.0)   # default, min, max

# What the driver is asking for, in m/s^2, from accelerator pedal position and speed.
#
# The override is meant to hand over when the driver asks for more than openpilot already
# is, and that comparison needs both sides in the same units. Pedal position is not an
# acceleration, so it is measured: grade corrected, brakes off, above 7 mph, over eleven of
# the owner's drives, 29718 samples. Medians per cell, which are monotonic in pedal and fall
# with speed as the same pedal buys less acceleration.
#
# Only used to decide who is in control, never to command anything, so being a little off
# costs a handover at a slightly wrong pedal position and nothing more. It errs toward
# handing over: the rows read a given pedal as asking for slightly more than the median cell
# did, so a driver on the edge gets control rather than openpilot keeping it.
PEDAL_ACCEL_BP = (0.0, 3.0, 5.0, 7.5, 11.0, 15.5, 21.5, 32.0)        # percent
PEDAL_ACCEL_SPEED_BP = (7.2, 15.6, 24.6)                             # m/s, the band centres
PEDAL_ACCEL_V = (
  (-0.21, -0.14, -0.02, 0.22, 0.48, 0.93, 1.31, 2.17),               # ~16 mph
  (-0.27, -0.14, -0.06, -0.06, 0.06, 0.47, 0.97, 2.11),              # ~35 mph
  (-0.29, -0.14, -0.09, -0.07, 0.09, 0.23, 0.77, 1.62),              # ~55 mph
)

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
