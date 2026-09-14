"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ford curvature-primary lateral control, ported from BluePilot bp-7.0.

Upstream openpilot drives the PSCM with curvature (c2) alone and leaves the other three
polynomial coefficients at their inactive sentinels. This strategy drives all four:

  c2  curvature       the planner's desired curvature blended with the model's prediction
  c3  curvature_rate  the derivative of that prediction, so the PSCM anticipates a curve
                      instead of chasing it
  c1  path_angle      a PI term on lane position, for centering trim
  c0  path_offset     computed, used to drive the PI term, then zeroed on the wire -- c0 and c1
                      fight each other and the ride is worse with both

plus the ramp and precision toggles, which upstream pins to Slow/Precise.

One deliberate difference from BluePilot: a human-turn or standstill reset drops the lateral
message to mode 0 rather than holding the mode active with zeroed signals. BluePilot needs a
three-second blanket bypass of every safety check to make the latter work; mode 0 needs none,
because every check in safety/modes/ford.h has a legitimate !steer_control_enabled branch. This
is the same pattern BluePilot itself moved to for angle mode.

The rate limit below is BluePilot's own table rather than openpilot's ISO lateral acceleration
and jerk envelope, which the panda does not enforce in this mode. See FORD_BP_CURVATURE_ROC in
safety/modes/ford.h for what does still bound the command.

Background: https://bluepilot.dev/announcements/
"""
from collections import deque

import numpy as np
from numpy import clip, interp

from opendbc.car import DT_CTRL
from opendbc.car.common.pid import PIDController
from opendbc.car.ford.values import CarControllerParams, FordFlags
from opendbc.car.lateral import MAX_LATERAL_ACCEL, apply_std_steer_angle_limits
from opendbc.sunnypilot.car.ford.human_turn import HumanTurnDetector
from opendbc.sunnypilot.car.ford.lateral_common import INACTIVE_RESULT, FordLateralResult, get_current_curvature
from opendbc.sunnypilot.car.ford.values_ext import (
  BLEND_RATIO_RANGE,
  BP_ANGLE_LIMITS,
  CURVATURE_MAX,
  CURVATURE_RATE_MAX,
  CURV_MODE_PATH_ANGLE_MAX,
  LANE_CHANGE_FACTOR_CURV_RANGE,
  LANE_POSITIONING_GAIN_RANGE,
  PATH_OFFSET_RANGE,
  T_IDXS,
)

_STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL  # 20 Hz lateral tick
_LATERAL_RATE = 1.0 / _STEER_DT                       # Hz, for the PI controller

# How far ahead the model's curvature is sampled for the blend, and how far ahead its lateral
# position is sampled for the centering term.
_CURVATURE_LOOKUP_TIME = 0.2  # s
_PATH_OFFSET_LOOKUP_TIME = 0.2  # s

# Model-vs-planner blend, interpolated over the magnitude of the planner's command: gentle
# curves lean on the prediction, sharp ones on the planner.
_BLEND_RATIO_BP = [0.0, 0.001]  # 1/m

# Lane change: scale the command down in the direction of the change so it does not snap across,
# interpolated over speed between a fixed low-speed value and the user's factor.
_LANE_CHANGE_SPEED_BP = [4.4, 40.23]  # m/s
_LANE_CHANGE_FACTOR_LOW = 0.95

# curvature_rate is the derivative of the predicted curvature over this window, then scaled down
# on straights (where it is noise), at low speed, and in large curves (where it overshoots).
_CURVATURE_RATE_DELTA_T = 0.3  # s
_CURVATURE_RATE_SPEED_BP = [0.0, 14.5, 15.5]   # m/s
_CURVATURE_RATE_SPEED_V = [1.0, 1.0, 0.0]
_CURVATURE_RATE_PRED_BP = [0.0, 0.008, 0.01]   # 1/m
_CURVATURE_RATE_PRED_V = [0.0, 0.0, 1.0]
_LARGE_CURVE_FACTOR_BP = [0.001, 0.02]         # 1/m
_LARGE_CURVE_FACTOR_V = [1.0, 0.80]

# Lane position: blend the model's own path with the midpoint of the two inner lane lines,
# weighted by how much the lane lines are trusted. The width tolerance keeps the blend from
# jumping when lanes merge or diverge.
_LANELINE_WIDTH_BP = [3.75, 4.25]    # m
_LANELINE_WIDTH_V = [0.81, 0.59]
_LANELINE_CONFIDENCE_BP = [0.6, 0.8]

# PI controller on lane position, producing the c1 trim. Fades in with speed, because lane
# position is not measurable enough to act on at walking pace.
_LC_PID_K_P = 0.25
_LC_PID_K_I = 0.05
_LC_PID_SPEED_BP = [0.0, 9.0, 15.0]  # m/s
_LC_PID_SPEED_V = [0.0, 0.0, 1.0]
_LC_PATH_ANGLE_ROC_BP = [5, 15, 25]              # m/s
_LC_PATH_ANGLE_ROC_V = [0.003, 0.0015, 0.002]    # rad per call; mirrored by the panda
_LC_PID_RESET_HOLD_S = 1.5  # sustained driver pressure this long empties the integrator

# After a lane change completes, ease c1/c0/c3 back rather than stepping to the new targets.
_POST_LANE_CHANGE_FRAMES = 160  # ~8 s at 20 Hz
_MAX_PATH_ANGLE_CHANGE = 0.00125
_MAX_PATH_OFFSET_CHANGE = 0.00125
_MAX_CURVATURE_RATE_CHANGE = 0.00025  # >= the tightest curvature step (0.00008 at 25 m/s)

_RAMP_IMMEDIATE = 3
_RAMP_FAST = 2
_PRECISION_COMFORTABLE = 0
_PRECISION_PRECISE = 1

# Platform default blend ratios, used unless the user turns on the custom profile.
_DEFAULT_BLEND_RATIO = 0.40
_DEFAULT_LANE_POSITIONING_GAIN = 3.0


def _tuned(value, spec):
  """Clamp a user tuning value, falling back to the default when unset.

  CarParamsSP defaults every float to 0.0, so a CarParamsSP written before this feature existed
  (a replay, or a first boot after update) must not be read as a zeroed tuning value. Zero is a
  legitimate setting for path offset, which is why that one is not routed through here.
  """
  default, lo, hi = spec
  if not value:
    return default
  return float(clip(float(value), lo, hi))


def apply_ford_curvature_limits(apply_curvature, apply_curvature_last, current_curvature,
                                v_ego_raw, lat_active, CP):
  """Clip the commanded curvature to what the panda will accept.

  Returns (apply_curvature, deviation_limited). deviation_limited is True when the
  current-curvature band, rather than the rate or envelope limits, is what constrained the
  command this frame.
  """
  deviation_limited = False

  # No blending at low speed: there is no torque wind-up yet and the measurement is poor.
  if v_ego_raw > 9:
    pre_clip = apply_curvature
    apply_curvature = float(clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                                 current_curvature + CarControllerParams.CURVATURE_ERROR))
    deviation_limited = abs(apply_curvature - pre_clip) > 1e-9

  # BluePilot's three-point rate table. This is the rate limit, in place of the ISO lateral jerk
  # envelope: the panda enforces the same table in the BluePilot modes rather than the envelope.
  # See FORD_BP_CURVATURE_ROC in safety/modes/ford.h.
  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw,
                                                 0., lat_active, BP_ANGLE_LIMITS)

  # Ford Q4 / CAN FD has more torque available than Q3 / CAN, so cap it by lateral acceleration.
  # BluePilot keeps this cap on CAN FD only; CAN is bounded by the DBC's 0.02 1/m range alone.
  if CP.flags & FordFlags.CANFD:
    accel_limit = MAX_LATERAL_ACCEL / (max(v_ego_raw, 1) ** 2)
    apply_curvature = float(clip(apply_curvature, -accel_limit, accel_limit))

  return apply_curvature, deviation_limited


class LateralCurvExt:
  """Mixed into the Ford CarController. Owns every piece of curvature-mode state."""

  def __init__(self, CP, CP_SP):
    tuning = CP_SP.fordLateralTuning
    self.CP = CP

    self.human_turn_detection = bool(tuning.humanTurnDetection)
    self.lane_positioning = bool(tuning.lanePositioning)
    self.lane_full_mode = bool(tuning.laneFullMode)
    self.path_offset_trim = float(clip(tuning.pathOffset, PATH_OFFSET_RANGE[1], PATH_OFFSET_RANGE[2]))
    self.lane_change_factor_high = _tuned(tuning.laneChangeFactorCurv, LANE_CHANGE_FACTOR_CURV_RANGE)

    # The custom profile is what makes the blend ratios and the centering gain user-tunable at
    # all; with it off the platform defaults are used and the three values are ignored.
    if tuning.customProfile == 1:
      self.blend_ratio_low = _tuned(tuning.blendRatioLow, BLEND_RATIO_RANGE)
      self.blend_ratio_high = _tuned(tuning.blendRatioHigh, BLEND_RATIO_RANGE)
      self.lane_positioning_gain = _tuned(tuning.lanePositioningGain, LANE_POSITIONING_GAIN_RANGE)
    else:
      self.blend_ratio_low = _DEFAULT_BLEND_RATIO
      self.blend_ratio_high = _DEFAULT_BLEND_RATIO
      self.lane_positioning_gain = _DEFAULT_LANE_POSITIONING_GAIN

    self.pid = PIDController(k_p=_LC_PID_K_P, k_i=_LC_PID_K_I, rate=_LATERAL_RATE)
    self.pid_reset_frames = 0

    self.human_turn_detector = HumanTurnDetector()
    self.human_turn_active = False
    self.reset_steering_last = False
    self.post_reset_ramp_active = False

    self.curvature_rate_deque = deque(maxlen=max(2, int(round(_CURVATURE_RATE_DELTA_T / _STEER_DT))))
    self.curvature_deviation_limited = False

    self.lane_change = False
    self.lane_change_last = False
    self.post_lane_change_timer = 0
    self.post_lane_change_active = False
    self.post_lane_change_values = {'path_angle': 0.0, 'path_offset': 0.0, 'curvature_rate': 0.0}

    self.path_angle_last = 0.0

  def _reset(self) -> FordLateralResult:
    """Hand lateral back to the driver: mode 0, every signal at its sentinel."""
    self.path_angle_last = 0.0
    self.curvature_deviation_limited = False
    self.curvature_rate_deque.clear()
    self.pid.reset()
    return INACTIVE_RESULT

  def update(self, CC, CC_SP, CS, actuators, apply_curvature_last) -> FordLateralResult:
    """Compute the curvature-mode lateral command for one 20 Hz frame."""
    if not CC.latActive:
      self.human_turn_detector.reset()
      self.human_turn_active = False
      self.post_reset_ramp_active = False
      self.reset_steering_last = False
      return self._reset()

    v_ego = float(CS.out.vEgoRaw)
    lat = CC_SP.fordLateral
    model_curvatures = list(lat.modelCurvatures)
    model_position_y = list(lat.modelPositionY)
    have_model = len(model_curvatures) == len(T_IDXS)

    desired_curvature = float(actuators.curvature)
    current_curvature = get_current_curvature(CS)

    # *** model blend ***
    predicted_curvature = float(interp(_CURVATURE_LOOKUP_TIME, T_IDXS, model_curvatures)) if have_model else 0.0
    blend = float(interp(abs(desired_curvature), _BLEND_RATIO_BP,
                         [self.blend_ratio_low, self.blend_ratio_high]))
    requested_curvature = predicted_curvature * blend + desired_curvature * (1.0 - blend)

    # *** lane change ***
    self.lane_change = lat.laneChangeState in (1, 2, 3)
    precision = _PRECISION_PRECISE
    if self.lane_change:
      lane_change_factor = float(interp(v_ego, _LANE_CHANGE_SPEED_BP,
                                        [_LANE_CHANGE_FACTOR_LOW, self.lane_change_factor_high]))
      direction = lat.laneChangeDirection
      if (direction == 1 and requested_curvature < 0) or (direction == 2 and requested_curvature > 0):
        requested_curvature *= lane_change_factor
        precision = _PRECISION_COMFORTABLE

    # *** hand back to the driver ***
    # A sustained manual turn, or a standstill, drops the whole message to mode 0. BluePilot
    # instead holds the mode active with zeroed signals and relies on a blanket safety bypass to
    # survive it; see the module docstring.
    self.human_turn_active = self.human_turn_detector.update(
      self.human_turn_detection, CS.out.steeringPressed, CS.out.steeringAngleDeg)
    if self.human_turn_active or v_ego < 0.1:
      self.reset_steering_last = True
      self.post_reset_ramp_active = False
      return self._reset()

    if self.reset_steering_last:
      # Coming off a reset: ramp the command back in from zero rather than stepping to it.
      self.post_reset_ramp_active = True
      apply_curvature_last = 0.0
    self.reset_steering_last = False

    apply_curvature, self.curvature_deviation_limited = apply_ford_curvature_limits(
      requested_curvature, apply_curvature_last, current_curvature, v_ego, CC.latActive, self.CP)

    if self.post_reset_ramp_active:
      # Done ramping once the command is within 10% of what was asked for.
      if abs(requested_curvature - apply_curvature) < max(abs(requested_curvature) * 0.1, 0.001):
        self.post_reset_ramp_active = False

    # *** curvature rate ***
    # Derivative of the predicted curvature over a fixed window, in the PSCM's units (1/m^2).
    self.curvature_rate_deque.append(predicted_curvature)
    if len(self.curvature_rate_deque) > 1:
      full = len(self.curvature_rate_deque) == self.curvature_rate_deque.maxlen
      delta_t = _CURVATURE_RATE_DELTA_T if full else (len(self.curvature_rate_deque) - 1) * _STEER_DT
      curvature_rate = ((self.curvature_rate_deque[-1] - self.curvature_rate_deque[0])
                        / delta_t / max(0.01, v_ego))
    else:
      curvature_rate = 0.0

    # Scale it away where it does more harm than good: on straights (noise), at low speed, and in
    # large curves (overshoot).
    curvature_rate *= float(interp(abs(predicted_curvature), _CURVATURE_RATE_PRED_BP, _CURVATURE_RATE_PRED_V))
    curvature_rate *= float(interp(v_ego, _CURVATURE_RATE_SPEED_BP, _CURVATURE_RATE_SPEED_V))
    curvature_rate *= float(interp(abs(requested_curvature), _LARGE_CURVE_FACTOR_BP, _LARGE_CURVE_FACTOR_V))
    if self.lane_change:
      curvature_rate = 0.0

    # *** lane position and the c1 trim ***
    path_offset = self._lane_position(lat, model_position_y, have_model)
    path_angle = self._centering_trim(path_offset, v_ego, CS.out.steeringPressed)

    path_angle, path_offset, curvature_rate = self._post_lane_change(path_angle, path_offset, curvature_rate)

    apply_curvature = float(clip(apply_curvature, -CURVATURE_MAX, CURVATURE_MAX))
    curvature_rate = float(clip(curvature_rate, -CURVATURE_RATE_MAX, CURVATURE_RATE_MAX))
    path_angle = float(clip(path_angle, -CURV_MODE_PATH_ANGLE_MAX, CURV_MODE_PATH_ANGLE_MAX))
    self.path_angle_last = path_angle

    # c0 is computed above only to drive the trim; sending both it and c1 makes the ride worse,
    # so the wire always carries the sentinel.
    path_offset = 0.0

    return FordLateralResult(
      apply_curvature=apply_curvature,
      curvature_rate=curvature_rate,
      path_offset=path_offset,
      path_angle=path_angle,
      ramp_type=_RAMP_IMMEDIATE if self.post_reset_ramp_active else _RAMP_FAST,
      precision_type=precision,
      lat_inactive=False,
    )

  def _lane_position(self, lat, model_position_y, have_model) -> float:
    """Where the car should sit in the lane: the model's own path, blended toward the midpoint of
    the two inner lane lines by how much those lines are trusted."""
    if not have_model or len(model_position_y) != len(T_IDXS):
      return self.path_offset_trim

    from_model = float(interp(_PATH_OFFSET_LOOKUP_TIME, T_IDXS, model_position_y))
    from_lanelines = (lat.laneLineLeftY + lat.laneLineRightY) / 2.0

    # A lane that has gone unusually wide or narrow is merging or diverging; trust it less.
    laneline_width = lat.laneLineRightY - lat.laneLineLeftY
    width_tolerance = float(interp(laneline_width, _LANELINE_WIDTH_BP, _LANELINE_WIDTH_V))
    confidence = min(lat.laneLineLeftProb, lat.laneLineRightProb, width_tolerance)
    if not self.lane_full_mode:
      confidence = 0.0

    weight = float(interp(confidence, _LANELINE_CONFIDENCE_BP, [0.0, 1.0]))
    if self.lane_change:
      return 0.0
    return from_model * (1.0 - weight) + from_lanelines * weight + self.path_offset_trim

  def _centering_trim(self, path_offset: float, v_ego: float, steering_pressed: bool) -> float:
    """PI term on lane position, producing the c1 trim, rate limited for comfort."""
    if not self.lane_positioning:
      self.pid.reset()
      self.path_angle_last = 0.0
      return 0.0

    error = path_offset * (self.lane_positioning_gain / 100.0)
    error *= float(interp(v_ego, _LC_PID_SPEED_BP, _LC_PID_SPEED_V))
    path_angle = float(self.pid.update(error))

    roc = float(interp(abs(v_ego), _LC_PATH_ANGLE_ROC_BP, _LC_PATH_ANGLE_ROC_V))
    path_angle = float(clip(path_angle, self.path_angle_last - roc, self.path_angle_last + roc))

    # Sustained driver pressure means the driver, not the controller, is choosing lane position;
    # empty the integrator rather than fighting back when they let go.
    self.pid_reset_frames = self.pid_reset_frames + 1 if steering_pressed else 0
    if self.pid_reset_frames > _LC_PID_RESET_HOLD_S * _LATERAL_RATE:
      self.pid.reset()

    return path_angle

  def _post_lane_change(self, path_angle, path_offset, curvature_rate):
    """Ease the trim signals back after a lane change instead of stepping to the new targets."""
    if self.lane_change_last and not self.lane_change:
      self.post_lane_change_active = True
      self.post_lane_change_timer = 0
      self.post_lane_change_values = {'path_angle': 0.0, 'path_offset': 0.0, 'curvature_rate': 0.0}
    self.lane_change_last = self.lane_change

    if not self.post_lane_change_active:
      return path_angle, path_offset, curvature_rate

    self.post_lane_change_timer += 1
    last = self.post_lane_change_values
    path_angle = float(np.clip(path_angle, last['path_angle'] - _MAX_PATH_ANGLE_CHANGE,
                               last['path_angle'] + _MAX_PATH_ANGLE_CHANGE))
    path_offset = float(np.clip(path_offset, last['path_offset'] - _MAX_PATH_OFFSET_CHANGE,
                                last['path_offset'] + _MAX_PATH_OFFSET_CHANGE))
    curvature_rate = float(np.clip(curvature_rate, last['curvature_rate'] - _MAX_CURVATURE_RATE_CHANGE,
                                   last['curvature_rate'] + _MAX_CURVATURE_RATE_CHANGE))
    self.post_lane_change_values = {'path_angle': path_angle, 'path_offset': path_offset,
                                    'curvature_rate': curvature_rate}

    if self.post_lane_change_timer >= _POST_LANE_CHANGE_FRAMES:
      self.post_lane_change_active = False

    return path_angle, path_offset, curvature_rate
