"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ford path-angle-primary lateral control ("angle control"), ported from BluePilot bp-7.0.

The PSCM evaluates y(x) = c0 + c1*x + 1/2*c2*x^2 + 1/6*c3*x^3 at a lookahead of only ~1.4 m at
highway speed, so c0 (path_offset) and c1 (path_angle) dominate and c2 (curvature) contributes at
the centimeter scale. Worse, c2 is fed through a speed-indexed low-pass filter inside the PSCM
(~35 ms at 100 km/h rising to ~330 ms at 5 km/h), so the module keeps acting on a curvature
command for up to a second after openpilot drops it. That lag, not model noise, is what produced
the ping-pong on Fords.

Angle control holds c0/c2/c3 at their inactive sentinels and steers with c1 alone:

    path_angle = kappa * v_ego * gain

where kappa is the planner's desired curvature (optionally blended with the model's predicted
curvature) and gain is a per-platform calibration times the user's tuning factors. The PSCM
executes the commanded angle directly, with no filter state to unwind.

Background: https://bluepilot.dev/announcements/
"""
from collections import deque

from numpy import clip, interp

from opendbc.car import DT_CTRL
from opendbc.car.ford.values import CarControllerParams
from opendbc.sunnypilot.car.ford.human_turn import HumanTurnDetector
from opendbc.sunnypilot.car.ford.lateral_common import INACTIVE_RESULT, FordLateralResult, get_current_curvature
from opendbc.sunnypilot.car.ford.values_ext import (
  FORD_DBC_PATH_ANGLE_MAX,
  FORD_DBC_PATH_ANGLE_MIN,
  HIGH_SPEED_DAMPENING_RANGE,
  HIGH_SPEED_FACTOR_RANGE,
  LANE_CHANGE_FACTOR_RANGE,
  LOW_SPEED_FACTOR_RANGE,
  T_IDXS,
  platform_path_angle_gains,
)

_STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL  # 20 Hz lateral tick

# Blend of model-predicted curvature into the planner's desired curvature, validated on F-150
# fleet data. 0 = planner only, 1 = model only.
_PATH_ANGLE_BLEND_RATIO = 0.50

# Variable lookup time (VLT): how far ahead the model curvature is sampled for the blend.
#   t_lookup = t_base + t_extra_max * speed_factor(v) * kappa_factor(|kappa|)
# t_base tracks the planner's own pre-compensation floor. The extra lookahead collapses toward
# zero at high speed (the PSCM responds faster) and at large curvature (so the blend does not
# import a "start unwinding" signal before the apex).
_DT_MDL = 0.05                    # model loop period
_VLT_T_EXTRA_MAX = 0.10           # max extra lookahead above t_base
_VLT_V_LOW_MS = 25.0 * 0.44704    # 25 mph -- full extra lookahead at or below
_VLT_V_HIGH_MS = 55.0 * 0.44704   # 55 mph -- no extra lookahead at or above
_VLT_KAPPA_FULL = 0.005           # 1/m -- full extra lookahead below this (200 m+ radius)
_VLT_KAPPA_TAPER = 0.020          # 1/m -- no extra lookahead above this (50 m radius)
# liveDelay can calibrate out to ~420 ms on some routes, which would inflate VLT to 0.6 s and push
# the model lookahead 5 m into a curve. At that depth the model sees full peak curvature, the
# "entering" test never goes False, and the exit-biased blend below never engages -- the car then
# commands max path_angle through the whole apex. Cap it so VLT stays <= 0.33 s.
_VLT_DELAY_MIN = 0.10
_VLT_DELAY_MAX = 0.15

# Speed breakpoints over which the platform gains and the user's factors fade in. Below the first
# breakpoint the gain is 1.0 in both curvature regimes.
_GAIN_SPEED_BP = [13.5, 26.82]    # m/s (~30 mph, ~60 mph)
# Curvature breakpoints between the low-curvature (straight/gentle) and high-curvature gains.
_GAIN_CURV_BP = [0.0007, 0.001]   # 1/m
# Low-speed authority multiplier the low-speed factor scales.
_LOW_SPEED_GAIN = 1.30

# Lane change scaling: interpolated over speed between a fixed low-speed value and the user's
# factor. Angle mode wants a boost (>1) where curvature mode wants a cut (<1).
_LANE_CHANGE_SPEED_BP = [4.4, 40.23]  # m/s
_LANE_CHANGE_FACTOR_LOW = 0.95

# Rate cap on path_angle magnitude DECREASE while pinned at the DBC limit (0.02 rad/call at 20 Hz
# = 0.40 rad/s). Both model and planner naturally drop path_angle ~0.36 rad/s at a sharp 90 deg
# apex while the PSCM is physically pinned and cannot execute it; the resulting desired-vs-actual
# gap (up to 47 deg observed) snaps the moment the PSCM is released. Holding the car slightly
# longer in the curve is the cheaper failure.
_PSCM_SAT_UNWIND_RATE = 0.02      # rad/call
_DBC_SAT_FRACTION = 0.90          # fraction of the DBC limit that counts as saturated
# Firmware cross-check (values_ext PSCM_SLEW_*): the module's own limiter unwinds 1.385x faster
# than it winds up (cal 1.3 down vs 1.8 up). _PSCM_SAT_UNWIND_RATE goes the other way, capping the
# decrease tighter than the wind-up the soft ROC below allows, because it is guarding the
# command-vs-delivery gap that snaps on release, not tracking the module's slew. The cal units
# were never pinned to physical units, so the two are not directly comparable and nothing here is
# re-derived from them; if a sweep ever pins the units, this asymmetry is the first thing to
# revisit.

# *** delivered-vs-commanded saturation observer ***
# The PSCM saturates its own output (firmware +-8.8 / +-10.0 / +-1440) and attenuates delivery
# well before path_angle reaches the DBC limit, and it does not say so on the bus: LatCtlLim_D_Stat
# is silent on CAN FD in angle mode. So _dbc_sat above fires late and the controller commands into
# the attenuation region blind.
#
# Measure it instead: compare the curvature the car actually achieved against the curvature that
# was commanded one actuator lag ago. An earlier attempt used angleState.saturated, which trips on
# command-vs-actual lag, normal on any curve entry, and produced a positive-feedback flat line
# (understeer -> saturated -> command frozen -> more understeer). Four things keep this one out of
# that hole:
#   * it compares against a delay-aligned command, so ordinary entry lag is not read as attenuation
#   * the ratio is low-passed, then has to hold below threshold for a debounce before the flag sets
#   * it runs hands-free only, above a speed floor, and only while commanding a real curve
#   * it can only gate the saturation handling that already exists. It never adds command of its
#     own, it clears on hysteresis, and a hard hold limit with a refractory period means a wrong
#     ratio cannot freeze the command for a whole drive.
#
# The ratio is measured against the *requested* curvature, before the deviation clip, not against
# the clipped kappa_cmd that goes out as path_angle. That is deliberate and it is not what the
# handoff suggested. The clip pins kappa_cmd at measured + CURVATURE_ERROR whenever the command
# leads the car, which puts an algebraic floor of m / (m + 0.002) under any post-clip ratio: 0.83
# at a 100 m radius, 0.91 at 50 m. A post-clip observer therefore cannot see attenuation at
# exactly the curvatures where the PSCM attenuates. Requested-vs-delivered asks the question that
# matters anyway -- how much of what the planner wanted did the truck actually do -- and it counts
# the clip's own throttling, which the saturation handling should respond to for the same reason.
_SAT_OBS_LAG_S = 0.031            # s, measured actuator lag the comparison aligns to
_SAT_OBS_MIN_KAPPA = 0.004        # 1/m -- inside a 250 m radius; below this the ratio is noise
_SAT_OBS_MIN_SPEED = 5.0          # m/s -- yaw rate over speed is not meaningful below this
_SAT_OBS_TAU_S = 0.50             # low-pass on the delivery ratio
_SAT_OBS_ENTER = 0.75             # delivered/requested below this counts as attenuating
_SAT_OBS_EXIT = 0.85              # and must come back above this to clear (hysteresis)
_SAT_OBS_DEBOUNCE_S = 0.50        # how long it must hold below before the flag sets
# Curve entry is the one place a low ratio is honest and meaningless: the request is climbing and
# the car has not caught up yet. That is the lag the old angleState.saturated attempt mistook for
# saturation. Rather than trying to out-filter it, the debounce simply does not accumulate while
# the request is still growing. Attenuation is a problem at and after the apex, which is exactly
# where this gate opens.
_SAT_OBS_RISING_EPS = 1e-4        # 1/m per call below which "still growing" is just noise
_SAT_OBS_MAX_HOLD_S = 10.0        # hard release; longer than any real corner, short of a latch
_SAT_OBS_REFRACTORY_S = 1.0       # after a hard release, before it may arm again
_SAT_OBS_RATIO_NEUTRAL = 1.0      # what the filter holds while gated, i.e. "delivering fine"

# Closed-loop correction on the shortfall the observer measures.
#
# path_angle = kappa * v * gain is open loop: nothing anywhere checks whether the module
# actually produced the curvature that was asked of it. On this truck it does not. System ID
# over four logs, hands-off segments only, lag aligned, gives delivered/commanded = 0.874
# (R2 0.94-0.97 per log, 0.851 at gentle curvature and 0.906 at moderate). That shortfall is
# uncorrected, so the truck runs wide and the driver adds the rest.
#
# delivery_ratio above is already the lag-aligned, low-passed measurement of exactly that, so
# this scales the gain by its reciprocal.
#
# It only ever adds. Under-delivery is the failure mode, and cutting the command when the
# ratio reads high would hand a measurement artifact, a cambered road or a driver nudge, the
# authority to steer less in a curve. The cap is what a 0.80 ratio needs; anything below that
# is a broken measurement rather than a gain to chase, and is ignored rather than extrapolated.
#
# Its own time constant is far slower than the ratio filter's, so the correction walks in over
# a couple of seconds instead of tracking the ratio's own movement through a corner, and it
# decays back to neutral slower still whenever the measurement stops being trustworthy, which
# is every driver touch, lane change and stall blip.
_DELIVERY_COMP_MAX = 1.25         # ceiling on the boost
_DELIVERY_COMP_MIN_RATIO = 0.80   # below this the measurement is suspect, not a target
_DELIVERY_COMP_TAU_S = 2.0        # rise
_DELIVERY_COMP_DECAY_TAU_S = 4.0  # fall back to 1.0 when the measurement is gated
_SAT_OBS_RATIO_MAX = 2.0          # clamp, so one bad frame cannot drag the filter far
# The lag is finer than the 50 ms lateral tick, so the aligned request is interpolated between
# the two bracketing frames rather than rounded to one of them. Rounding 31 ms to a whole frame
# would mean comparing against a command either 0 ms or 50 ms old, and at 20 Hz on a curve entry
# that difference is most of the ratio.
_SAT_OBS_LAG_IDX = int(_SAT_OBS_LAG_S // _STEER_DT)               # whole frames back
_SAT_OBS_LAG_W = (_SAT_OBS_LAG_S / _STEER_DT) - _SAT_OBS_LAG_IDX  # weight on the older frame
_SAT_OBS_HIST_LEN = _SAT_OBS_LAG_IDX + 2

# Soft rate-of-change limit on path_angle, per lateral call (20 Hz). Deliberately slightly
# tighter than the panda mirror in safety/modes/ford.h so openpilot never provokes a block.
_SOFT_ROC_SPEED_BP = [9.0, 10.0, 15.0, 25.0]      # m/s
_SOFT_ROC_V = [0.055, 0.055, 0.0425, 0.009]       # rad/call

# Exit-biased blend: near the DBC limit, or while the planner is actively unwinding, drop the
# model's weight so the planner's unwind dominates instead of being diluted by a prediction that
# still sees the curve (slow unwind) or that snaps when its window crosses the exit.
_EXIT_BLEND_SCALE = 0.25
_EXIT_DESIRED_FALLING = 0.010     # (1/m) per call; below this a falling command is just noise

# Post-override stall blip. After a driver-touch episode the PSCM can keep reporting InProgress
# while honoring path_angle at only ~0.56x (healthy hands-free delivery is ~0.95). The
# deviation clip below then pins kappa_cmd at measured + CURVATURE_ERROR, so the command can never
# lead the car enough to overcome the attenuation -- a stall the driver reads as "not engaging".
# A short mode-0 pulse (the same panda-clean wire pattern the human-turn override sends) resets
# the PSCM's authority, after which path_angle ramps back in from zero through the soft ROC.
_STALL_GAP_MIN = 2.0 * CarControllerParams.CURVATURE_ERROR
_STALL_HOLD_S = 0.5               # accumulated clip-binding time before a pulse fires
_STALL_BLIP_FRAMES = 6            # 6 frames @ 20 Hz = 300 ms; the PSCM acked mode 0 in ~150 ms
_STALL_COOLDOWN_S = 2.0           # re-arm delay after a pulse
_STALL_MAX_BLIPS = 3              # give up on a stuck episode rather than pulsing forever
# Proactive hand-off blip: any sustained driver press attenuates the PSCM, and waiting for the
# reactive detector above costs a missed curve. Firing the same pulse on the falling edge of a
# sustained press resets the PSCM while the car is still straight and the command small.
_PRESS_BLIP_MIN_S = 0.5
_BLIP_MAX_PATH_ANGLE = 0.10       # rad -- the pulse releases steering for 300 ms; never in a curve

def _tuned(value: float, spec: tuple[float, float, float]) -> float:
  """Clamp a user tuning factor, falling back to the default when unset.

  CarParamsSP defaults every float to 0.0, so a CarParamsSP written before this feature existed
  (a replay, or a first boot after update) must not be read as "zero steering gain".
  """
  default, lo, hi = spec
  if not value:
    return default
  return float(clip(value, lo, hi))


class LateralAngleExt:
  """Mixed into the Ford CarController. Owns every piece of angle-mode state."""

  def __init__(self, CP, CP_SP):
    tuning = CP_SP.fordLateralTuning

    # Per-platform gains, applied only at high speed (see _GAIN_SPEED_BP).
    self.path_angle_gain_low_curv, self.path_angle_gain_high_curv = platform_path_angle_gains(CP.carFingerprint)

    # User tuning. Read once at car init: the panda safety flag comes from the same read, and a
    # live flip against stale firmware would fight the panda.
    self.low_speed_factor = _tuned(tuning.lowSpeedFactor, LOW_SPEED_FACTOR_RANGE)
    self.high_speed_factor = _tuned(tuning.highSpeedFactor, HIGH_SPEED_FACTOR_RANGE)
    self.high_speed_dampening = _tuned(tuning.highSpeedDampening, HIGH_SPEED_DAMPENING_RANGE)
    self.lane_change_factor_high = _tuned(tuning.laneChangeFactor, LANE_CHANGE_FACTOR_RANGE)
    # Hand lateral back while the driver holds a real turn. Was hard-wired on here, which made the
    # setting a curvature-mode-only control in practice and cost a 2.5 s dropout in the middle of a
    # 15 m radius corner on a logged drive (s12 t=759.6, wheel at -241 deg, latActive throughout).
    # It stays on by default, because the PSCM re-engage stall it prevents is real, but it is the
    # driver's call now.
    self.human_turn_detection = bool(tuning.humanTurnDetection)

    self.path_angle_last = 0.0
    self.desired_curvature_last = 0.0

    # kappa that path_angle was derived from. Published to the panda as shadow_curvature: angle
    # mode pins the real curvature signal at its inactive sentinel, so without this there is no
    # commanded-vs-measured deviation check for angle mode at all.
    self.shadow_curvature = 0.0
    self.curvature_deviation_limited = False

    self.human_turn_detector = HumanTurnDetector()
    self.human_turn_active = False

    self.stall_blip_hold_s = 0.0
    self.stall_blip_frames_left = 0
    self.stall_blip_cooldown_s = 0.0
    self.stall_blip_count = 0
    self.stall_blip_active = False
    self.press_timer_s = 0.0

    self.lane_change = False

    # Delivered-vs-commanded saturation observer. Default off: this is live steering code and the
    # thresholds below are road-validated on one truck, not derived from the firmware.
    self.sat_observer_enabled = bool(tuning.satObserver)
    # Acting on that measurement rather than only flagging it. Separate toggle: the observer is
    # a detector and can run on its own, this closes a loop around live steering.
    self.delivery_comp_enabled = bool(tuning.deliveryCompensation)
    self.delivery_comp = 1.0
    self.delivery_measured = False
    self.kappa_req_history: deque[float] = deque(maxlen=_SAT_OBS_HIST_LEN)
    self.kappa_req_last = 0.0
    self.delivery_ratio = _SAT_OBS_RATIO_NEUTRAL
    self.delivery_measured = False
    self.pscm_attenuating = False
    self.sat_obs_below_s = 0.0
    self.sat_obs_hold_s = 0.0
    self.sat_obs_refractory_s = 0.0

  def _clear_saturation_observer(self) -> None:
    """Back to "delivering fine". Every path that puts mode 0 on the wire lands here: the PSCM's
    authority resets with the mode drop, so a ratio measured before it says nothing after it."""
    self.kappa_req_history.clear()
    self.kappa_req_last = 0.0
    self.delivery_ratio = _SAT_OBS_RATIO_NEUTRAL
    self.delivery_measured = False
    self.pscm_attenuating = False
    self.sat_obs_below_s = 0.0
    self.sat_obs_hold_s = 0.0

  def _update_saturation_observer(self, CS, v_ego: float, kappa_req: float) -> None:
    """Fold this frame into the delivery-ratio filter and set the attenuating flag.

    Runs at the end of the frame, so the flag is read one 20 Hz tick later. Against a 0.5 s
    low-pass and a 0.5 s debounce, 50 ms of staleness does not matter, and computing it here is
    what lets the comparison use the request this frame actually produced."""
    self.sat_obs_refractory_s = max(0.0, self.sat_obs_refractory_s - _STEER_DT)

    # Hands-free only, and only while the command means something. Everything in this list either
    # moves the measurement independently of our command (driver) or deliberately drops the mode
    # (blip), both of which make the ratio meaningless rather than low.
    if (CS.out.steeringPressed or self.human_turn_active or self.lane_change
        or self.stall_blip_active or v_ego < _SAT_OBS_MIN_SPEED):
      self._clear_saturation_observer()
      self._update_delivery_compensation()
      return

    rising = abs(kappa_req) > abs(self.kappa_req_last) + _SAT_OBS_RISING_EPS
    self.kappa_req_last = kappa_req

    self.kappa_req_history.append(kappa_req)
    if len(self.kappa_req_history) < self.kappa_req_history.maxlen:
      self._update_delivery_compensation()
      return

    # Delay-aligned: what the car is doing now against what was asked for one actuator lag ago.
    # This is the difference between measuring attenuation and measuring ordinary entry lag.
    newer = self.kappa_req_history[-1 - _SAT_OBS_LAG_IDX]
    older = self.kappa_req_history[-2 - _SAT_OBS_LAG_IDX]
    kappa_then = newer * (1.0 - _SAT_OBS_LAG_W) + older * _SAT_OBS_LAG_W
    kappa_now = get_current_curvature(CS)
    alpha = _STEER_DT / (_SAT_OBS_TAU_S + _STEER_DT)
    self.delivery_measured = abs(kappa_then) >= _SAT_OBS_MIN_KAPPA and kappa_then * kappa_now > 0.0
    if self.delivery_measured:
      ratio = float(clip(kappa_now / kappa_then, 0.0, _SAT_OBS_RATIO_MAX))
      self.delivery_ratio += alpha * (ratio - self.delivery_ratio)
    else:
      # Not asking for a real curve, or the car is going the other way: decay back toward neutral
      # rather than carrying a stale ratio into the next curve.
      self.delivery_ratio += alpha * (_SAT_OBS_RATIO_NEUTRAL - self.delivery_ratio)

    if self.delivery_ratio < _SAT_OBS_ENTER and not rising:
      self.sat_obs_below_s += _STEER_DT
    elif self.delivery_ratio > _SAT_OBS_EXIT:
      self.sat_obs_below_s = 0.0

    if self.pscm_attenuating:
      self.sat_obs_hold_s += _STEER_DT
      if self.delivery_ratio > _SAT_OBS_EXIT:
        self.pscm_attenuating = False
        self.sat_obs_below_s = 0.0
        self.sat_obs_hold_s = 0.0
      elif self.sat_obs_hold_s >= _SAT_OBS_MAX_HOLD_S:
        # Longer than any real corner. Whatever is driving the ratio down is not something this
        # observer should keep freezing the command over, so let go and make it re-earn the flag.
        self.pscm_attenuating = False
        self.sat_obs_below_s = 0.0
        self.sat_obs_hold_s = 0.0
        self.sat_obs_refractory_s = _SAT_OBS_REFRACTORY_S
    elif self.sat_obs_below_s >= _SAT_OBS_DEBOUNCE_S and self.sat_obs_refractory_s <= 0.0:
      self.pscm_attenuating = True
      self.sat_obs_hold_s = 0.0

    self._update_delivery_compensation()

  def _update_delivery_compensation(self) -> None:
    """Walk the gain correction toward whatever cancels the measured shortfall.

    Called from every exit of the observer, including the gated ones, so the correction always
    moves: toward the reciprocal of the ratio while the measurement is live, and back toward
    neutral whenever it is not. Raising the gain raises path_angle only. shadow_curvature, which
    is what the panda deviation-checks, is kappa_cmd and is not touched, so a bigger correction
    moves the measured curvature toward the shadow rather than away from it.
    """
    if self.delivery_measured and self.delivery_ratio >= _DELIVERY_COMP_MIN_RATIO:
      target = float(clip(1.0 / self.delivery_ratio, 1.0, _DELIVERY_COMP_MAX))
      tau = _DELIVERY_COMP_TAU_S
    else:
      target = 1.0
      tau = _DELIVERY_COMP_DECAY_TAU_S
    alpha = _STEER_DT / (tau + _STEER_DT)
    self.delivery_comp += alpha * (target - self.delivery_comp)

  def _reset(self, CS, actuators=None) -> FordLateralResult:
    """Zero the command and publish a truthful shadow. Used by the inactive, human-turn and
    stall-blip paths, which all put mode 0 on the wire."""
    self.path_angle_last = 0.0
    self.curvature_deviation_limited = False
    self._clear_saturation_observer()
    # Lane_Assist_Data1 carries the shadow at 33 Hz whenever angle mode is configured, and the
    # panda latches it from every such frame regardless of whether lateral is active. Parking it
    # at a stale zero would make the first enabled LMC frame after re-engage race that latch
    # against a real measured curvature, so track reality here instead.
    self.shadow_curvature = get_current_curvature(CS)
    if actuators is not None:
      self.desired_curvature_last = float(actuators.curvature)
    return INACTIVE_RESULT

  def _clear_stall_state(self) -> None:
    self.stall_blip_hold_s = 0.0
    self.stall_blip_frames_left = 0
    self.stall_blip_cooldown_s = 0.0
    self.stall_blip_count = 0
    self.stall_blip_active = False
    self.press_timer_s = 0.0

  def update(self, CC, CC_SP, CS, actuators) -> FordLateralResult:
    """Compute the angle-mode lateral command for one 20 Hz frame."""
    v_ego = float(CS.out.vEgoRaw)
    model_curvatures = list(CC_SP.fordLateral.modelCurvatures)
    have_model = len(model_curvatures) == len(T_IDXS)

    if not CC.latActive:
      self.human_turn_detector.reset()
      self.human_turn_active = False
      self._clear_stall_state()
      self.desired_curvature_last = 0.0
      return self._reset(CS)

    # Human-turn override: a sustained driver press at a large wheel angle forces lateral
    # inactive so path_angle cannot wind into a stale command while the driver turns. On release
    # path_angle ramps back in from zero through the soft ROC; no seeding, no bypass. Under the
    # driver's control, because handing lateral back mid-corner is the single largest deviation
    # from carControl this controller can make.
    self.human_turn_active = self.human_turn_detector.update(self.human_turn_detection, CS.out.steeringPressed,
                                                             CS.out.steeringAngleDeg)
    if self.human_turn_active:
      # A human turn ends any stall episode: its own mode 0 does the PSCM reset job. That also
      # covers the press so far, so only press time after the latch releases earns a hand-off pulse.
      self._clear_stall_state()
      return self._reset(CS, actuators)

    # Proactive hand-off blip on the falling edge of a sustained press.
    if CS.out.steeringPressed:
      self.press_timer_s += _STEER_DT
    else:
      if (self.press_timer_s >= _PRESS_BLIP_MIN_S and self.stall_blip_cooldown_s <= 0.0
          and self.stall_blip_frames_left <= 0
          and abs(self.path_angle_last) < _BLIP_MAX_PATH_ANGLE):
        self.stall_blip_frames_left = _STALL_BLIP_FRAMES
      self.press_timer_s = 0.0

    if self.stall_blip_frames_left > 0:
      self.stall_blip_frames_left -= 1
      self.stall_blip_active = True
      if self.stall_blip_frames_left <= 0:
        self.stall_blip_cooldown_s = _STALL_COOLDOWN_S
      return self._reset(CS, actuators)
    self.stall_blip_active = False

    desired_curvature = float(actuators.curvature)

    # *** variable lookup time ***
    t_base = float(clip(CC_SP.fordLateral.lateralDelay, _VLT_DELAY_MIN, _VLT_DELAY_MAX)) + _DT_MDL
    speed_factor = float(interp(v_ego, [_VLT_V_LOW_MS, _VLT_V_HIGH_MS], [1.0, 0.0]))
    # Direction-aware: on curve entry (the model shows more curvature at t_base than the planner
    # commands now) keep full lookahead so pre-steering starts early. At the apex and on exit,
    # taper by magnitude so the blend does not start unwinding early.
    kappa_at_t_base = abs(float(interp(t_base, T_IDXS, model_curvatures))) if have_model else 0.0
    kappa_entering = kappa_at_t_base > abs(desired_curvature)
    if kappa_entering:
      kappa_factor = 1.0
    else:
      kappa_factor = float(interp(abs(desired_curvature), [_VLT_KAPPA_FULL, _VLT_KAPPA_TAPER], [1.0, 0.0]))
    lookup_time = t_base + _VLT_T_EXTRA_MAX * speed_factor * kappa_factor

    predicted_curvature = float(interp(lookup_time, T_IDXS, model_curvatures)) if have_model else 0.0

    # *** exit-biased blend ***
    # Saturation has two sources. DBC-limit proximity is the hard backstop and is always on, but
    # it fires late: the PSCM attenuates delivery long before path_angle reaches the CAN limit and
    # does not report it (LatCtlLim_D_Stat is silent on CAN FD in angle mode). The observer is the
    # early one, measured from delivered-vs-commanded curvature, and is off unless the driver
    # enables it. angleState.saturated is not used as a proxy for either: it trips whenever the car
    # lags the commanded path_angle by > 2.5 deg, i.e. on any normal curve entry, which produced a
    # positive-feedback flat line (understeer -> saturated -> path_angle frozen -> more understeer).
    dbc_saturated = (self.path_angle_last >= FORD_DBC_PATH_ANGLE_MAX * _DBC_SAT_FRACTION or
                     self.path_angle_last <= FORD_DBC_PATH_ANGLE_MIN * _DBC_SAT_FRACTION)
    saturated = dbc_saturated or (self.sat_observer_enabled and self.pscm_attenuating)
    desired_falling = abs(desired_curvature) < abs(self.desired_curvature_last) - _EXIT_DESIRED_FALLING
    on_exit = not kappa_entering and (saturated or desired_falling)
    blend = _PATH_ANGLE_BLEND_RATIO * _EXIT_BLEND_SCALE if on_exit else _PATH_ANGLE_BLEND_RATIO
    requested_curvature = predicted_curvature * blend + desired_curvature * (1.0 - blend)
    self.desired_curvature_last = desired_curvature

    # *** lane change ***
    self.lane_change = CC_SP.fordLateral.laneChangeState in (1, 2, 3)
    precision = 1
    if self.lane_change:
      lane_change_factor = float(interp(v_ego, _LANE_CHANGE_SPEED_BP,
                                        [_LANE_CHANGE_FACTOR_LOW, self.lane_change_factor_high]))
      direction = CC_SP.fordLateral.laneChangeDirection
      if (direction == 1 and requested_curvature < 0) or (direction == 2 and requested_curvature > 0):
        requested_curvature *= lane_change_factor
        precision = 0

    # *** deviation clip ***
    # Clip to measured curvature +- CURVATURE_ERROR, the same clip the stock curvature path
    # applies. Without it kappa_cmd -- and therefore path_angle, and the shadow curvature the
    # panda checks -- can legitimately lead the measurement by more than the safety tolerance
    # during ordinary curve entry, and the deviation check would block routinely rather than only
    # on a genuine divergence. Clipping the steering intent itself, not just the reported value,
    # is what keeps that check meaningful.
    kappa_cmd = float(requested_curvature)
    current_curvature = get_current_curvature(CS)
    self.curvature_deviation_limited = False
    if v_ego > 9:
      kappa_pre_clip = kappa_cmd
      kappa_cmd = float(clip(kappa_cmd, current_curvature - CarControllerParams.CURVATURE_ERROR,
                             current_curvature + CarControllerParams.CURVATURE_ERROR))
      self.curvature_deviation_limited = abs(kappa_cmd - kappa_pre_clip) > 1e-9

    # No curvature clamp here, deliberately. CURVATURE_MAX is the range of the c2 signal, and angle
    # mode does not steer with c2: it steers with c1, which has its own range (FORD_DBC_PATH_ANGLE_*)
    # and its own rate limit, both applied below and both mirrored by the panda. Clamping the
    # intent at 0.02 1/m capped path_angle at 0.02 * v * gain no matter how tight the corner, which
    # on a logged 15 m radius turn held the command at 34% of what carControl asked for while the
    # PSCM was still following. BluePilot does not clamp here either. The value that does go to the
    # panda as shadow_curvature is saturated at the limit on the wire, in create_lka_msg, so the
    # safety check still sees an in-range number without the actuator inheriting the c2 ceiling.

    # *** kappa -> path_angle ***
    low_gain = float(interp(v_ego, _GAIN_SPEED_BP,
                            [1.0, self.path_angle_gain_low_curv * self.high_speed_dampening]))
    high_gain = float(interp(v_ego, _GAIN_SPEED_BP,
                             [_LOW_SPEED_GAIN * self.low_speed_factor,
                              self.path_angle_gain_high_curv * self.high_speed_factor]))
    # Bigger curves need a little more signal to avoid understeer.
    gain = float(interp(abs(kappa_cmd), _GAIN_CURV_BP, [low_gain, high_gain]))
    # Cancel the measured delivery shortfall. Computed at the end of the previous frame, so it
    # is one 20 Hz tick old, which against a 2 s time constant does not matter.
    if self.delivery_comp_enabled:
      gain *= self.delivery_comp
    path_angle = kappa_cmd * v_ego * gain

    # *** saturation handling ***
    if saturated:
      last_mag = abs(self.path_angle_last)
      if abs(path_angle) > last_mag:
        path_angle = self.path_angle_last                      # magnitude growing: block
      elif last_mag - abs(path_angle) > _PSCM_SAT_UNWIND_RATE:  # unwinding faster than the PSCM can
        limited = last_mag - _PSCM_SAT_UNWIND_RATE
        path_angle = float(limited if self.path_angle_last >= 0 else -limited)

    path_angle = float(clip(path_angle, FORD_DBC_PATH_ANGLE_MIN, FORD_DBC_PATH_ANGLE_MAX))

    # *** soft rate limit ***
    soft_roc = float(interp(v_ego, _SOFT_ROC_SPEED_BP, _SOFT_ROC_V))
    path_angle = float(clip(path_angle, self.path_angle_last - soft_roc, self.path_angle_last + soft_roc))
    self.path_angle_last = path_angle

    # While the driver is pressing but the human-turn override has not latched yet, the clipped
    # planner kappa cannot keep up with the wheel: the driver moves the measurement faster than
    # the deviation clip tracks it, and the shadow can leave the panda's error band mid-curve.
    # The honest command during a press is the driver's actual curvature.
    self.shadow_curvature = current_curvature if CS.out.steeringPressed else kappa_cmd

    self._update_saturation_observer(CS, v_ego, float(requested_curvature))
    self._update_stall_detection(CS, desired_curvature, current_curvature)

    return FordLateralResult(
      apply_curvature=0.0,
      curvature_rate=0.0,
      path_offset=0.0,
      path_angle=path_angle,
      # Inert in Limited mode: the PSCM's slew rate and deadband are fixed calibration
      # (values_ext PSCM_SLEW_*, PSCM_DEADBAND_CAL) and no consumer of a received ramp or
      # precision request selects either. These values are kept exactly as they have always been
      # transmitted so the wire is unchanged, but nothing should be tuned on them, and `precision`
      # above is a lane-change marker rather than a request the module acts on.
      ramp_type=2,
      precision_type=precision,
      lat_inactive=False,
    )

  def _update_stall_detection(self, CS, desired_curvature: float, current_curvature: float) -> None:
    """Arm the mode-0 pulse when, hands-free, the command has led the measurement by more than
    twice the deviation tolerance while that clip was actually binding. The clip flickers
    mid-stall (~63% duty on the diagnosis route), so off frames hold the accumulator rather than
    resetting it; a closed gap or a driver press ends the episode."""
    self.stall_blip_cooldown_s = max(0.0, self.stall_blip_cooldown_s - _STEER_DT)

    gap = desired_curvature - current_curvature
    stalled = (not CS.out.steeringPressed and not self.lane_change and CS.out.vEgoRaw > 9.0
               and abs(gap) > _STALL_GAP_MIN and abs(desired_curvature) > abs(current_curvature))

    if stalled:
      if self.curvature_deviation_limited and self.stall_blip_cooldown_s <= 0.0:
        self.stall_blip_hold_s += _STEER_DT
      if (self.stall_blip_hold_s >= _STALL_HOLD_S and self.stall_blip_count < _STALL_MAX_BLIPS
          and abs(self.path_angle_last) < _BLIP_MAX_PATH_ANGLE):
        self.stall_blip_frames_left = _STALL_BLIP_FRAMES
        self.stall_blip_hold_s = 0.0
        self.stall_blip_count += 1
    else:
      self.stall_blip_hold_s = 0.0
      if CS.out.steeringPressed or abs(gap) < 0.5 * _STALL_GAP_MIN:
        self.stall_blip_count = 0  # episode over: the car is tracking again, or the driver took it
