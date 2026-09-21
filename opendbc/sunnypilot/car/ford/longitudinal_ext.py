"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ford lead-aware longitudinal control, ported from BluePilot bp-7.0.

Layered on top of openpilot's own accel and gas, not a replacement for them: the planner still
decides, and this narrows what is sent to the car based on what the lead vehicle is doing.

  gaining    closing on the lead. Inside 1.5 s of headway, gas is capped at zero, because
             adding any throttle while closing is what makes the following brake harder a
             moment later.
  pacing     matched to the lead. Gas is capped, so the car holds station instead of surging.
  trailing   falling behind. Left alone.

With no lead in sight none of this applies and the planner goes through untouched.

Every one of those is a cap and never a floor: a deceleration the planner asked for is passed
through untouched, whatever the lead is doing.

Downward accel changes are rate limited so the first brake application eases in instead of
stomping, except when time-to-collision says that would be the wrong call. The brake and
pre-charge requests get separate hysteresis, so the brakes pre-charge slightly before they bite.

This also owns the propulsion channel's inactive sentinel, which upstream applies in the
CarController. See _gas_request for why it has to be decided here and why it is hysteresised.

Deliberately limited to highway speeds (engages above 50 mph, drops below 45) because the lead
classification is only meaningful at steady cruise, and to leads that are themselves moving
above 40 mph.
"""
from collections import namedtuple

from numpy import clip

from opendbc.car.ford.values import CarControllerParams
from opendbc.sunnypilot.car.ford.values_ext import PEDAL_OVERRIDE_RANGE

LongitudinalResult = namedtuple('LongitudinalResult', [
  'acc_enabled',
  'accel',
  'gas',
  'brake_actuate',
  'precharge_actuate',
  'accel_pred',
  'follow_control_used',
])

MS_TO_MPH = 2.23694

# Speed band with hysteresis, so the feature does not flicker in and out around the threshold.
_ENGAGE_ABOVE_MPH = 50.0
_DISENGAGE_BELOW_MPH = 45.0
# A lead slower than this is urban traffic, not a highway lead; leave the planner alone.
_MIN_LEAD_SPEED_MPH = 40.0

# Lead classification deadband on relative speed (m/s).
_V_REL_DEADBAND = 0.1
# Inside this headway, a closing lead means no gas at all.
_NO_GAS_HEADWAY_S = 1.5
# Gas cap while pacing (m/s^2), offset by pitch so a hill does not read as surging.
_PACING_GAS_CAP = 0.2

# Max downward accel change per 50 Hz frame, to ease the first brake application in.
_FOLLOW_ACCEL_ROC = 0.002
# Above this time-to-collision, and outside this headway, easing in is safe. Below either, it is
# not: let the planner's braking through immediately.
_TTC_BYPASS_S = 8.0
_HEADWAY_BYPASS_S = 0.5
# And nor is it safe once the planner is asking for materially more than the ramp is delivering,
# however comfortable the lead looks.
#
# Those two bypasses key off the lead alone, so a brake request the lead did not cause, a speed
# limit, a curve, traffic further ahead, crawls out at _FOLLOW_ACCEL_ROC. Worse, the ramp and the
# CarController's own limiter feed each other: that limiter works from the value sent last frame,
# which is the eased one, so neither can get ahead of the other. Logged at 49 mph behind a lead
# 38 m away and closing at 2.4 m/s: the planner asked for -0.65 and the wire carried -0.04, and
# three seconds later it was asking -1.28 against -0.28. Follow control then dropped out, both
# limiters went with it, and the whole request landed in 400 ms at -2.05 m/s^2.
_FOLLOW_EASE_BYPASS = 0.20        # m/s^2 of shortfall before the ease-in gets out of the way

# Separate hysteresis for the brake and pre-charge requests.
#
# The brakes are asked for only once the propulsion channel has run out of room. Below
# MIN_GAS that channel releases to its inactive sentinel, which is a closed throttle, and on
# this platform a closed throttle measures -0.55 m/s^2 of overrun braking at 9-29 mph. So
# every request shallower than MIN_GAS is deliverable without the friction brakes, and asking
# for them there only lights the brake lamp at the car behind for deceleration the engine was
# going to produce anyway. BluePilot engages at -0.14, which is inside that band and is what
# made the lamp flash on and off while following.
#
# Release sits just shallower than where the propulsion channel resumes (_GAS_RELEASE), so
# propulsion is already carrying the request by the time the brakes let go.
_BRAKE_ENGAGE = -0.50
_BRAKE_RELEASE = -0.35
# Pre-charge leads the brakes so the calipers are up to pressure before they bite. It does not
# light the lamp on its own: over 236 s of logs the lamp was never lit by pre-charge alone.
_PRECHARGE_ENGAGE = -0.42
_PRECHARGE_RELEASE = -0.30

# Releasing the propulsion request to the inactive sentinel is a 4.5 m/s^2 step on the wire,
# so the threshold that does it gets hysteresis: a command hovering at MIN_GAS would otherwise
# toggle it at 50 Hz.
_GAS_RELEASE = -0.40
# Below this the creep compensation owns the accel request, and there is no propulsion worth
# asking for while the brakes bring the car to a stop.
_GAS_CREEP_SPEED = 1.0

# How a feathered pedal is told from a real press, and why the threshold is tunable rather
# than fixed, is documented on PEDAL_OVERRIDE_RANGE in values_ext.


def _tuned_pedal(value: float) -> float:
  """Clamp the stored threshold into range. A param that has never been written reads 0.0,
  which would mean every touch counts, so an unset one falls back to the default."""
  default, lo, hi = PEDAL_OVERRIDE_RANGE
  if value <= 0.0:
    return default
  return float(clip(value, lo, hi))


class LongitudinalExt:
  """Mixed into the Ford CarController. Owns every piece of follow-control state."""

  def __init__(self, CP, CP_SP):
    tuning = CP_SP.fordLongitudinalTuning
    self.follow_control = bool(tuning.followControl)
    self.downhill_compensation = bool(tuning.downhillCompensation)
    self.pedal_override_pc = _tuned_pedal(tuning.pedalOverrideThreshold)

    self.speed_allowed = False
    self.accel_last = 0.0
    self.brake_actuate_last = False
    self.gas_inactive_last = True

  def pitch_compensation(self, accel_due_to_pitch: float) -> float:
    """Pitch compensation the brake and pre-charge decisions are made against.

    With downhill compensation off, a negative (downhill) pitch is dropped, so the car does not
    pre-charge the brakes every time the road tips forward.
    """
    if not self.downhill_compensation and accel_due_to_pitch < 0:
      return 0.0
    return accel_due_to_pitch

  def _driver_overriding(self, CS) -> bool:
    """Whether the accelerator counts as a driver override.

    Against the pedal position where the CarState has it, so a feathered pedal does not take
    the whole ACCDATA inactive and cost the driver speed. gasPressed is the fallback for a
    CarState that does not carry the position.
    """
    pedal = getattr(CS, "accelerator_pedal_pc", None)
    if pedal is None:
      return bool(CS.out.gasPressed)
    return bool(CS.out.gasPressed) and pedal > self.pedal_override_pc

  def update(self, CC, CC_SP, CS, op_accel, op_gas, accel_due_to_pitch) -> LongitudinalResult:
    """Narrow openpilot's accel and gas based on the lead, for one 50 Hz frame."""
    overriding = self._driver_overriding(CS)
    # Brake hysteresis on the planner's own command, used whenever follow control is not driving
    brake_actuate = self.brake_actuate_last
    accel_pitch_compensated = op_accel + accel_due_to_pitch
    if accel_pitch_compensated > _BRAKE_RELEASE or not CC.longActive or overriding:
      brake_actuate = False
    elif accel_pitch_compensated < _BRAKE_ENGAGE:
      brake_actuate = True
    self.brake_actuate_last = brake_actuate

    accel, gas = op_accel, op_gas
    precharge_actuate = brake_actuate
    follow_control_used = False

    if self.follow_control:
      v_ego_mph = CS.out.vEgo * MS_TO_MPH
      if v_ego_mph > _ENGAGE_ABOVE_MPH:
        self.speed_allowed = True
      elif v_ego_mph < _DISENGAGE_BELOW_MPH:
        self.speed_allowed = False

      lead = CC_SP.leadOne if CC_SP.leadOne.status else None
      v_lead_mph = (lead.vLead * MS_TO_MPH) if lead else 0.0

      # Every limit here is defined against a lead, so without one there is nothing to apply
      # and the planner is passed straight through.
      use_follow = (self.speed_allowed and CC.longActive
                    and not overriding and not CS.out.brakePressed
                    and lead is not None and v_lead_mph > _MIN_LEAD_SPEED_MPH)

      if use_follow:
        accel, gas = self._follow_limits(lead, CS, op_accel, op_gas, accel_due_to_pitch)
        brake_actuate = accel < _BRAKE_ENGAGE
        precharge_actuate = accel < _PRECHARGE_ENGAGE
        follow_control_used = True

    # While the driver is on the accelerator this puts the whole ACCDATA back to its inactive
    # form, which is what the bus saw before the override could be held at all.
    #
    # The first version of that hold kept commanding propulsion through the press, on the
    # assumption that the PCM arbitrates it against the pedal. It does not. Engaging with the
    # pedal down puts CcStat_D_Actl into 5, override, and an active AccPrpl_A_Rq in that state
    # is refused: CmbbDeny_B_ActlPrpl goes to 1, CcStat_D_Actl to 2, and openpilot reads that
    # as accFaulted. Seen twice in one drive, 280 ms after engaging, once with a positive
    # request and once with a negative one, so it is the presence of a request and not its
    # magnitude or sign.
    #
    # What the hold is still worth is upstream of the wire: longitudinal control stays engaged,
    # so the long control state machine does not fall to off and reset its PID, and lifting off
    # resumes from what openpilot was already asking for instead of from zero.
    if overriding:
      brake_actuate = False
      precharge_actuate = False

    self.accel_last = accel

    # Cmbb_B_Enbl and AccResumEnbl_B_Rq. Cleared while the driver is on the accelerator, so the
    # whole message goes back to the inactive one the bus saw before the hold existed.
    acc_enabled = bool(CC.longActive) and not overriding

    # The car must never be asked to brake and accelerate at once. A mild negative request is
    # a throttle lift, not acceleration, so it is left alone.
    if brake_actuate:
      gas = min(gas, 0.0)

    accel = float(clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    gas = self._gas_request(CC, CS, gas, brake_actuate, overriding)

    # Clearing the enable bit is not enough on its own: AccBrkTot_A_Rq has to go with it.
    # Holding the accelerator override keeps the planner running, so a live brake total kept
    # reaching the wire under a cleared Cmbb_B_Enbl, and the module refuses that combination
    # the same way it refuses an active propulsion request. Both logged faults show it: enable
    # clear, AccPrpl at its sentinel, AccBrkTot still carrying +0.24 and +0.34, and
    # CmbbDeny_B_ActlPrpl going up 250 ms later. Zeroing it makes the message the same
    # inactive one the bus saw before any of this existed.
    if not acc_enabled:
      accel = 0.0

    return LongitudinalResult(
      acc_enabled=acc_enabled,
      accel=accel,
      gas=gas,
      brake_actuate=brake_actuate,
      precharge_actuate=precharge_actuate,
      # The stock system uses this to preview the request; BluePilot pins it inactive so the
      # PCM acts on the real request alone.
      accel_pred=CarControllerParams.INACTIVE_GAS,
      follow_control_used=follow_control_used,
    )

  def _gas_request(self, CC, CS, gas: float, brake_actuate: bool, overriding: bool) -> float:
    """The propulsion request as it goes on the wire, inactive sentinel included.

    AccPrpl_A_Rq can carry -5.0 (inactive: the PCM stops treating it as a request at all, so
    the engine falls back to a closed throttle) or [-0.5, +2.0]. There is nothing in between,
    so a request the planner puts below MIN_GAS has to be released to the sentinel and left to
    the brake channel.

    That release is a 4.5 m/s^2 step, and on an engine with as much overrun braking as a
    Raptor R's it is felt as a snatch, so two things guard it. It is hysteresised, because a
    planner command sitting on the limit would otherwise toggle it every frame. And it is not
    reached on the way down until the request really is below MIN_GAS: the gentle coast region
    is expressed through this channel rather than collapsed into full engine braking.

    Deciding it here rather than in the CarController is what makes that possible. Upstream
    substitutes the sentinel before the follow limits run, which leaves them clipping against
    -5.0 as if it were a real request.

    It is also the sentinel whenever the driver is on the accelerator: the PCM refuses an
    active request in that state. See update().
    """
    if not CC.longActive or overriding or (brake_actuate and CS.out.vEgo < _GAS_CREEP_SPEED):
      gas = CarControllerParams.INACTIVE_GAS
    elif self.gas_inactive_last:
      if gas < _GAS_RELEASE:
        gas = CarControllerParams.INACTIVE_GAS
    elif gas < CarControllerParams.MIN_GAS:
      gas = CarControllerParams.INACTIVE_GAS

    self.gas_inactive_last = gas == CarControllerParams.INACTIVE_GAS
    if gas != CarControllerParams.INACTIVE_GAS:
      gas = float(clip(gas, CarControllerParams.MIN_GAS, CarControllerParams.ACCEL_MAX))
    return gas

  def _follow_limits(self, lead, CS, op_accel, op_gas, accel_due_to_pitch):
    """Gas and accel bounds for the current lead state. Only called when there is a lead."""
    v_ego = max(CS.out.vEgo, 0.5)

    headway_s = 999.0
    ttc_s = 120.0
    gas_max = op_gas

    d_rel = float(lead.dRel)
    v_rel = float(lead.vRel)
    if d_rel > 0:
      headway_s = float(clip(d_rel / v_ego, 0.0, 999.0))
      ttc_s = float(clip(d_rel / -v_rel, 0.2, 120.0)) if v_rel < 0 else 60.0

    if v_rel < -_V_REL_DEADBAND:                      # gaining on the lead
      gas_max = 0.0 if headway_s < _NO_GAS_HEADWAY_S else op_gas
    elif v_rel > _V_REL_DEADBAND:                     # trailing
      gas_max = op_gas
    else:                                             # pacing
      gas_max = _PACING_GAS_CAP + accel_due_to_pitch

    gas = min(op_gas, gas_max)
    accel = op_accel

    # Ease the first brake application in, unless closing fast, already very close, or the
    # planner is asking for materially more than the ramp can hand over. op_gas is the planner's
    # own request before the creep compensation and the CarController's rate limit, which is why
    # the comparison is made against it rather than against op_accel: op_accel is already held
    # within one frame of the eased value and can never show the shortfall.
    behind = op_gas < self.accel_last - _FOLLOW_EASE_BYPASS
    if ttc_s > _TTC_BYPASS_S and headway_s > _HEADWAY_BYPASS_S and not behind:
      accel = max(accel, self.accel_last - _FOLLOW_ACCEL_ROC)

    return accel, gas
