"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ford lead-aware longitudinal control, ported from BluePilot bp-7.0.

Layered on top of openpilot's own accel and gas, not a replacement for them: the planner still
decides, and this narrows what is sent to the car based on what the lead vehicle is doing.

  gaining    closing on the lead. Inside 1.5 s of headway, gas is cut to zero rather than being
             trimmed, because adding any throttle while closing is what makes the following
             brake harder a moment later.
  pacing     matched to the lead. Gas is capped, so the car holds station instead of surging.
  trailing   falling behind. Left alone.

Downward accel changes are rate limited so the first brake application eases in instead of
stomping, except when time-to-collision says that would be the wrong call. The brake and
pre-charge requests get separate hysteresis, so the brakes pre-charge slightly before they bite.

Deliberately limited to highway speeds (engages above 50 mph, drops below 45) because the lead
classification is only meaningful at steady cruise, and to leads that are themselves moving
above 40 mph.
"""
from collections import namedtuple

from numpy import clip

from opendbc.car.ford.values import CarControllerParams

LongitudinalResult = namedtuple('LongitudinalResult', [
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

# Separate hysteresis for the brake and pre-charge requests.
_BRAKE_ENGAGE = -0.14
_BRAKE_RELEASE = -0.06
_PRECHARGE_ENGAGE = -0.12
_PRECHARGE_RELEASE = -0.06


class LongitudinalExt:
  """Mixed into the Ford CarController. Owns every piece of follow-control state."""

  def __init__(self, CP, CP_SP):
    tuning = CP_SP.fordLongitudinalTuning
    self.follow_control = bool(tuning.followControl)
    self.downhill_compensation = bool(tuning.downhillCompensation)

    self.speed_allowed = False
    self.accel_last = 0.0
    self.brake_actuate_last = False

  def pitch_compensation(self, accel_due_to_pitch: float) -> float:
    """Pitch compensation the brake and pre-charge decisions are made against.

    With downhill compensation off, a negative (downhill) pitch is dropped, so the car does not
    pre-charge the brakes every time the road tips forward.
    """
    if not self.downhill_compensation and accel_due_to_pitch < 0:
      return 0.0
    return accel_due_to_pitch

  def update(self, CC, CC_SP, CS, op_accel, op_gas, accel_due_to_pitch) -> LongitudinalResult:
    """Narrow openpilot's accel and gas based on the lead, for one 50 Hz frame."""
    # Brake hysteresis on the planner's own command, used whenever follow control is not driving
    brake_actuate = self.brake_actuate_last
    accel_pitch_compensated = op_accel + accel_due_to_pitch
    if accel_pitch_compensated > _BRAKE_RELEASE or not CC.longActive:
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

      use_follow = (self.speed_allowed and CC.longActive
                    and not CS.out.gasPressed and not CS.out.brakePressed
                    and (lead is None or v_lead_mph > _MIN_LEAD_SPEED_MPH))

      if use_follow:
        accel, gas = self._follow_limits(lead, CS, op_accel, op_gas, accel_due_to_pitch)
        brake_actuate = accel < _BRAKE_ENGAGE
        precharge_actuate = accel < _PRECHARGE_ENGAGE
        follow_control_used = True

    self.accel_last = accel

    # The car must never be asked to brake and accelerate at once.
    if brake_actuate:
      gas = CarControllerParams.INACTIVE_GAS

    accel = float(clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    if gas != CarControllerParams.INACTIVE_GAS:
      gas = float(clip(gas, CarControllerParams.MIN_GAS, CarControllerParams.ACCEL_MAX))

    return LongitudinalResult(
      accel=accel,
      gas=gas,
      brake_actuate=brake_actuate,
      precharge_actuate=precharge_actuate,
      # The stock system uses this to preview the request; BluePilot pins it inactive so the
      # PCM acts on the real request alone.
      accel_pred=CarControllerParams.INACTIVE_GAS,
      follow_control_used=follow_control_used,
    )

  def _follow_limits(self, lead, CS, op_accel, op_gas, accel_due_to_pitch):
    """Gas and accel bounds for the current lead state."""
    v_ego = max(CS.out.vEgo, 0.5)

    headway_s = 999.0
    ttc_s = 120.0
    gas_max = op_gas
    accel_min, accel_max = op_accel, op_accel

    if lead is None:
      # Nothing ahead: the planner is free on gas, but there is nothing to accelerate toward
      # either, so hold accel at zero rather than letting it drift.
      accel_min = accel_max = 0.0
    else:
      d_rel = float(lead.dRel)
      v_rel = float(lead.vRel)
      if d_rel > 0:
        headway_s = float(clip(d_rel / v_ego, 0.0, 999.0))
        ttc_s = float(clip(d_rel / -v_rel, 0.2, 120.0)) if v_rel < 0 else 60.0

      if v_rel < -_V_REL_DEADBAND:                    # gaining on the lead
        gas_max = 0.0 if headway_s < _NO_GAS_HEADWAY_S else op_gas
      elif v_rel > _V_REL_DEADBAND:                   # trailing
        gas_max = op_gas
      else:                                           # pacing
        gas_max = _PACING_GAS_CAP + accel_due_to_pitch

    gas = float(clip(op_gas, min(0.0, gas_max), max(0.0, gas_max)))
    accel = float(clip(op_accel, accel_min, accel_max))

    # Ease the first brake application in, unless closing fast or already very close.
    if ttc_s > _TTC_BYPASS_S and headway_s > _HEADWAY_BYPASS_S:
      accel = max(accel, self.accel_last - _FOLLOW_ACCEL_ROC)

    return accel, gas
