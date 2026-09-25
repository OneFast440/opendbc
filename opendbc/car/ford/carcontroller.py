import math
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX
from opendbc.sunnypilot.car.ford import fordcan_ext
from opendbc.sunnypilot.car.ford.hud_ext import HudExt
from opendbc.sunnypilot.car.ford.icbm import IntelligentCruiseButtonManagementInterface
from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt
from opendbc.sunnypilot.car.ford.lateral_curv_ext import LateralCurvExt
from opendbc.sunnypilot.car.ford.longitudinal_ext import LongitudinalExt
from opendbc.sunnypilot.car.ford.values_ext import PrimaryLateralControl

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

def anti_overshoot(apply_curvature, apply_curvature_last, v_ego):
  diff = 0.1
  tau = 5  # 5s smooths over the overshoot
  dt = DT_CTRL * CarControllerParams.STEER_STEP
  alpha = 1 - np.exp(-dt / tau)

  lataccel = apply_curvature * (v_ego ** 2)
  last_lataccel = apply_curvature_last * (v_ego ** 2)
  last_lataccel = apply_hysteresis(lataccel, last_lataccel, diff)
  last_lataccel = alpha * lataccel + (1 - alpha) * last_lataccel

  output_curvature = last_lataccel / (max(v_ego, 1) ** 2)

  return float(np.interp(v_ego, [5, 10], [apply_curvature, output_curvature]))


def apply_creep_compensation(accel: float, v_ego: float) -> float:
  creep_accel = np.interp(v_ego, [1., 3.], [0.6, 0.])
  creep_accel = np.interp(accel, [0., 0.2], [creep_accel, 0.])
  accel -= creep_accel
  return float(accel)


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.CAN = fordcan.CanBus(CP)

    self.apply_curvature_last = 0
    self.anti_overshoot_curvature_last = 0

    # sunnypilot: which signal steers the car. Selected once at car init, because the matching
    # panda safety mode is set from the same read. See opendbc/sunnypilot/car/ford/values_ext.py.
    self.lateral_mode = PrimaryLateralControl(CP_SP.fordLateralTuning.primaryControl)
    self.lat_angle = LateralAngleExt(CP, CP_SP) if self.lateral_mode == PrimaryLateralControl.angle else None
    self.lat_curv = LateralCurvExt(CP, CP_SP) if self.lateral_mode == PrimaryLateralControl.curvature else None
    self.long_ext = LongitudinalExt(CP, CP_SP)
    self.hud = HudExt(CP, CP_SP)

    self.accel = 0.0
    self.gas = 0.0
    self.last_button_frame = 0

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))

    # sunnypilot: move the stock cruise setpoint by pressing the wheel's own buttons
    icbm_sends, self.last_button_frame = IntelligentCruiseButtonManagementInterface.update(
      self, CC_SP, CS, self.packer, self.CAN, self.frame, self.last_button_frame)
    can_sends.extend(icbm_sends)

    ### lateral control ###
    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0:
      if self.lateral_mode == PrimaryLateralControl.stock:
        can_sends.append(self._stock_lateral_msg(CC, CS, actuators))
      else:
        can_sends.append(self._bluepilot_lateral_msg(CC, CC_SP, CS, actuators))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      if self.lat_angle is not None:
        # Carries angle-mode state to safety/modes/ford.h in bits no DBC signal maps to. Sent
        # whenever angle mode is configured, not only while engaged: the panda latches the shadow
        # curvature from every one of these frames, so a stale value here would race the first
        # enabled LMC frame after re-engage. Negated into the CAN sign convention, same as
        # path_angle and curvature.
        can_sends.append(fordcan_ext.create_lka_msg(self.packer, self.CAN, True, -self.lat_angle.shadow_curvature))
      else:
        can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      can_sends.append(self._long_msg(CC, CC_SP, CS, actuators))

    ### ui ###
    can_sends.extend(self.hud.update(CC, CC_SP, CS, hud_control, main_on, fcw_alert,
                                     self.frame, self.packer, self.CAN, self.CP))

    new_actuators = actuators.as_builder()
    new_actuators.curvature = self._reported_curvature()
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends

  def _reported_curvature(self) -> float:
    # Angle mode pins the curvature signal at zero on the wire, so report the curvature
    # path_angle was actually derived from; otherwise logs show a flat zero for the whole drive.
    if self.lat_angle is not None:
      return float(self.lat_angle.shadow_curvature)
    return float(self.apply_curvature_last)

  def _stock_lateral_msg(self, CC, CS, actuators):
    """Upstream openpilot: curvature only, every other signal at its inactive sentinel."""
    # Bronco and some other cars consistently overshoot curv requests
    # Apply some deadzone + smoothing convergence to avoid oscillations
    if self.CP.carFingerprint in (CAR.FORD_BRONCO_SPORT_MK1, CAR.FORD_F_150_MK14):
      self.anti_overshoot_curvature_last = anti_overshoot(actuators.curvature, self.anti_overshoot_curvature_last, CS.out.vEgoRaw)
      apply_curvature = self.anti_overshoot_curvature_last
    else:
      apply_curvature = actuators.curvature

    # apply rate limits, curvature error limit, and clip to signal range
    current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)
    # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
    if CS.out.vEgoRaw > 9:
      apply_curvature = float(np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                                      current_curvature + CarControllerParams.CURVATURE_ERROR))
    apply_curvature = CarControllerParams.CURVATURE_LIMITS.apply_limits(apply_curvature, self.apply_curvature_last, CS.out.vEgoRaw,
                                                                        0., CC.latActive, CarControllerParams.STEER_STEP)
    self.apply_curvature_last = apply_curvature

    if self.CP.flags & FordFlags.CANFD:
      # Ford uses four individual signals to dictate how to drive to the car. Curvature alone (limited to 0.02 m^-1)
      # can actuate the steering for a large portion of any lateral movements. However, in order to get further control on
      # steer actuation, the other three signals are necessary. Ford controls vehicles differently than most other makes.
      # A detailed explanation on ford control can be found here:
      # https://www.f150gen14.com/forum/threads/introducing-bluepilot-a-ford-specific-fork-for-comma3x-openpilot.24241/#post-457706
      mode = 1 if CC.latActive else 0
      counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
      return fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0., 0., -self.apply_curvature_last, 0., counter)
    return fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, 0., 0., -self.apply_curvature_last, 0.)

  def _bluepilot_lateral_msg(self, CC, CC_SP, CS, actuators):
    """BluePilot: curvature-primary uses all four polynomial signals, angle-primary uses c1 alone."""
    if self.lat_angle is not None:
      lat = self.lat_angle.update(CC, CC_SP, CS, actuators)
    else:
      lat = self.lat_curv.update(CC, CC_SP, CS, actuators, self.apply_curvature_last)
    self.apply_curvature_last = lat.apply_curvature

    # Both strategies hand lateral back to the driver by dropping the message to mode 0 rather
    # than freezing a command the PSCM has to reconcile later: on a manual turn, at a standstill,
    # and during angle mode's stall blip. Every check in safety/modes/ford.h has a legitimate
    # !steer_control_enabled branch, so those frames need no safety bypass.
    lat_active = CC.latActive and not lat.lat_inactive

    if self.CP.flags & FordFlags.CANFD:
      mode = 1 if lat_active else 0
      counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
      return fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, -lat.path_offset, -lat.path_angle,
                                         -lat.apply_curvature, -lat.curvature_rate, counter,
                                         lat.ramp_type, lat.precision_type)
    return fordcan.create_lat_ctl_msg(self.packer, self.CAN, lat_active, -lat.path_offset, -lat.path_angle,
                                      -lat.apply_curvature, -lat.curvature_rate,
                                      lat.ramp_type, lat.precision_type)

  def _long_msg(self, CC, CC_SP, CS, actuators):
    accel = actuators.accel
    gas = accel

    if CC.longActive:
      # Compensate for engine creep at low speed.
      # Either the ABS does not account for engine creep, or the correction is very slow
      # TODO: verify this applies to EV/hybrid
      accel = apply_creep_compensation(accel, CS.out.vEgo)

      # The stock system has been seen rate limiting the brake accel to 5 m/s^3,
      # however even 3.5 m/s^3 causes some overshoot with a step response.
      accel = max(accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))

    accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    gas = float(np.clip(gas, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

    # Both gas and accel are in m/s^2, accel is used solely for braking
    if not CC.longActive or gas < CarControllerParams.MIN_GAS:
      gas = CarControllerParams.INACTIVE_GAS

    # PCM applies pitch compensation to gas/accel, but we need to compensate for the brake/pre-charge bits
    accel_due_to_pitch = 0.0
    if len(CC.orientationNED) == 3:
      accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY
    accel_due_to_pitch = self.long_ext.pitch_compensation(accel_due_to_pitch)

    # sunnypilot: narrow the planner's command based on what the lead is doing, and give the
    # brake and pre-charge requests their own hysteresis
    lng = self.long_ext.update(CC, CC_SP, CS, accel, gas, accel_due_to_pitch)
    self.accel = lng.accel
    self.gas = lng.gas

    stopping = actuators.longControlState == LongCtrlState.stopping
    # TODO: look into using the actuators packet to send the desired speed
    return fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, lng.gas, lng.accel, stopping,
                                  lng.brake_actuate, v_ego_kph=V_CRUISE_MAX,
                                  precharge_request=lng.precharge_actuate, accel_pred=lng.accel_pred)
