"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Delivered-vs-commanded saturation observer (PSCM firmware handoff, Improvement B).
"""
import unittest

from opendbc.car.ford.values import CAR
from opendbc.sunnypilot.car.ford.lateral_angle_ext import (
  _SAT_OBS_ENTER,
  _SAT_OBS_EXIT,
  _SAT_OBS_LAG_FRAMES,
  _SAT_OBS_MAX_HOLD_S,
  _SAT_OBS_MIN_KAPPA,
  _SAT_OBS_MIN_SPEED,
  _STEER_DT,
  LateralAngleExt,
)
from opendbc.sunnypilot.car.ford.tests.helpers import make_actuators, make_car_params, make_cc, make_cc_sp, make_cs
from opendbc.sunnypilot.car.ford.values_ext import PrimaryLateralControl

V = 20.0          # m/s, above the speed floor and above the deviation clip's 9 m/s
KAPPA = 0.008     # 1/m, a real curve; chosen so a 0.625 delivery ratio keeps the
DELIVERED = 0.005  # desired-vs-measured gap under the stall detector's threshold
ARM_FRAMES = 60   # filter settle + debounce is ~26 frames; well short of the max hold


def angle_ctrl(sat_observer=True, **tuning):
  CP, CP_SP = make_car_params(CAR.FORD_F_150_MK14, mode=PrimaryLateralControl.angle,
                              sat_observer=sat_observer, **tuning)
  return LateralAngleExt(CP, CP_SP)


def yaw_for(curvature, v_ego=V):
  """CarState.yawRate that get_current_curvature() reads back as `curvature`."""
  return -curvature * v_ego


def drive(ctrl, frames, requested, delivered, v_ego=V, **cs_kwargs):
  """Hold a curve. `requested` goes to both the planner and the model so the blend is a no-op,
  which keeps these tests about the observer rather than about the blend."""
  cc = make_cc(lat_active=True, curvature=requested)
  cc_sp = make_cc_sp(model_curvature=requested)
  for _ in range(frames):
    cs = make_cs(v_ego=v_ego, yaw_rate=yaw_for(delivered, v_ego), **cs_kwargs)
    ctrl.update(cc, cc_sp, cs, make_actuators(curvature=requested))
  return ctrl


def attenuating(**kwargs):
  """A controller that has watched a sustained under-delivery and armed the flag."""
  ctrl = angle_ctrl(**kwargs)
  drive(ctrl, ARM_FRAMES, KAPPA, DELIVERED)
  return ctrl


class TestSaturationObserver(unittest.TestCase):
  """Fire on real attenuation, stay quiet on everything else.

  The attempt this replaces (angleState.saturated) fired on ordinary curve-entry lag and froze the
  command, so most of what follows is about the cases where it must NOT fire.
  """

  def test_off_by_default(self):
    CP, CP_SP = make_car_params(CAR.FORD_F_150_MK14, mode=PrimaryLateralControl.angle)
    self.assertFalse(CP_SP.fordLateralTuning.satObserver)
    self.assertFalse(LateralAngleExt(CP, CP_SP).sat_observer_enabled)

  def test_fires_on_sustained_attenuation(self):
    ctrl = attenuating()
    self.assertLess(ctrl.delivery_ratio, _SAT_OBS_ENTER)
    self.assertTrue(ctrl.pscm_attenuating)

  def test_quiet_when_delivery_is_healthy(self):
    ctrl = drive(angle_ctrl(), ARM_FRAMES, KAPPA, KAPPA * 0.95)
    self.assertGreater(ctrl.delivery_ratio, _SAT_OBS_EXIT)
    self.assertFalse(ctrl.pscm_attenuating)

  def test_does_not_fire_before_the_debounce(self):
    ctrl = drive(angle_ctrl(), _SAT_OBS_LAG_FRAMES + 2, KAPPA, 0.0)
    self.assertFalse(ctrl.pscm_attenuating)

  def test_curve_entry_alone_does_not_trip_it(self):
    """The exact failure mode of the old proxy: request ramping up, car lagging behind it."""
    ctrl = angle_ctrl()
    history = [0.0] * _SAT_OBS_LAG_FRAMES
    requested = 0.0
    for _ in range(80):                 # 4 s of continuous entry
      requested = min(KAPPA, requested + 0.0002)
      delivered = history.pop(0)
      history.append(requested)
      cc = make_cc(lat_active=True, curvature=requested)
      cc_sp = make_cc_sp(model_curvature=requested)
      ctrl.update(cc, cc_sp, make_cs(v_ego=V, yaw_rate=yaw_for(delivered)),
                  make_actuators(curvature=requested))
      self.assertFalse(ctrl.pscm_attenuating, "entry lag was read as attenuation")

  def test_disabled_observer_never_flags(self):
    ctrl = drive(angle_ctrl(sat_observer=False), ARM_FRAMES, KAPPA, 0.0)
    self.assertFalse(ctrl.pscm_attenuating)

  def test_gated_while_the_driver_steers(self):
    ctrl = drive(angle_ctrl(), ARM_FRAMES, KAPPA, DELIVERED, steering_pressed=True)
    self.assertFalse(ctrl.pscm_attenuating)

  def test_gated_below_the_speed_floor(self):
    ctrl = drive(angle_ctrl(), ARM_FRAMES, KAPPA, DELIVERED, v_ego=_SAT_OBS_MIN_SPEED - 1.0)
    self.assertFalse(ctrl.pscm_attenuating)

  def test_gated_on_a_straight(self):
    """Below the curvature floor the ratio is noise, not a delivery measurement."""
    tiny = _SAT_OBS_MIN_KAPPA / 4
    ctrl = drive(angle_ctrl(), ARM_FRAMES, tiny, 0.0)
    self.assertFalse(ctrl.pscm_attenuating)

  def test_opposite_sign_is_ignored(self):
    """Counter-steer makes the ratio negative; that is not a delivery measurement either."""
    ctrl = drive(angle_ctrl(), ARM_FRAMES, KAPPA, -DELIVERED)
    self.assertGreater(ctrl.delivery_ratio, _SAT_OBS_ENTER)
    self.assertFalse(ctrl.pscm_attenuating)

  def test_clears_on_recovery_with_hysteresis(self):
    ctrl = attenuating()
    self.assertTrue(ctrl.pscm_attenuating)
    drive(ctrl, ARM_FRAMES, KAPPA, KAPPA)
    self.assertFalse(ctrl.pscm_attenuating)

  def test_cannot_latch_forever(self):
    """A ratio that stays low longer than any real corner lets go anyway, and has to re-earn it.

    It does re-arm afterwards, which is intended: a genuinely attenuating curve should still be
    flagged. What must not happen is the flag holding indefinitely on a ratio that is low for some
    reason this observer has no business freezing the command over.
    """
    ctrl = attenuating()
    self.assertTrue(ctrl.pscm_attenuating)

    released_after = None
    for frame in range(int(_SAT_OBS_MAX_HOLD_S / _STEER_DT) + 20):
      drive(ctrl, 1, KAPPA, DELIVERED)
      if not ctrl.pscm_attenuating:
        released_after = frame
        break

    self.assertIsNotNone(released_after, "flag never let go")
    self.assertLessEqual(ctrl.sat_obs_hold_s, _SAT_OBS_MAX_HOLD_S)
    self.assertGreater(ctrl.sat_obs_refractory_s, 0.0, "released without a refractory period")

    # and the refractory really does block an immediate re-arm
    drive(ctrl, 1, KAPPA, DELIVERED)
    self.assertFalse(ctrl.pscm_attenuating)

  def test_reset_clears_observer_state(self):
    ctrl = attenuating()
    self.assertTrue(ctrl.pscm_attenuating)
    # lateral going inactive puts mode 0 on the wire, which resets the PSCM's authority
    ctrl.update(make_cc(lat_active=False), make_cc_sp(), make_cs(v_ego=V), make_actuators())
    self.assertFalse(ctrl.pscm_attenuating)
    self.assertEqual(len(ctrl.kappa_req_history), 0)

  def test_flag_blocks_magnitude_growth(self):
    """What the flag is for: stop commanding harder into a curve the module is refusing."""
    ctrl = attenuating()
    self.assertTrue(ctrl.pscm_attenuating)
    held = ctrl.path_angle_last
    drive(ctrl, 5, KAPPA + 0.003, DELIVERED + 0.0025)
    self.assertLessEqual(abs(ctrl.path_angle_last), abs(held) + 1e-9)

  def test_the_same_drive_without_the_flag_keeps_winding_up(self):
    """The control: identical input, observer off, command grows."""
    ctrl = angle_ctrl(sat_observer=False)
    drive(ctrl, ARM_FRAMES, KAPPA, DELIVERED)
    held = ctrl.path_angle_last
    drive(ctrl, 5, KAPPA + 0.003, DELIVERED + 0.0025)
    self.assertGreater(abs(ctrl.path_angle_last), abs(held))

  def test_the_stall_blip_is_not_triggered_by_these_scenarios(self):
    """Guard the test design itself: the blip would reset the observer and mask a regression."""
    ctrl = attenuating()
    self.assertEqual(ctrl.stall_blip_count, 0)
    self.assertFalse(ctrl.stall_blip_active)


if __name__ == "__main__":
  unittest.main()
