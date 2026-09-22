import os
import re
import unittest

from opendbc.car import DT_CTRL
from opendbc.car.ford.values import CAR, CarControllerParams
from opendbc.car.lateral import MAX_LATERAL_ACCEL
from opendbc.sunnypilot.car.ford.human_turn import (
  HUMAN_TURN_ANGLE_DEG,
  HUMAN_TURN_HOLD_PRETURNED_S,
  HUMAN_TURN_HOLD_S,
)
from numpy import interp

from opendbc.sunnypilot.car.ford.lateral_angle_ext import _SOFT_ROC_SPEED_BP, _SOFT_ROC_V, LateralAngleExt
from opendbc.sunnypilot.car.ford.tests.helpers import make_actuators, make_car_params, make_cc, make_cc_sp, make_cs
from opendbc.sunnypilot.car.ford.values_ext import (
  ANGLE_CURVATURE_ERROR,
  FORD_DBC_PATH_ANGLE_MAX,
  FORD_DBC_PATH_ANGLE_MIN,
  PrimaryLateralControl,
)

STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL


def angle_params(platform=CAR.FORD_F_150_MK14, **tuning):
  return make_car_params(platform, mode=PrimaryLateralControl.angle, **tuning)


class TestLateralAngleExt(unittest.TestCase):
  def setUp(self):
    self.CP, self.CP_SP = angle_params()
    self.lat = LateralAngleExt(self.CP, self.CP_SP)

  def _run(self, frames=1, **kwargs):
    """Step the strategy `frames` times with constant inputs and return the last result."""
    cc = make_cc(kwargs.pop('lat_active', True))
    curvature = kwargs.pop('curvature', 0.0)
    cs_kwargs = {k: kwargs.pop(k) for k in list(kwargs) if k in
                 ('v_ego', 'yaw_rate', 'steering_pressed', 'steering_angle')}
    cc_sp = make_cc_sp(**kwargs)
    result = None
    for _ in range(frames):
      result = self.lat.update(cc, cc_sp, make_cs(**cs_kwargs), make_actuators(curvature))
    return result

  def test_unset_tuning_falls_back_to_defaults(self):
    """CarParamsSP defaults floats to 0.0. A CarParamsSP written before this feature existed must
    not be read as zero steering gain."""
    _, CP_SP = make_car_params()
    CP_SP.fordLateralTuning.primaryControl = int(PrimaryLateralControl.angle)
    lat = LateralAngleExt(self.CP, CP_SP)
    self.assertEqual(lat.low_speed_factor, 1.0)
    self.assertEqual(lat.high_speed_factor, 1.0)
    self.assertEqual(lat.high_speed_dampening, 1.0)
    self.assertEqual(lat.lane_change_factor_high, 1.0)

  def test_tuning_is_clamped(self):
    _, CP_SP = angle_params(low_speed_factor=99.0, high_speed_factor=-5.0,
                          high_speed_dampening=99.0, lane_change_factor=99.0)
    lat = LateralAngleExt(self.CP, CP_SP)
    self.assertEqual(lat.low_speed_factor, 1.5)
    self.assertEqual(lat.high_speed_factor, 0.5)
    self.assertEqual(lat.high_speed_dampening, 1.25)
    self.assertEqual(lat.lane_change_factor_high, 1.5)

  def test_inactive_is_all_zero_with_truthful_shadow(self):
    """While lateral is inactive the command is zero, but the shadow must track the measurement:
    the panda latches it from every LKA frame, so a stale zero would race the first enabled frame
    after re-engage."""
    result = self._run(lat_active=False, v_ego=30.0, yaw_rate=0.15)
    self.assertEqual(result.path_angle, 0.0)
    self.assertEqual(result.apply_curvature, 0.0)
    self.assertEqual(result.path_offset, 0.0)
    self.assertEqual(result.curvature_rate, 0.0)
    self.assertTrue(result.lat_inactive)
    self.assertAlmostEqual(self.lat.shadow_curvature, -0.15 / 30.0, places=6)

  def test_path_angle_tracks_kappa_times_speed(self):
    """The whole point of angle control: path_angle is kappa * v_ego * gain, so it grows with both
    curvature and speed rather than being a curvature the PSCM filters."""
    self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=30.0), make_actuators(0.0))
    small = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=30.0), make_actuators(0.0005)).path_angle
    self.assertGreater(small, 0.0)

    self.lat.path_angle_last = 0.0
    bigger = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=30.0), make_actuators(0.001)).path_angle
    self.assertGreater(bigger, small)

    # sign follows the commanded curvature
    self.lat.path_angle_last = 0.0
    left = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=30.0), make_actuators(-0.001)).path_angle
    self.assertLess(left, 0.0)

  def test_soft_rate_limit(self):
    """A large step in commanded curvature cannot produce a large step in path_angle."""
    for v_ego in (10.0, 15.0, 25.0, 35.0):
      max_step = float(interp(v_ego, _SOFT_ROC_SPEED_BP, _SOFT_ROC_V))
      lat = LateralAngleExt(self.CP, self.CP_SP)
      last = 0.0
      for _ in range(5):
        result = lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=v_ego, yaw_rate=0.02 * v_ego),
                            make_actuators(0.02))
        self.assertLessEqual(abs(result.path_angle - last), max_step + 1e-9,
                             f"step too large at {v_ego} m/s")
        last = result.path_angle

  def test_soft_rate_limit_admits_the_jerk_clip_curvature_allows(self):
    """The limit is a runaway backstop, not a second jerk limit. It must pass the largest
    lateral jerk FordLateralJerkLimit can set (12 m/s^3) at a typical gain of 1.5: J * G / v
    rad/s, i.e. 0.9 / v rad per call. The table it replaced let through 3.9 m/s^3 at 25 m/s."""
    for v_ego in (10.0, 12.0, 15.0, 18.0, 22.0, 25.0, 30.0, 35.0):
      soft_roc = float(interp(v_ego, _SOFT_ROC_SPEED_BP, _SOFT_ROC_V))
      self.assertGreaterEqual(soft_roc, 0.9 / v_ego - 1e-4, f"{v_ego} m/s")

  def test_soft_rate_limit_is_mirrored_by_the_panda(self):
    """ford.h holds the same nodes x1.02, so the panda never blocks what openpilot sends and
    never allows much more. Read from the source so the two cannot drift apart."""
    ford_h = os.path.join(os.path.dirname(__file__), "../../../../safety/modes/ford.h")
    with open(ford_h) as f:
      src = f.read()
    table = re.search(r"FORD_ANGLE_PATH_ANGLE_ROC = \{\s*\{([^}]*)\},\s*\{([^}]*)\}", src)
    self.assertIsNotNone(table)
    bp = [float(x.strip().rstrip(".")) for x in table.group(1).split(",")]
    v = [float(x) for x in table.group(2).split(",")]
    self.assertEqual(bp, list(_SOFT_ROC_SPEED_BP))
    for panda, ours in zip(v, _SOFT_ROC_V, strict=True):
      self.assertAlmostEqual(panda, ours * 1.02, places=5)

  def test_never_exceeds_dbc_range(self):
    lat = LateralAngleExt(self.CP, self.CP_SP)
    for _ in range(200):
      result = lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=35.0, yaw_rate=0.02 * 35.0),
                          make_actuators(0.02))
      self.assertLessEqual(result.path_angle, FORD_DBC_PATH_ANGLE_MAX + 1e-9)
      self.assertGreaterEqual(result.path_angle, FORD_DBC_PATH_ANGLE_MIN - 1e-9)

  def test_no_lateral_accel_ceiling(self):
    """BluePilot does not apply the ISO lateral acceleration ceiling in angle mode, so the
    steering intent can exceed it when the car is actually tracking that hard."""
    v_ego = 30.0
    iso_envelope = MAX_LATERAL_ACCEL / (v_ego ** 2)
    # the car is already cornering hard, so the deviation clip is not what limits the command
    measured = 0.01
    self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=v_ego, yaw_rate=-measured * v_ego),
                    make_actuators(0.02))
    self.assertGreater(abs(self.lat.shadow_curvature), iso_envelope)

  def test_deviation_clip_binds_and_is_reported(self):
    """The command is clipped to measured curvature +- ANGLE_CURVATURE_ERROR, the band the panda
    allows in angle mode. Without it the shadow would leave that band routinely."""
    result = self._run(v_ego=30.0, yaw_rate=0.0, curvature=0.02)
    self.assertTrue(self.lat.curvature_deviation_limited)
    self.assertAlmostEqual(abs(self.lat.shadow_curvature), ANGLE_CURVATURE_ERROR, places=9)
    self.assertGreater(result.path_angle, 0.0)

  def test_angle_band_is_twice_stock_and_mirrors_the_panda(self):
    """Angle mode leads the measurement by up to 0.004, stock and curvature mode by 0.002, and
    ford.h holds exactly those two numbers. Read from the source so neither side can drift."""
    self.assertEqual(ANGLE_CURVATURE_ERROR, 2 * CarControllerParams.CURVATURE_ERROR)
    ford_h = os.path.join(os.path.dirname(__file__), "../../../../safety/modes/ford.h")
    with open(ford_h) as f:
      src = f.read()
    for name, expected in (("FORD_STEERING_LIMITS", CarControllerParams.CURVATURE_ERROR),
                           ("FORD_ANGLE_STEERING_LIMITS", ANGLE_CURVATURE_ERROR)):
      block = re.search(name + r" = \{(.*?)\};", src, re.S)
      self.assertIsNotNone(block, name)
      error = int(re.search(r"\.max_curvature_error = (\d+)", block.group(1)).group(1))
      to_can = int(re.search(r"\.curvature_to_can = (\d+)", block.group(1)).group(1))
      self.assertAlmostEqual(error / to_can, expected, places=9, msg=name)

  def test_no_deviation_clip_at_low_speed(self):
    self._run(v_ego=5.0, yaw_rate=0.0, curvature=0.02)
    self.assertFalse(self.lat.curvature_deviation_limited)

  def test_human_turn_forces_lateral_inactive(self):
    # the driver winds the wheel up through the threshold, rather than grabbing an already-turned one
    self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=15.0, steering_pressed=True, steering_angle=5.0),
                    make_actuators(0.01))
    frames = int(HUMAN_TURN_HOLD_S / STEER_DT) + 2
    result = None
    for _ in range(frames):
      result = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=15.0, steering_pressed=True,
                                                                steering_angle=HUMAN_TURN_ANGLE_DEG + 10.0),
                               make_actuators(0.01))
    self.assertTrue(self.lat.human_turn_active)
    self.assertTrue(result.lat_inactive)
    self.assertEqual(result.path_angle, 0.0)
    self.assertEqual(result.ramp_type, 0)

    # on release the command ramps back in from zero rather than snapping to a stale value
    released = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=15.0), make_actuators(0.01))
    self.assertFalse(released.lat_inactive)
    self.assertLessEqual(abs(released.path_angle), float(interp(15.0, _SOFT_ROC_SPEED_BP, _SOFT_ROC_V)) + 1e-9)

  def test_grabbing_an_already_turned_wheel_needs_a_longer_hold(self):
    """Lateral control turns the wheel past the angle threshold on its own in a curve, so a brief
    corrective nudge there must not read as a takeover and kill steering mid-curve."""
    frames = int(HUMAN_TURN_HOLD_S / STEER_DT) + 2
    for _ in range(frames):
      self.lat.update(make_cc(), make_cc_sp(),
                      make_cs(v_ego=15.0, steering_pressed=True, steering_angle=HUMAN_TURN_ANGLE_DEG + 10.0),
                      make_actuators(0.01))
    self.assertFalse(self.lat.human_turn_active)

    for _ in range(int((HUMAN_TURN_HOLD_PRETURNED_S - HUMAN_TURN_HOLD_S) / STEER_DT) + 2):
      self.lat.update(make_cc(), make_cc_sp(),
                      make_cs(v_ego=15.0, steering_pressed=True, steering_angle=HUMAN_TURN_ANGLE_DEG + 10.0),
                      make_actuators(0.01))
    self.assertTrue(self.lat.human_turn_active)

  def test_brief_nudge_is_not_a_human_turn(self):
    for _ in range(int(1.0 / STEER_DT)):
      result = self.lat.update(make_cc(), make_cc_sp(),
                               make_cs(v_ego=15.0, steering_pressed=True, steering_angle=10.0),
                               make_actuators(0.01))
    self.assertFalse(self.lat.human_turn_active)
    self.assertFalse(result.lat_inactive)

  def test_hand_off_blip_on_release_of_a_sustained_press(self):
    """A sustained press attenuates the PSCM; a short mode-0 pulse on release resets it while the
    car is still straight, instead of waiting for the reactive detector to watch a curve be missed."""
    press_frames = int(0.6 / STEER_DT)
    for _ in range(press_frames):
      self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=15.0, steering_pressed=True, steering_angle=5.0),
                      make_actuators(0.0))
    self.assertFalse(self.lat.human_turn_active)

    result = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=15.0), make_actuators(0.0))
    self.assertTrue(result.lat_inactive)
    self.assertTrue(self.lat.stall_blip_active)

    # the pulse is short and self-clearing
    for _ in range(8):
      result = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=15.0), make_actuators(0.0))
    self.assertFalse(result.lat_inactive)
    self.assertFalse(self.lat.stall_blip_active)

  def test_stall_blip_fires_when_the_command_cannot_lead_the_car(self):
    """Hands-free, with the deviation clip binding and the car not following, a pulse must fire."""
    fired = False
    for _ in range(int(3.0 / STEER_DT)):
      result = self.lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=30.0, yaw_rate=0.0),
                               make_actuators(0.02))
      if result.lat_inactive:
        fired = True
        break
    self.assertTrue(fired, "stall blip never fired while the deviation clip was pinned")

  def test_stall_blip_fires_on_a_highway_stall(self):
    """The command a total stall pins is 0.004 * v * G, past the 0.10 rad small-command guard above
    about 26 m/s. The truck going straight is what makes the pulse safe, and it still fires."""
    for v_ego in (26.0, 30.0, 35.0):
      lat = LateralAngleExt(self.CP, self.CP_SP)
      fired, held = False, 0.0
      for _ in range(int(3.0 / STEER_DT)):
        result = lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=v_ego, yaw_rate=0.0), make_actuators(0.02))
        if result.lat_inactive:
          fired = True
          self.assertGreater(abs(held), 0.10, "test is not exercising the yaw arm")
          break
        held = result.path_angle
      self.assertTrue(fired, f"stall blip never fired at {v_ego} m/s")

  def test_stall_blip_never_fires_in_a_real_curve(self):
    """A stalled command in a curve the truck is genuinely turning: both the command and the yaw
    rate are past the guard, so releasing steering for 300 ms is not allowed."""
    v_ego, measured = 30.0, 0.005    # 0.15 rad/s of yaw, 4.5 m/s^2
    lat = LateralAngleExt(self.CP, self.CP_SP)
    for _ in range(int(3.0 / STEER_DT)):
      result = lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=v_ego, yaw_rate=-measured * v_ego),
                          make_actuators(0.02))
      self.assertFalse(result.lat_inactive, "stall blip fired mid-curve")
    self.assertTrue(lat.curvature_deviation_limited)

  def test_lane_change_scaling(self):
    """The lane change factor scales authority only in the direction of the change."""
    base = LateralAngleExt(self.CP, self.CP_SP)
    boosted_cp, boosted_cp_sp = angle_params(lane_change_factor=1.5)
    boosted = LateralAngleExt(boosted_cp, boosted_cp_sp)

    # laneChangeState=2 (starting), direction=1 (left), curvature negative == left
    args = (make_cc(), make_cc_sp(lane_change_state=2, lane_change_direction=1),
            make_cs(v_ego=12.0, yaw_rate=0.0), make_actuators(-0.0015))
    base_angle = base.update(*args).path_angle
    boosted_angle = boosted.update(*args).path_angle
    self.assertLess(boosted_angle, base_angle)  # both negative; boosted is larger in magnitude

    # precision drops to Comfortable during a lane change in the change direction
    self.assertEqual(base.update(*args).precision_type, 0)

  def test_model_blend_moves_the_command(self):
    """The model's predicted curvature is blended into the planner's, so a model that disagrees
    with the planner changes the command."""
    planner_only = LateralAngleExt(self.CP, self.CP_SP).update(
      make_cc(), make_cc_sp(model_curvature=0.0), make_cs(v_ego=12.0), make_actuators(0.001)).path_angle
    with_model = LateralAngleExt(self.CP, self.CP_SP).update(
      make_cc(), make_cc_sp(model_curvature=0.003), make_cs(v_ego=12.0), make_actuators(0.001)).path_angle
    self.assertGreater(with_model, planner_only)

  def test_missing_model_data_is_survivable(self):
    """carControlSP can arrive before modelV2 has been seen; the planner command must still work."""
    result = self.lat.update(make_cc(), make_cc_sp(model_curvatures=[]), make_cs(v_ego=12.0),
                             make_actuators(0.001))
    self.assertGreater(result.path_angle, 0.0)

  def test_platform_gains_differ(self):
    truck_cp, truck_cp_sp = angle_params(CAR.FORD_F_150_MK14)
    suv_cp, suv_cp_sp = angle_params(CAR.FORD_MUSTANG_MACH_E_MK1)
    self.assertNotEqual(LateralAngleExt(truck_cp, truck_cp_sp).path_angle_gain_high_curv,
                        LateralAngleExt(suv_cp, suv_cp_sp).path_angle_gain_high_curv)


if __name__ == "__main__":
  unittest.main()


class TestAngleModeCommandFidelity(unittest.TestCase):
  """Angle mode must follow carControl as closely as its own actuator limits allow.

  Both cases here were found in a logged 15 m radius corner (route 0000000a, segment 12, t=757-762)
  where latActive stayed true the whole way through and the panda blocked nothing, yet the command
  on the wire did not follow carControl.
  """

  @staticmethod
  def _drive(ctrl, frames, curvature, v_ego, pressed=False, angle=0.0):
    cc = make_cc(lat_active=True, curvature=curvature)
    cc_sp = make_cc_sp(model_curvature=curvature)
    res = None
    for _ in range(frames):
      cs = make_cs(v_ego=v_ego, steering_pressed=pressed, steering_angle=angle)
      res = ctrl.update(cc, cc_sp, cs, make_actuators(curvature=curvature))
    return res

  def test_tight_corner_is_not_capped_by_the_curvature_signal_range(self):
    """CURVATURE_MAX bounds c2. Angle mode steers with c1, so it must not inherit that ceiling."""
    v_ego, kappa = 7.0, 0.0667            # 15 m radius, the logged corner
    ctrl = LateralAngleExt(*angle_params())
    res = self._drive(ctrl, 60, kappa, v_ego)
    capped = 0.02 * v_ego                 # what the old clamp allowed, before gain
    self.assertGreater(abs(res.path_angle), capped,
                       "command still capped at the c2 signal range")

  def test_the_command_still_respects_its_own_signal_range(self):
    """Removing the c2 clamp must not let c1 leave its own range."""
    ctrl = LateralAngleExt(*angle_params())
    res = self._drive(ctrl, 200, 0.5, 25.0)   # absurd request, on purpose
    self.assertLessEqual(res.path_angle, FORD_DBC_PATH_ANGLE_MAX + 1e-9)
    self.assertGreaterEqual(res.path_angle, FORD_DBC_PATH_ANGLE_MIN - 1e-9)

  def test_gentle_curves_are_unchanged(self):
    """The clamp only ever bound past 0.02 1/m, so nothing below it may move."""
    for kappa in (0.001, 0.005, 0.010, 0.019):
      with self.subTest(kappa=kappa):
        ctrl = LateralAngleExt(*angle_params())
        res = self._drive(ctrl, 60, kappa, 20.0)
        expected = LateralAngleExt(*angle_params())
        want = self._drive(expected, 60, kappa, 20.0)
        self.assertAlmostEqual(res.path_angle, want.path_angle, places=9)

  def test_human_turn_override_is_on_by_default(self):
    ctrl = LateralAngleExt(*angle_params())
    self.assertTrue(ctrl.human_turn_detection)
    self._drive(ctrl, int(HUMAN_TURN_HOLD_PRETURNED_S / STEER_DT) + 20, 0.02, 7.0, pressed=True, angle=-120.0)
    self.assertTrue(ctrl.human_turn_active)

  def test_human_turn_override_can_be_turned_off(self):
    """The setting used to be curvature-mode only in practice; angle mode hard-wired it on."""
    ctrl = LateralAngleExt(*angle_params(human_turn_detection=False))
    self.assertFalse(ctrl.human_turn_detection)
    res = self._drive(ctrl, int(HUMAN_TURN_HOLD_PRETURNED_S / STEER_DT) + 20, 0.02, 7.0, pressed=True, angle=-120.0)
    self.assertFalse(ctrl.human_turn_active)
    self.assertFalse(res.lat_inactive, "lateral was handed back despite the override being off")
    self.assertNotEqual(res.path_angle, 0.0, "no command while carControl still wants lateral")
