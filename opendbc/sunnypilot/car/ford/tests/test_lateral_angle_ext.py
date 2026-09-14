import unittest

from opendbc.car import DT_CTRL
from opendbc.car.ford.values import CAR, CarControllerParams
from opendbc.car.lateral import MAX_LATERAL_ACCEL
from opendbc.sunnypilot.car.ford.human_turn import (
  HUMAN_TURN_ANGLE_DEG,
  HUMAN_TURN_HOLD_PRETURNED_S,
  HUMAN_TURN_HOLD_S,
)
from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt
from opendbc.sunnypilot.car.ford.tests.helpers import make_actuators, make_car_params, make_cc, make_cc_sp, make_cs
from opendbc.sunnypilot.car.ford.values_ext import (
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
    for v_ego, max_step in ((10.0, 0.055), (15.0, 0.0425), (25.0, 0.009)):
      lat = LateralAngleExt(self.CP, self.CP_SP)
      last = 0.0
      for _ in range(5):
        result = lat.update(make_cc(), make_cc_sp(), make_cs(v_ego=v_ego, yaw_rate=0.02 * v_ego),
                            make_actuators(0.02))
        self.assertLessEqual(abs(result.path_angle - last), max_step + 1e-9,
                             f"step too large at {v_ego} m/s")
        last = result.path_angle

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
    """The command is clipped to measured curvature +- CURVATURE_ERROR, the same clip the stock
    curvature path applies. Without it the shadow would leave the panda's error band routinely."""
    result = self._run(v_ego=30.0, yaw_rate=0.0, curvature=0.02)
    self.assertTrue(self.lat.curvature_deviation_limited)
    self.assertLessEqual(abs(self.lat.shadow_curvature), CarControllerParams.CURVATURE_ERROR + 1e-9)
    self.assertGreater(result.path_angle, 0.0)

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
    self.assertLessEqual(abs(released.path_angle), 0.0425 + 1e-9)

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
