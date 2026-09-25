"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest

import numpy as np

from opendbc.car import DT_CTRL
from opendbc.car.ford.values import CAR, CarControllerParams
from opendbc.car.lateral import MAX_LATERAL_JERK
from opendbc.sunnypilot.car.ford.human_turn import HUMAN_TURN_ANGLE_DEG, HUMAN_TURN_HOLD_S
from opendbc.sunnypilot.car.ford.lateral_curv_ext import LateralCurvExt
from opendbc.sunnypilot.car.ford.tests.helpers import make_actuators, make_car_params, make_cc, make_cc_sp, make_cs
from opendbc.sunnypilot.car.ford.values_ext import (
  CURVATURE_MAX,
  CURV_MODE_PATH_ANGLE_MAX,
  T_IDXS,
  PrimaryLateralControl,
)

STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL
LATERAL_FREQUENCY = 20  # Hz


def curv_params(platform=CAR.FORD_F_150_MK14, **tuning):
  return make_car_params(platform, mode=PrimaryLateralControl.curvature, **tuning)


# What safety/modes/ford.h accepts in this mode, with its 1 m/s speed fudge. BluePilot's table
# replaces the ISO lateral jerk envelope, and there is no acceleration ceiling below the DBC cap.
PANDA_ROC_BP = [5., 16., 25.]
PANDA_ROC_V = [0.0025, 0.0014, 0.00018]


def panda_max_curvature_step(speed):
  return float(np.interp(max(speed - 1.0, 1.0), PANDA_ROC_BP, PANDA_ROC_V))


def iso_max_curvature_step(speed):
  fudged = max(speed - 1.0, 1.0)
  return MAX_LATERAL_JERK / (fudged ** 2) / LATERAL_FREQUENCY


class TestLateralCurvExt(unittest.TestCase):
  def setUp(self):
    self.CP, self.CP_SP = curv_params()
    self.lat = LateralCurvExt(self.CP, self.CP_SP)

  def _step(self, lat=None, last=0.0, **kwargs):
    lat = lat if lat is not None else self.lat
    cc = make_cc(kwargs.pop('lat_active', True))
    curvature = kwargs.pop('curvature', 0.0)
    cs_keys = ('v_ego', 'yaw_rate', 'steering_pressed', 'steering_angle')
    cs_kwargs = {k: kwargs.pop(k) for k in list(kwargs) if k in cs_keys}
    return lat.update(cc, make_cc_sp(**kwargs), make_cs(**cs_kwargs), make_actuators(curvature), last)

  def test_inactive_is_all_zero(self):
    result = self._step(lat_active=False, v_ego=30.0, curvature=0.01)
    self.assertTrue(result.lat_inactive)
    self.assertEqual(result.apply_curvature, 0.0)
    self.assertEqual(result.path_angle, 0.0)
    self.assertEqual(result.curvature_rate, 0.0)
    self.assertEqual(result.ramp_type, 0)

  def test_all_four_signals_are_used(self):
    """The whole point of curvature mode: c2 steers, c1 trims, c3 anticipates. Only c0 is unused."""
    lat = LateralCurvExt(*curv_params(lane_positioning=True, custom_profile=1))
    last = 0.0
    result = None
    # c3 is deliberately limited to tight curves at low speed, which is the only place it helps,
    # so drive a tightening curve at 12 m/s
    for i in range(10):
      cc_sp = make_cc_sp(model_curvature=0.002 * (i + 1), model_position_y=[0.4] * len(T_IDXS))
      result = lat.update(make_cc(), cc_sp, make_cs(v_ego=12.0, yaw_rate=0.0), make_actuators(0.01), last)
      last = result.apply_curvature
    self.assertNotEqual(result.apply_curvature, 0.0)
    self.assertNotEqual(result.curvature_rate, 0.0)
    self.assertNotEqual(result.path_angle, 0.0)
    self.assertEqual(result.path_offset, 0.0)

  def test_path_offset_is_never_sent(self):
    """c0 and c1 fight each other on this platform, so c0 stays at its sentinel whatever the
    lane position works out to."""
    lat = LateralCurvExt(*curv_params(lane_positioning=True, lane_full_mode=True, path_offset=0.5))
    for _ in range(20):
      result = lat.update(make_cc(), make_cc_sp(model_position_y=[0.8] * len(T_IDXS)),
                          make_cs(v_ego=25.0), make_actuators(0.001), 0.0)
      self.assertEqual(result.path_offset, 0.0)

  def test_stays_inside_what_the_panda_accepts(self):
    """openpilot's wind-up table is the stricter of the two BluePilot ships, so the command
    always sits inside the symmetric table the panda enforces."""
    for v_ego in (10.0, 16.0, 20.0, 25.0, 30.0, 35.0):
      lat = LateralCurvExt(self.CP, self.CP_SP)
      last = 0.0
      for _ in range(100):
        result = lat.update(make_cc(), make_cc_sp(model_curvature=0.02),
                            make_cs(v_ego=v_ego, yaw_rate=0.02 * v_ego), make_actuators(0.02), last)
        self.assertLessEqual(abs(result.apply_curvature), CURVATURE_MAX + 1e-9,
                             f"DBC curvature range exceeded at {v_ego} m/s")
        self.assertLessEqual(abs(result.apply_curvature - last), panda_max_curvature_step(v_ego) + 1e-9,
                             f"panda rate limit exceeded at {v_ego} m/s")
        last = result.apply_curvature

  def test_uses_bluepilots_rate_table_not_the_iso_envelope(self):
    """The requested behavior: through the middle of the speed range the command moves faster
    than the ISO lateral jerk envelope would have allowed."""
    exceeded = False
    for v_ego in (12.0, 16.0, 20.0):
      lat = LateralCurvExt(self.CP, self.CP_SP)
      last = 0.0
      for _ in range(20):
        # measured tracks the command, so the deviation band is not what limits the ramp
        result = lat.update(make_cc(), make_cc_sp(model_curvature=0.02),
                            make_cs(v_ego=v_ego, yaw_rate=last * v_ego), make_actuators(0.02), last)
        if abs(result.apply_curvature - last) > iso_max_curvature_step(v_ego) + 1e-9:
          exceeded = True
        last = result.apply_curvature
    self.assertTrue(exceeded, "the command never exceeded the ISO jerk envelope it no longer respects")

  def test_deviation_clip_binds(self):
    self._step(v_ego=30.0, yaw_rate=0.0, curvature=0.02)
    self.assertTrue(self.lat.curvature_deviation_limited)

  def test_human_turn_hands_back(self):
    """A sustained manual turn drops to mode 0 rather than holding the mode active with zeroed
    signals, which is what needs a safety bypass in BluePilot."""
    self._step(v_ego=15.0, steering_pressed=True, steering_angle=5.0, curvature=0.01)
    result = None
    for _ in range(int(HUMAN_TURN_HOLD_S / STEER_DT) + 2):
      result = self._step(v_ego=15.0, steering_pressed=True,
                          steering_angle=HUMAN_TURN_ANGLE_DEG + 10.0, curvature=0.01)
    self.assertTrue(self.lat.human_turn_active)
    self.assertTrue(result.lat_inactive)

  def test_human_turn_detection_can_be_disabled(self):
    lat = LateralCurvExt(*curv_params(human_turn_detection=False))
    for _ in range(int(HUMAN_TURN_HOLD_S / STEER_DT) + 5):
      result = self._step(lat=lat, v_ego=15.0, steering_pressed=True,
                          steering_angle=HUMAN_TURN_ANGLE_DEG + 10.0, curvature=0.01)
    self.assertFalse(lat.human_turn_active)
    self.assertFalse(result.lat_inactive)

  def test_standstill_hands_back(self):
    result = self._step(v_ego=0.0, curvature=0.01)
    self.assertTrue(result.lat_inactive)

  def test_ramps_back_in_after_a_reset(self):
    """Coming off a reset the command starts from zero, so the panda's rate limit is never the
    thing that has to catch it."""
    self._step(v_ego=0.0, curvature=0.01)          # standstill reset
    result = self._step(v_ego=20.0, curvature=0.01, last=0.02)
    self.assertTrue(self.lat.post_reset_ramp_active)
    self.assertLessEqual(abs(result.apply_curvature), panda_max_curvature_step(20.0) + 1e-9)

  def test_curvature_rate_zeroed_during_a_lane_change(self):
    lat = LateralCurvExt(self.CP, self.CP_SP)
    last = 0.0
    for i in range(10):
      result = lat.update(make_cc(), make_cc_sp(model_curvature=-0.002 * (i + 1), lane_change_state=2,
                                                lane_change_direction=1),
                          make_cs(v_ego=12.0), make_actuators(-0.01), last)
      last = result.apply_curvature
    self.assertEqual(result.curvature_rate, 0.0)
    self.assertEqual(result.precision_type, 0)  # Comfortable

  def test_curvature_rate_is_limited_to_tight_low_speed_curves(self):
    """c3 contributes sub-millimetre at the lookahead the PSCM uses, so BluePilot only sends it
    where it helps: a tight curve below ~15 m/s."""
    for v_ego, expect_rate in ((12.0, True), (25.0, False)):
      lat = LateralCurvExt(self.CP, self.CP_SP)
      last = 0.0
      for i in range(10):
        result = lat.update(make_cc(), make_cc_sp(model_curvature=0.002 * (i + 1)),
                            make_cs(v_ego=v_ego), make_actuators(0.01), last)
        last = result.apply_curvature
      self.assertEqual(expect_rate, result.curvature_rate != 0.0, f"at {v_ego} m/s")

  def test_lane_positioning_off_means_no_trim(self):
    lat = LateralCurvExt(*curv_params(lane_positioning=False))
    for _ in range(20):
      result = lat.update(make_cc(), make_cc_sp(model_position_y=[1.0] * len(T_IDXS)),
                          make_cs(v_ego=25.0), make_actuators(0.001), 0.0)
    self.assertEqual(result.path_angle, 0.0)

  def test_trim_is_rate_limited_and_capped(self):
    lat = LateralCurvExt(*curv_params(lane_positioning=True, custom_profile=1, lane_positioning_gain=20.0))
    last = 0.0
    for _ in range(400):
      result = lat.update(make_cc(), make_cc_sp(model_position_y=[5.0] * len(T_IDXS)),
                          make_cs(v_ego=25.0), make_actuators(0.0), 0.0)
      self.assertLessEqual(abs(result.path_angle - last), 0.002 + 1e-9)  # panda mirror at 25 m/s
      self.assertLessEqual(abs(result.path_angle), CURV_MODE_PATH_ANGLE_MAX + 1e-9)
      last = result.path_angle

  def test_custom_profile_off_uses_platform_defaults(self):
    lat = LateralCurvExt(*curv_params(custom_profile=0, blend_ratio_low=0.9, blend_ratio_high=0.9,
                                      lane_positioning_gain=19.0))
    self.assertEqual(lat.blend_ratio_low, 0.4)
    self.assertEqual(lat.blend_ratio_high, 0.4)
    self.assertEqual(lat.lane_positioning_gain, 3.0)

  def test_custom_profile_on_uses_the_tuning(self):
    lat = LateralCurvExt(*curv_params(custom_profile=1, blend_ratio_low=0.9, blend_ratio_high=0.1,
                                      lane_positioning_gain=19.0))
    self.assertEqual(lat.blend_ratio_low, 0.9)
    self.assertAlmostEqual(lat.blend_ratio_high, 0.1)
    self.assertEqual(lat.lane_positioning_gain, 19.0)

  def test_unset_tuning_falls_back_to_defaults(self):
    """CarParamsSP defaults every float to 0.0, so a CarParamsSP written before this feature
    existed must not be read as a zeroed tuning value."""
    _, CP_SP = make_car_params()
    CP_SP.fordLateralTuning.primaryControl = int(PrimaryLateralControl.curvature)
    lat = LateralCurvExt(self.CP, CP_SP)
    self.assertEqual(lat.lane_change_factor_high, 0.85)
    self.assertEqual(lat.blend_ratio_low, 0.4)
    self.assertEqual(lat.lane_positioning_gain, 3.0)

  def test_path_offset_trim_shifts_lane_position(self):
    """Zero is a legitimate in-lane offset, so it must not be treated as unset."""
    lat = LateralCurvExt(*curv_params(path_offset=0.0, lane_positioning=True))
    self.assertEqual(lat.path_offset_trim, 0.0)
    shifted = LateralCurvExt(*curv_params(path_offset=-0.4, lane_positioning=True))
    self.assertAlmostEqual(shifted.path_offset_trim, -0.4)

  def test_missing_model_data_is_survivable(self):
    result = self._step(v_ego=20.0, curvature=0.001, model_curvatures=[], model_position_y=[])
    self.assertNotEqual(result.apply_curvature, 0.0)
    self.assertTrue(np.isfinite(result.path_angle))


if __name__ == "__main__":
  unittest.main()
