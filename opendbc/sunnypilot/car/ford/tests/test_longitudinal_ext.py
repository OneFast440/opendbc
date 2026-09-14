"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest

from opendbc.car.ford.values import CarControllerParams
from opendbc.sunnypilot.car.ford.longitudinal_ext import MS_TO_MPH, LongitudinalExt
from opendbc.sunnypilot.car.ford.tests.helpers import make_car_params, make_cc, make_cc_sp, make_cs, make_lead

HIGHWAY_MS = 60.0 / MS_TO_MPH
URBAN_MS = 30.0 / MS_TO_MPH


class TestLongitudinalExt(unittest.TestCase):
  def _build(self, **tuning):
    return LongitudinalExt(*make_car_params(**tuning))

  @staticmethod
  def _step(lng, op_accel=0.0, op_gas=0.0, pitch=0.0, v_ego=HIGHWAY_MS, lead=None,
            long_active=True, gas_pressed=False, brake_pressed=False):
    return lng.update(make_cc(long_active=long_active), make_cc_sp(lead=lead),
                      make_cs(v_ego=v_ego, gas_pressed=gas_pressed, brake_pressed=brake_pressed),
                      op_accel, op_gas, pitch)

  def _settle_speed(self, lng, v_ego=HIGHWAY_MS, lead=None):
    """Cross the engage threshold so the speed band latches on."""
    for _ in range(3):
      self._step(lng, v_ego=v_ego, lead=lead)

  def test_disabled_passes_the_planner_through(self):
    lng = self._build(follow_control=False)
    lead = make_lead(status=True, d_rel=10.0, v_rel=-5.0, v_lead=HIGHWAY_MS)
    result = self._step(lng, op_accel=0.3, op_gas=0.3, lead=lead)
    self.assertAlmostEqual(result.accel, 0.3)
    self.assertAlmostEqual(result.gas, 0.3)
    self.assertFalse(result.follow_control_used)

  def test_urban_speed_is_left_alone(self):
    """The lead classification only means anything at steady cruise."""
    lng = self._build()
    lead = make_lead(status=True, d_rel=10.0, v_rel=-5.0, v_lead=HIGHWAY_MS)
    result = self._step(lng, op_accel=0.3, op_gas=0.3, v_ego=URBAN_MS, lead=lead)
    self.assertFalse(result.follow_control_used)
    self.assertAlmostEqual(result.gas, 0.3)

  def test_speed_band_has_hysteresis(self):
    lng = self._build()
    self._settle_speed(lng)
    self.assertTrue(lng.speed_allowed)
    # between the two thresholds, the previous state holds
    self._step(lng, v_ego=47.0 / MS_TO_MPH)
    self.assertTrue(lng.speed_allowed)
    self._step(lng, v_ego=40.0 / MS_TO_MPH)
    self.assertFalse(lng.speed_allowed)

  def test_gas_cut_when_closing_on_a_near_lead(self):
    lng = self._build()
    # 1.0 s of headway at 60 mph, closing
    lead = make_lead(status=True, d_rel=HIGHWAY_MS * 1.0, v_rel=-3.0, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    result = self._step(lng, op_accel=0.0, op_gas=0.5, lead=lead)
    self.assertTrue(result.follow_control_used)
    self.assertEqual(result.gas, 0.0)

  def test_gas_kept_when_closing_on_a_distant_lead(self):
    lng = self._build()
    lead = make_lead(status=True, d_rel=HIGHWAY_MS * 3.0, v_rel=-1.0, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    result = self._step(lng, op_accel=0.0, op_gas=0.5, lead=lead)
    self.assertAlmostEqual(result.gas, 0.5)

  def test_gas_capped_while_pacing(self):
    lng = self._build()
    lead = make_lead(status=True, d_rel=HIGHWAY_MS * 2.0, v_rel=0.0, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    result = self._step(lng, op_accel=0.0, op_gas=1.5, lead=lead)
    self.assertAlmostEqual(result.gas, 0.2)

  def test_gas_kept_while_trailing(self):
    lng = self._build()
    lead = make_lead(status=True, d_rel=HIGHWAY_MS * 2.0, v_rel=3.0, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    result = self._step(lng, op_accel=0.0, op_gas=1.5, lead=lead)
    self.assertAlmostEqual(result.gas, 1.5)

  def test_no_lead_holds_accel_at_zero(self):
    lng = self._build()
    self._settle_speed(lng)
    result = self._step(lng, op_accel=-1.0, op_gas=0.5)
    self.assertTrue(result.follow_control_used)
    self.assertEqual(result.accel, 0.0)
    self.assertAlmostEqual(result.gas, 0.5)

  def test_slow_lead_is_left_to_the_planner(self):
    lng = self._build()
    lead = make_lead(status=True, d_rel=20.0, v_rel=-5.0, v_lead=URBAN_MS)
    self._settle_speed(lng, lead=lead)
    result = self._step(lng, op_accel=0.0, op_gas=0.5, lead=lead)
    self.assertFalse(result.follow_control_used)

  def test_driver_input_hands_back(self):
    lng = self._build()
    lead = make_lead(status=True, d_rel=HIGHWAY_MS, v_rel=-3.0, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    for kwargs in ({'gas_pressed': True}, {'brake_pressed': True}):
      result = self._step(lng, op_accel=0.0, op_gas=0.5, lead=lead, **kwargs)
      self.assertFalse(result.follow_control_used, kwargs)
      self.assertAlmostEqual(result.gas, 0.5)

  def test_brake_hysteresis(self):
    lng = self._build(follow_control=False)
    self.assertFalse(self._step(lng, op_accel=-0.10).brake_actuate)  # inside the band
    self.assertTrue(self._step(lng, op_accel=-0.20).brake_actuate)   # past engage
    self.assertTrue(self._step(lng, op_accel=-0.10).brake_actuate)   # holds inside the band
    self.assertFalse(self._step(lng, op_accel=0.0).brake_actuate)    # past release

  def test_precharge_engages_before_the_brakes(self):
    lng = self._build()
    lead = make_lead(status=True, d_rel=HIGHWAY_MS * 2.0, v_rel=0.0, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    lng.accel_last = -1.0  # already braking, so the ease-in limiter is not what decides
    result = self._step(lng, op_accel=-0.13, op_gas=0.0, lead=lead)
    self.assertTrue(result.precharge_actuate)
    self.assertFalse(result.brake_actuate)

  def test_brake_and_gas_are_mutually_exclusive(self):
    lng = self._build(follow_control=False)
    result = self._step(lng, op_accel=-1.0, op_gas=0.5)
    self.assertTrue(result.brake_actuate)
    self.assertEqual(result.gas, CarControllerParams.INACTIVE_GAS)

  def test_braking_eases_in(self):
    """The first brake application is rate limited so it does not stomp."""
    lng = self._build()
    lead = make_lead(status=True, d_rel=HIGHWAY_MS * 3.0, v_rel=0.0, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    result = self._step(lng, op_accel=-2.0, op_gas=0.0, lead=lead)
    self.assertGreater(result.accel, -0.1)

  def test_imminent_collision_is_not_eased_in(self):
    lng = self._build()
    lead = make_lead(status=True, d_rel=5.0, v_rel=-10.0, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    result = self._step(lng, op_accel=-2.0, op_gas=0.0, lead=lead)
    self.assertAlmostEqual(result.accel, -2.0)

  def test_downhill_compensation_toggle(self):
    on = self._build(downhill_compensation=True)
    off = self._build(downhill_compensation=False)
    self.assertAlmostEqual(on.pitch_compensation(-0.5), -0.5)
    self.assertEqual(off.pitch_compensation(-0.5), 0.0)
    # uphill is never dropped
    self.assertAlmostEqual(off.pitch_compensation(0.5), 0.5)

  def test_output_stays_inside_the_can_limits(self):
    lng = self._build()
    lead = make_lead(status=True, d_rel=HIGHWAY_MS, v_rel=-3.0, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    for op_accel in (-10.0, 10.0):
      result = self._step(lng, op_accel=op_accel, op_gas=op_accel, lead=lead)
      self.assertGreaterEqual(result.accel, CarControllerParams.ACCEL_MIN)
      self.assertLessEqual(result.accel, CarControllerParams.ACCEL_MAX)
      self.assertTrue(result.gas == CarControllerParams.INACTIVE_GAS or
                      CarControllerParams.MIN_GAS <= result.gas <= CarControllerParams.ACCEL_MAX)


if __name__ == "__main__":
  unittest.main()
