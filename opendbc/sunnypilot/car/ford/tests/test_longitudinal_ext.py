"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest

from opendbc.car.ford.values import CarControllerParams
from opendbc.sunnypilot.car.ford.longitudinal_ext import (MS_TO_MPH, _BRAKE_ENGAGE, _BRAKE_RELEASE,
                                                          _GAS_CREEP_SPEED, _GAS_RELEASE,
                                                          _PRECHARGE_ENGAGE, LongitudinalExt)
from opendbc.sunnypilot.car.ford.tests.helpers import make_car_params, make_cc, make_cc_sp, make_cs, make_lead

HIGHWAY_MS = 60.0 / MS_TO_MPH
URBAN_MS = 30.0 / MS_TO_MPH

# Expressed against the thresholds rather than hard coded, so a retune does not silently
# turn these into tests of something else.
BRAKING = _BRAKE_ENGAGE - 0.05                      # past engage
COASTING = (_BRAKE_ENGAGE + _BRAKE_RELEASE) / 2     # inside the band
PRECHARGE_ONLY = (_PRECHARGE_ENGAGE + _BRAKE_ENGAGE) / 2


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

  def test_no_lead_is_left_to_the_planner(self):
    """Every follow limit is defined against a lead, so without one there is nothing to do."""
    lng = self._build()
    self._settle_speed(lng)
    result = self._step(lng, op_accel=0.5, op_gas=0.5)
    self.assertFalse(result.follow_control_used)
    self.assertAlmostEqual(result.accel, 0.5)
    self.assertAlmostEqual(result.gas, 0.5)
    # and a brake request is not capped either
    result = self._step(lng, op_accel=-1.0, op_gas=-1.0)
    self.assertAlmostEqual(result.accel, -1.0)

  def test_no_lead_can_still_brake(self):
    """Regression: the no-lead clamp pinned the brake channel at zero and kept it there,
    which also reset the CarController's rate limiter every frame. Braking was impossible."""
    lng = self._build()
    self._settle_speed(lng)
    sent = 0.0
    for _ in range(60):
      # what the CarController would hand us, including its 3.5 m/s^3 downward limit
      op_accel = max(-3.0, sent - 3.5 * CarControllerParams.ACC_CONTROL_STEP * 0.01)
      sent = self._step(lng, op_accel=op_accel, op_gas=-3.0).accel
    self.assertAlmostEqual(sent, -3.0)

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
    result = self._step(lng, op_accel=0.0, op_gas=0.5, lead=lead, brake_pressed=True)
    self.assertFalse(result.follow_control_used)
    self.assertAlmostEqual(result.gas, 0.5)
    result = self._step(lng, op_accel=0.0, op_gas=0.5, lead=lead, gas_pressed=True)
    self.assertFalse(result.follow_control_used)

  def test_the_accdata_goes_inactive_under_the_drivers_foot(self):
    """The PCM refuses an active AccPrpl_A_Rq while it has cruise in its own override state:
    CmbbDeny_B_ActlPrpl goes to 1 and CcStat_D_Actl to 2, which reads back as accFaulted.
    Seen twice in one drive, with a positive request and with a negative one, so the whole
    message goes back to its inactive form rather than the request being trimmed."""
    lng = self._build(follow_control=False)
    active = self._step(lng, op_accel=0.3, op_gas=0.3)
    self.assertTrue(active.acc_enabled)
    self.assertAlmostEqual(active.gas, 0.3)

    for op_gas in (0.5, 0.0, -0.3, -2.0):
      result = self._step(lng, op_accel=op_gas, op_gas=op_gas, gas_pressed=True)
      self.assertFalse(result.acc_enabled, op_gas)
      self.assertEqual(result.gas, CarControllerParams.INACTIVE_GAS, op_gas)
      self.assertFalse(result.brake_actuate, op_gas)
      self.assertFalse(result.precharge_actuate, op_gas)
      # AccBrkTot_A_Rq has to go inactive with the enable bit. Clearing the bit alone still
      # faulted: the module denied a live brake total under a cleared Cmbb_B_Enbl.
      self.assertEqual(result.accel, 0.0, op_gas)

  def test_a_feathered_pedal_is_not_an_override(self):
    """CarState calls any non-zero pedal an override, so the lightest touch used to take the
    whole ACCDATA inactive and the truck lost speed under a pedal just been rested on."""
    lng = self._build(follow_control=False, pedal_override_threshold=2.0)
    result = lng.update(make_cc(long_active=True), make_cc_sp(),
                        make_cs(v_ego=HIGHWAY_MS, gas_pressed=True, pedal_pc=1.0), 0.3, 0.3, 0.0)
    self.assertTrue(result.acc_enabled)
    self.assertAlmostEqual(result.gas, 0.3)

  def test_a_real_press_is_still_an_override(self):
    lng = self._build(follow_control=False, pedal_override_threshold=2.0)
    result = lng.update(make_cc(long_active=True), make_cc_sp(),
                        make_cs(v_ego=HIGHWAY_MS, gas_pressed=True, pedal_pc=12.0), 0.3, 0.3, 0.0)
    self.assertFalse(result.acc_enabled)
    self.assertEqual(result.gas, CarControllerParams.INACTIVE_GAS)
    self.assertEqual(result.accel, 0.0)

  def test_the_threshold_is_tunable(self):
    for threshold, pedal, overriding in ((2.0, 5.0, True), (10.0, 5.0, False), (10.0, 12.0, True)):
      lng = self._build(follow_control=False, pedal_override_threshold=threshold)
      result = lng.update(make_cc(long_active=True), make_cc_sp(),
                          make_cs(v_ego=HIGHWAY_MS, gas_pressed=True, pedal_pc=pedal), 0.3, 0.3, 0.0)
      self.assertEqual(result.acc_enabled, not overriding, (threshold, pedal))

  def test_a_carstate_without_a_pedal_position_falls_back_to_gas_pressed(self):
    lng = self._build(follow_control=False)
    cs = make_cs(v_ego=HIGHWAY_MS, gas_pressed=True, pedal_pc=0.0)
    del cs.accelerator_pedal_pc
    self.assertFalse(lng.update(make_cc(long_active=True), make_cc_sp(), cs, 0.3, 0.3, 0.0).acc_enabled)

  def test_an_unset_threshold_param_does_not_mean_every_touch(self):
    """A float param that has never been written reads 0.0, which would override on contact."""
    lng = self._build(follow_control=False, pedal_override_threshold=0.0)
    self.assertGreater(lng.pedal_override_pc, 0.0)

  def test_an_inactive_message_carries_no_request_at_all(self):
    lng = self._build(follow_control=False)
    for kwargs in ({'long_active': False}, {'gas_pressed': True}):
      result = self._step(lng, op_accel=-2.0, op_gas=-2.0, **kwargs)
      self.assertFalse(result.acc_enabled, kwargs)
      self.assertEqual(result.accel, 0.0, kwargs)
      self.assertEqual(result.gas, CarControllerParams.INACTIVE_GAS, kwargs)

  def test_acc_enabled_tracks_long_active_otherwise(self):
    lng = self._build(follow_control=False)
    self.assertTrue(self._step(lng, op_accel=0.0, op_gas=0.0).acc_enabled)
    self.assertFalse(self._step(lng, op_accel=0.0, op_gas=0.0, long_active=False).acc_enabled)

  def test_brake_hysteresis(self):
    lng = self._build(follow_control=False)
    self.assertFalse(self._step(lng, op_accel=COASTING).brake_actuate)  # inside the band
    self.assertTrue(self._step(lng, op_accel=BRAKING).brake_actuate)    # past engage
    self.assertTrue(self._step(lng, op_accel=COASTING).brake_actuate)   # holds inside the band
    self.assertFalse(self._step(lng, op_accel=0.0).brake_actuate)       # past release

  def test_the_brakes_stay_out_of_what_the_engine_can_do(self):
    """A request the closed throttle can deliver must not reach for the friction brakes,
    because that lights the lamp at the car behind for nothing."""
    lng = self._build(follow_control=False)
    for accel in (-0.10, -0.20, -0.30, -0.40):
      lng = self._build(follow_control=False)
      self.assertFalse(self._step(lng, op_accel=accel, op_gas=accel).brake_actuate, accel)
    self.assertLessEqual(_BRAKE_ENGAGE, CarControllerParams.MIN_GAS)

  def test_precharge_engages_before_the_brakes(self):
    lng = self._build()
    lead = make_lead(status=True, d_rel=HIGHWAY_MS * 2.0, v_rel=0.0, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    lng.accel_last = -1.0  # already braking, so the ease-in limiter is not what decides
    result = self._step(lng, op_accel=PRECHARGE_ONLY, op_gas=0.0, lead=lead)
    self.assertTrue(result.precharge_actuate)
    self.assertFalse(result.brake_actuate)

  def test_never_brakes_against_the_driver_accelerator(self):
    """With the accelerator override held, openpilot keeps commanding through the press, so
    it must not put the brakes on under the driver's foot."""
    lng = self._build(follow_control=False)
    self._step(lng, op_accel=BRAKING, op_gas=BRAKING)     # latch the brake request on
    self.assertTrue(lng.brake_actuate_last)
    result = self._step(lng, op_accel=-2.0, op_gas=-2.0, gas_pressed=True)
    self.assertFalse(result.brake_actuate)
    self.assertFalse(result.precharge_actuate)

  def test_driver_accelerator_does_not_latch_the_brake_request(self):
    lng = self._build(follow_control=False)
    self._step(lng, op_accel=-2.0, op_gas=-2.0, gas_pressed=True)
    self.assertFalse(lng.brake_actuate_last)

  def test_braking_never_asks_for_throttle(self):
    lng = self._build(follow_control=False)
    result = self._step(lng, op_accel=-1.0, op_gas=0.5)
    self.assertTrue(result.brake_actuate)
    self.assertLessEqual(result.gas, 0.0)

  def test_a_gentle_coast_stays_on_the_gas_channel(self):
    """The request the planner made, not a release to full engine braking."""
    lng = self._build(follow_control=False)
    result = self._step(lng, op_accel=BRAKING, op_gas=-0.2)
    self.assertTrue(result.brake_actuate)
    self.assertAlmostEqual(result.gas, -0.2)

  def test_released_to_inactive_below_min_gas(self):
    """Under MIN_GAS the channel cannot carry the request, so the brakes take it."""
    lng = self._build(follow_control=False)
    self._step(lng, op_accel=0.0, op_gas=0.0)
    result = self._step(lng, op_accel=-0.6, op_gas=-0.6)
    self.assertEqual(result.gas, CarControllerParams.INACTIVE_GAS)

  def test_the_inactive_release_has_hysteresis(self):
    """A command sitting on MIN_GAS must not toggle a 4.5 m/s^2 step every frame."""
    lng = self._build(follow_control=False)
    self._step(lng, op_accel=0.0, op_gas=0.0)
    self.assertEqual(self._step(lng, op_accel=-0.6, op_gas=-0.6).gas,
                     CarControllerParams.INACTIVE_GAS)
    # back inside the expressible range, but not yet past the release threshold
    self.assertEqual(self._step(lng, op_accel=-0.45, op_gas=-0.45).gas,
                     CarControllerParams.INACTIVE_GAS)
    self.assertAlmostEqual(self._step(lng, op_accel=-0.3, op_gas=-0.3).gas, -0.3)
    self.assertLess(_GAS_RELEASE, 0.0)
    self.assertGreater(_GAS_RELEASE, CarControllerParams.MIN_GAS)

  def test_no_propulsion_request_while_stopping(self):
    """Below creep speed the brakes own the stop."""
    lng = self._build(follow_control=False)
    result = self._step(lng, op_accel=BRAKING, op_gas=-0.2, v_ego=_GAS_CREEP_SPEED - 0.1)
    self.assertTrue(result.brake_actuate)
    self.assertEqual(result.gas, CarControllerParams.INACTIVE_GAS)

  def test_inactive_when_not_long_active(self):
    lng = self._build(follow_control=False)
    result = self._step(lng, op_accel=0.5, op_gas=0.5, long_active=False)
    self.assertEqual(result.gas, CarControllerParams.INACTIVE_GAS)

  def test_follow_limits_never_raise_a_decel_request(self):
    """The lead limits are caps. Pacing must not turn a brake request into coasting."""
    lng = self._build()
    lead = make_lead(status=True, d_rel=HIGHWAY_MS * 2.0, v_rel=0.0, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    result = self._step(lng, op_accel=-0.3, op_gas=-0.3, lead=lead)
    self.assertTrue(result.follow_control_used)
    self.assertAlmostEqual(result.gas, -0.3)

  def test_braking_eases_in(self):
    """The first brake application is rate limited so it does not stomp."""
    lng = self._build()
    lead = make_lead(status=True, d_rel=HIGHWAY_MS * 3.0, v_rel=0.0, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    result = self._step(lng, op_accel=-2.0, op_gas=0.0, lead=lead)
    self.assertGreater(result.accel, -0.1)

  def test_a_request_the_ramp_cannot_keep_up_with_is_not_eased_in(self):
    """The ease-in keys off the lead, so a brake request the lead did not cause used to crawl
    out at 0.1 m/s^3 behind a comfortable lead. Logged at 49 mph with the lead 38 m away and
    closing at 2.4 m/s: the planner asked -0.65 and the wire carried -0.04."""
    lng = self._build()
    lead = make_lead(status=True, d_rel=38.0, v_rel=-2.4, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    lng.accel_last = -0.04
    result = self._step(lng, op_accel=-0.15, op_gas=-0.65, lead=lead)
    self.assertTrue(result.follow_control_used)
    self.assertLess(result.accel, -0.10)

  def test_a_lead_being_tracked_comfortably_is_still_eased_in(self):
    lng = self._build()
    lead = make_lead(status=True, d_rel=38.0, v_rel=-2.4, v_lead=HIGHWAY_MS)
    self._settle_speed(lng, lead=lead)
    lng.accel_last = -0.04
    result = self._step(lng, op_accel=-0.15, op_gas=-0.15, lead=lead)
    self.assertTrue(result.follow_control_used)
    self.assertAlmostEqual(result.accel, -0.042)

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
