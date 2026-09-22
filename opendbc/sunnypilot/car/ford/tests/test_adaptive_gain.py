"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Adaptive steering gain: a closed loop on delivered-vs-commanded curvature (FordDeliveryCompensation_ang).
"""
import math
import unittest
from collections import deque

from opendbc.car.ford.values import CAR
from opendbc.sunnypilot.car.ford.lateral_angle_ext import (
  _ADAPT_ERR_SCALE,
  _ADAPT_GAIN_MAX,
  _ADAPT_GAIN_MIN,
  _ADAPT_TAU_FAST_S,
  _ADAPT_TAU_SLOW_S,
  _STEER_DT,
  LateralAngleExt,
  adapt_time_constant,
)
from opendbc.sunnypilot.car.ford.tests.helpers import make_actuators, make_car_params, make_cc, make_cc_sp, make_cs
from opendbc.sunnypilot.car.ford.values_ext import PrimaryLateralControl

V_LOW = 7.0       # m/s, below the deviation clip, so the gain is all that stands between request and car
V_MID = 15.0      # m/s, above it
V_HWY = 25.0      # m/s, where the yaw bias is learned
KAPPA = 0.008     # 1/m, a real curve
YAW_BIAS = -0.0047  # rad/s, the offset every logged drive on this truck shows on straight road


def angle_ctrl(enabled=True):
  CP, CP_SP = make_car_params(CAR.FORD_F_150_MK14, mode=PrimaryLateralControl.angle, delivery_compensation=enabled)
  return LateralAngleExt(CP, CP_SP)


class Plant:
  """Lagged linear steering: curvature follows k * path_angle / v through a 0.15 s delay and a
  0.1 s first-order response, the shape fitted to the transmitted path_angle on this truck."""

  def __init__(self, k, v_ego, yaw_bias=0.0):
    self.k, self.v_ego, self.yaw_bias = k, v_ego, yaw_bias
    self.delay = deque([0.0] * 3, maxlen=3)
    self.kappa = 0.0

  def yaw_rate(self):
    return -self.kappa * self.v_ego + self.yaw_bias

  def step(self, path_angle):
    u = self.delay[0]
    self.delay.append(path_angle / self.v_ego)
    self.kappa += _STEER_DT / (0.1 + _STEER_DT) * (self.k * u - self.kappa)


def closed_loop(ctrl, plant, requested, frames):
  cc = make_cc(lat_active=True, curvature=requested)
  cc_sp = make_cc_sp(model_curvature=requested)
  peak = 0.0
  for _ in range(frames):
    out = ctrl.update(cc, cc_sp, make_cs(v_ego=plant.v_ego, yaw_rate=plant.yaw_rate()), make_actuators(curvature=requested))
    plant.step(out.path_angle)
    peak = max(peak, abs(plant.kappa))
  return peak


def open_loop(ctrl, frames, requested, delivered, v_ego=V_LOW, yaw_bias=0.0, **cs_kwargs):
  """Hold a curve with a delivery that does not answer the command: the loop has nothing to
  converge to, so this shows which way and how fast it moves, and what stops it."""
  cc = make_cc(lat_active=True, curvature=requested)
  cc_sp = make_cc_sp(model_curvature=requested)
  for _ in range(frames):
    cs = make_cs(v_ego=v_ego, yaw_rate=-delivered * v_ego + yaw_bias, **cs_kwargs)
    ctrl.update(cc, cc_sp, cs, make_actuators(curvature=requested))
  return ctrl


def ramp(ctrl, start, end, frames, delivered_ratio, v_ego=V_LOW):
  for i in range(frames):
    requested = start + (end - start) * i / (frames - 1)
    cc = make_cc(lat_active=True, curvature=requested)
    cs = make_cs(v_ego=v_ego, yaw_rate=-requested * delivered_ratio * v_ego)
    ctrl.update(cc, make_cc_sp(model_curvature=requested), cs, make_actuators(curvature=requested))
  return ctrl


def learn_bias(ctrl, frames=1600):
  """Straight, hands-free highway: the only place the sensor offset can be read."""
  cc = make_cc(lat_active=True, curvature=0.0)
  for _ in range(frames):
    ctrl.update(cc, make_cc_sp(), make_cs(v_ego=V_HWY, yaw_rate=YAW_BIAS), make_actuators())
  return ctrl


class TestAdaptiveGainConverges(unittest.TestCase):
  """Delivered equals commanded, from either side, and nowhere else."""

  def test_off_by_default(self):
    CP, CP_SP = make_car_params(CAR.FORD_F_150_MK14, mode=PrimaryLateralControl.angle)
    self.assertFalse(CP_SP.fordLateralTuning.deliveryCompensation)
    self.assertFalse(LateralAngleExt(CP, CP_SP).delivery_comp_enabled)

  def test_neutral_until_something_is_measured(self):
    self.assertEqual(angle_ctrl().delivery_comp, 1.0)

  def test_converges_to_the_plant_inverse(self):
    """An under-delivering, a nearly exact and an over-delivering module all end on the request.

    The version this replaces targeted 1 / ratio, which ignores the gain already in the loop and
    settles at 1 / sqrt(k): on the 0.6 plant that is 0.88 of the request, not 1.00.
    """
    for k in (0.6, 0.8, 1.1):
      with self.subTest(k=k):
        plant = Plant(k, V_LOW)
        closed_loop(angle_ctrl(), plant, 0.02, 400)
        self.assertAlmostEqual(plant.kappa / 0.02, 1.0, delta=0.02)

  def test_it_is_better_than_no_correction(self):
    for k in (0.6, 1.1):
      with self.subTest(k=k):
        on, off = Plant(k, V_LOW), Plant(k, V_LOW)
        closed_loop(angle_ctrl(), on, 0.02, 400)
        closed_loop(angle_ctrl(enabled=False), off, 0.02, 400)
        self.assertLess(abs(on.kappa - 0.02), abs(off.kappa - 0.02) / 5)

  def test_raises_on_a_shortfall_and_cuts_on_an_excess(self):
    ctrl = angle_ctrl()
    closed_loop(ctrl, Plant(0.6, V_LOW), 0.02, 400)
    self.assertGreater(ctrl.delivery_comp, 1.2)
    ctrl = angle_ctrl()
    closed_loop(ctrl, Plant(1.1, V_LOW), 0.02, 400)
    self.assertLess(ctrl.delivery_comp, 0.8)

  def test_breaks_the_deviation_clip_latch(self):
    """Above 9 m/s the command is held within 0.004 1/m of the measurement. If the loop gain
    k * G is above 1 that clip stops being a limit and becomes a latch past a curvature of
    band * kG / (kG - 1): the car over-delivers, the command follows the measurement up, and the
    car over-delivers on that. Off, this runs away. On, the loop takes the gain back under 1 and
    the car lands on the request."""
    k = 1.1  # k * G = 1.39 at 15 m/s on this platform's default schedule: latches past ~0.014
    off = Plant(k, V_MID)
    closed_loop(angle_ctrl(enabled=False), off, 0.012, 600)
    self.assertGreater(off.kappa, 0.012 * 2.0)
    on = Plant(k, V_MID)
    peak = closed_loop(angle_ctrl(), on, 0.012, 600)
    self.assertAlmostEqual(on.kappa / 0.012, 1.0, delta=0.03)
    self.assertLess(peak, 0.012 * 1.5)

  def test_bounded(self):
    self.assertLessEqual(open_loop(angle_ctrl(), 600, KAPPA, KAPPA * 0.8).delivery_comp, _ADAPT_GAIN_MAX)
    self.assertGreaterEqual(open_loop(angle_ctrl(), 600, KAPPA, KAPPA * 1.3).delivery_comp, _ADAPT_GAIN_MIN)
    self.assertAlmostEqual(open_loop(angle_ctrl(), 600, KAPPA, KAPPA * 0.8).delivery_comp, _ADAPT_GAIN_MAX, places=3)
    self.assertAlmostEqual(open_loop(angle_ctrl(), 600, KAPPA, KAPPA * 1.3).delivery_comp, _ADAPT_GAIN_MIN, places=3)

  def test_a_bigger_error_moves_it_faster(self):
    """Error-scaled rate, not an error-scaled gain: the target is what the measurement says, and
    the error only decides how fast the gain gets there."""
    self.assertEqual(adapt_time_constant(0.0), _ADAPT_TAU_SLOW_S)
    self.assertAlmostEqual(adapt_time_constant(_ADAPT_ERR_SCALE), _ADAPT_TAU_SLOW_S / math.e)
    errors = [0.0, 0.001, 0.002, 0.004, 0.008]
    taus = [adapt_time_constant(e) for e in errors]
    self.assertEqual(taus, sorted(taus, reverse=True))
    self.assertEqual(adapt_time_constant(1.0), _ADAPT_TAU_FAST_S)

  def test_a_bigger_error_does_not_overshoot(self):
    """A tight curve arrives with a big error and so the fastest walk; it still lands on the
    request instead of past it."""
    plant = Plant(0.6, V_LOW)
    peak = closed_loop(angle_ctrl(), plant, 0.04, 400)
    self.assertAlmostEqual(plant.kappa / 0.04, 1.0, delta=0.02)
    self.assertLess(peak, 0.04 * 1.05)


class TestAdaptiveGainHolds(unittest.TestCase):
  """Frames that measure something other than the gain must not move it."""

  def test_holds_while_the_command_grows(self):
    """Curve entry: the car is behind because it is lagging, not because the gain is short."""
    ctrl = ramp(angle_ctrl(), 0.005, 0.03, 60, delivered_ratio=0.7)
    self.assertEqual(ctrl.delivery_comp, 1.0)

  def test_holds_while_the_command_shrinks(self):
    """Curve exit: the car is ahead because it is lagging, not because the gain is long."""
    ctrl = ramp(angle_ctrl(), 0.03, 0.005, 60, delivered_ratio=1.3)
    self.assertEqual(ctrl.delivery_comp, 1.0)

  def test_holds_when_something_else_shaped_the_output(self):
    """0.08 1/m at 7 m/s asks for more path_angle than the signal can carry. The shortfall that
    follows is the DBC limit's, and 0.5 is a ratio the loop would otherwise act on."""
    ctrl = open_loop(angle_ctrl(), 300, 0.08, 0.08 * 0.5)
    self.assertTrue(ctrl.output_constrained)
    self.assertEqual(ctrl.delivery_comp, 1.0)

  def test_holds_on_a_shortfall_no_gain_could_explain(self):
    """0.30 delivered would need 3.3x: a stalled or saturated module, not a gain to chase."""
    ctrl = open_loop(angle_ctrl(), 300, KAPPA, KAPPA * 0.3)
    self.assertEqual(ctrl.delivery_comp, 1.0)

  def test_relaxes_while_the_driver_steers(self):
    ctrl = open_loop(angle_ctrl(), 300, KAPPA, KAPPA * 0.8)
    self.assertGreater(ctrl.delivery_comp, 1.3)
    open_loop(ctrl, 600, KAPPA, KAPPA * 0.8, steering_pressed=True)
    self.assertAlmostEqual(ctrl.delivery_comp, 1.0, delta=0.01)

  def test_relaxes_on_a_straight(self):
    ctrl = open_loop(angle_ctrl(), 300, KAPPA, KAPPA * 1.3)
    self.assertLess(ctrl.delivery_comp, 0.7)
    open_loop(ctrl, 600, 0.0, 0.0)
    self.assertAlmostEqual(ctrl.delivery_comp, 1.0, delta=0.01)


class TestAdaptiveGainOutput(unittest.TestCase):

  def _path_angle(self, ctrl, delivered):
    cc = make_cc(lat_active=True, curvature=KAPPA)
    cs = make_cs(v_ego=V_LOW, yaw_rate=-delivered * V_LOW)
    return ctrl.update(cc, make_cc_sp(model_curvature=KAPPA), cs, make_actuators(curvature=KAPPA)).path_angle

  def test_it_moves_path_angle_and_leaves_the_shadow_alone(self):
    """The panda deviation-checks the shadow, which is kappa_cmd. Only path_angle may move."""
    for ratio in (0.8, 1.2):
      with self.subTest(ratio=ratio):
        plain = open_loop(angle_ctrl(enabled=False), 300, KAPPA, KAPPA * ratio)
        comp = open_loop(angle_ctrl(), 300, KAPPA, KAPPA * ratio)
        a, b = self._path_angle(plain, KAPPA * ratio), self._path_angle(comp, KAPPA * ratio)
        if ratio < 1.0:
          self.assertGreater(abs(b), abs(a))
        else:
          self.assertLess(abs(b), abs(a))
        self.assertAlmostEqual(plain.shadow_curvature, comp.shadow_curvature, places=9)

  def test_does_nothing_when_disabled(self):
    """Tracked either way, so a log shows what it would have done, but it must not reach the gain."""
    off = open_loop(angle_ctrl(enabled=False), 300, KAPPA, KAPPA * 0.8)
    self.assertGreater(off.delivery_comp, 1.05)
    self.assertEqual(off.comp_applied, 1.0)
    reference = open_loop(angle_ctrl(enabled=False), 300, KAPPA, KAPPA * 0.8)
    reference.delivery_comp = 1.0
    self.assertAlmostEqual(self._path_angle(off, KAPPA * 0.8), self._path_angle(reference, KAPPA * 0.8), places=9)


class TestYawBias(unittest.TestCase):
  """A sensor offset is bias / v in curvature: largest exactly where the corners are tightest."""

  def test_learned_on_straight_road(self):
    self.assertAlmostEqual(learn_bias(angle_ctrl()).yaw_bias, YAW_BIAS, delta=abs(YAW_BIAS) * 0.03)

  def test_not_learned_in_a_curve_or_at_low_speed(self):
    ctrl = open_loop(angle_ctrl(), 1600, KAPPA, KAPPA, v_ego=V_HWY)
    self.assertEqual(ctrl.yaw_bias, 0.0)
    ctrl = open_loop(angle_ctrl(), 1600, 0.0, 0.0, v_ego=V_LOW, yaw_bias=YAW_BIAS)
    self.assertEqual(ctrl.yaw_bias, 0.0)

  def test_not_learned_as_a_gain(self):
    """An exact module seen through the truck's own yaw offset, in both directions. Without the
    de-bias the loop reads the offset as a gain error, opposite in left and right turns."""
    k = 1.0 / 1.3  # exact: k * G = 1 at 7 m/s
    for sign in (1.0, -1.0):
      with self.subTest(sign=sign):
        debiased = learn_bias(angle_ctrl())
        closed_loop(debiased, Plant(k, V_LOW, YAW_BIAS), sign * KAPPA, 400)
        self.assertAlmostEqual(debiased.delivery_comp, 1.0, delta=0.015)
        naive = angle_ctrl()
        closed_loop(naive, Plant(k, V_LOW, YAW_BIAS), sign * KAPPA, 400)
        self.assertGreater(abs(naive.delivery_comp - 1.0), 0.04)


if __name__ == "__main__":
  unittest.main()
