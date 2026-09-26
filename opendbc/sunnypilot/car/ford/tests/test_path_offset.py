"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Angle mode's c0 (path offset) channel: FordPathOffsetLimit_ang.
"""
import os
import re
import unittest

import numpy as np

from opendbc.car.ford.values import CAR
from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt
from opendbc.sunnypilot.car.ford.tests.helpers import (
  INACTIVE_PATH_ANGLE,
  INACTIVE_PATH_OFFSET,
  MSG_LateralMotionControl2,
  make_actuators,
  make_car_controller,
  make_car_params,
  make_cc,
  make_cc_sp,
  make_cs,
  unpack_lat_ctl,
)
from opendbc.sunnypilot.car.ford.values_ext import (
  ANGLE_PATH_OFFSET_MAX,
  ANGLE_PATH_OFFSET_ROC,
  ANGLE_PATH_OFFSET_SPEED_BP,
  ANGLE_PATH_OFFSET_SPEED_V,
  PrimaryLateralControl,
)

TIGHT = 0.06       # 1/m, a ~17 m radius intersection turn


def angle_ctrl(limit=0.0):
  CP, CP_SP = make_car_params(CAR.FORD_F_150_MK14, mode=PrimaryLateralControl.angle, path_offset_limit=limit)
  return LateralAngleExt(CP, CP_SP)


def hold(ctrl, frames, curvature, v_ego, yaw_rate=None, lat_active=True):
  """Hold a request. The measurement follows it unless told otherwise, so the deviation clip is
  not what shapes the command and these tests stay about the split."""
  yaw = -curvature * v_ego if yaw_rate is None else yaw_rate
  out = []
  for _ in range(frames):
    out.append(ctrl.update(make_cc(lat_active=lat_active, curvature=curvature), make_cc_sp(model_curvature=curvature),
                           make_cs(v_ego=v_ego, yaw_rate=yaw), make_actuators(curvature)))
  return out


class TestPathOffsetSplit(unittest.TestCase):

  def test_off_by_default_and_byte_identical(self):
    """No limit set: c0 stays 0 and c1 is exactly the single-channel path_angle."""
    CP, CP_SP = make_car_params(CAR.FORD_F_150_MK14, mode=PrimaryLateralControl.angle)
    self.assertEqual(CP_SP.fordLateralTuning.pathOffsetLimit, 0.0)
    for v_ego in (3.0, 6.0, 12.0):
      off = hold(angle_ctrl(0.0), 60, TIGHT * 6.0 / v_ego, v_ego)
      for r in off:
        self.assertEqual(r.path_offset, 0.0)
      ref = LateralAngleExt(CP, CP_SP)
      ref_out = hold(ref, 60, TIGHT * 6.0 / v_ego, v_ego)
      self.assertEqual([r.path_angle for r in off], [r.path_angle for r in ref_out])

  def test_c0_carries_part_of_a_tight_low_speed_turn(self):
    """Opposite sign to c1 on the wire (see _C0_WIRE_SIGN), capped at the limit, and c1 gives up
    what c0 adds."""
    for limit in (0.3, 1.0):
      for sign in (1.0, -1.0):
        with self.subTest(limit=limit, sign=sign):
          ctrl = angle_ctrl(limit)
          r = hold(ctrl, 80, sign * TIGHT, 6.0)[-1]
          self.assertLess(r.path_offset * sign, 0.0)
          self.assertAlmostEqual(abs(r.path_offset), limit, places=6)
          self.assertGreater(r.path_angle * sign, 0.0)
          self.assertLess(abs(r.path_angle), abs(ctrl.path_angle_last))

  def test_the_pair_asks_for_the_same_curvature(self):
    """c1 + (what c0 is expected to add, in c1 units) equals the single-channel command."""
    ctrl = angle_ctrl(1.0)
    v_ego = 6.0
    r = hold(ctrl, 80, TIGHT, v_ego)[-1]
    full = ctrl.path_angle_last
    g0 = 2.0 / (max(v_ego, 4.0) * 2.6) ** 2
    gain = full / (TIGHT * v_ego)
    # c0 in the turn's own direction is the wire value negated back
    self.assertAlmostEqual(r.path_angle + g0 * v_ego * gain * -r.path_offset, full, places=6)

  def test_small_turns_split_by_equal_arrival(self):
    """Below the cap, c0 is 15 m per rad of c1: both held copies arrive together."""
    r = hold(angle_ctrl(1.0), 80, 0.004, 6.0)[-1]
    self.assertLess(abs(r.path_offset), 1.0)
    self.assertAlmostEqual(r.path_offset / r.path_angle, -15.0, places=4)

  def test_fades_with_speed(self):
    for v_ego in (1.5, 3.0, 6.0, 10.0, 13.0, 14.0, 20.0, 30.0):
      with self.subTest(v_ego=v_ego):
        r = hold(angle_ctrl(1.0), 80, 2.0 / v_ego ** 2, v_ego)[-1]
        cap = float(np.interp(v_ego, ANGLE_PATH_OFFSET_SPEED_BP, ANGLE_PATH_OFFSET_SPEED_V))
        self.assertLessEqual(abs(r.path_offset), cap + 1e-9)
        if v_ego <= 2.0 or v_ego >= 14.0:
          self.assertEqual(r.path_offset, 0.0)

  def test_rate_limited_on_both_channels(self):
    ctrl = angle_ctrl(1.0)
    hold(ctrl, 5, 0.0, 6.0)
    prev = hold(ctrl, 1, 0.0, 6.0)[-1]
    for _ in range(40):
      r = hold(ctrl, 1, TIGHT, 6.0, yaw_rate=0.0)[-1]
      self.assertLessEqual(abs(r.path_offset - prev.path_offset), ANGLE_PATH_OFFSET_ROC + 1e-9)
      self.assertLessEqual(abs(r.path_angle - prev.path_angle), 0.055 + 1e-9)
      prev = r

  def test_back_to_zero_when_lateral_drops(self):
    ctrl = angle_ctrl(1.0)
    hold(ctrl, 80, TIGHT, 6.0)
    self.assertNotEqual(ctrl.path_offset_last, 0.0)
    r = hold(ctrl, 1, TIGHT, 6.0, lat_active=False)[-1]
    self.assertEqual(r.path_offset, 0.0)
    self.assertEqual(ctrl.path_offset_last, 0.0)
    # and it rebuilds from zero at the rate limit, not from where it was
    r = hold(ctrl, 1, TIGHT, 6.0)[-1]
    self.assertLessEqual(abs(r.path_offset), ANGLE_PATH_OFFSET_ROC + 1e-9)


class TestPathOffsetOnTheWire(unittest.TestCase):

  def test_c0_and_c1_leave_with_opposite_signs(self):
    """Pins the wire convention the first c0 drive pointed to. Sent with the same sign as c1, c0
    fit the truck's response at -1.0 to -1.5x its expected effect, i.e. it steered the other
    way, so the PSCM most likely reads path offset as the truck relative to the path."""
    for sign in (1.0, -1.0):
      with self.subTest(sign=sign):
        controller, CP = make_car_controller(CAR.FORD_F_150_MK14, mode=PrimaryLateralControl.angle,
                                             path_offset_limit=1.0)
        sig = None
        for _ in range(400):
          _, sends = controller.update(make_cc(curvature=sign * TIGHT), make_cc_sp(model_curvature=sign * TIGHT),
                                       make_cs(v_ego=6.0, yaw_rate=-sign * TIGHT * 6.0), 0)
          for addr, dat, _ in sends:
            if addr == MSG_LateralMotionControl2:
              sig = unpack_lat_ctl(addr, dat)
        c0 = sig['path_offset'] - INACTIVE_PATH_OFFSET
        c1 = sig['path_angle'] - INACTIVE_PATH_ANGLE
        self.assertNotEqual(c0, 0)
        self.assertEqual(np.sign(c0), -np.sign(c1))
        # openpilot's positive curvature goes out negative on both, like path_angle always has
        self.assertEqual(np.sign(c1), -sign)


class TestPathOffsetPandaMirror(unittest.TestCase):
  """ford.h must always allow what openpilot sends, and only slightly more."""

  @classmethod
  def setUpClass(cls):
    with open(os.path.join(os.path.dirname(__file__), "../../../../safety/modes/ford.h")) as f:
      src = f.read()
    table = re.search(r"FORD_ANGLE_MAX_PATH_OFFSET = \{\s*\{([^}]*)\},\s*\{([^}]*)\}", src)
    cls.bp = [float(x.strip().rstrip(".")) for x in table.group(1).split(",")]
    cls.v = [float(x) for x in table.group(2).split(",")]
    cls.roc_can = int(re.search(r"FORD_ANGLE_PATH_OFFSET_ROC_CAN = ([0-9]+);", src).group(1))

  def test_ceiling_covers_openpilot_at_every_speed(self):
    for v_ego in np.arange(0.0, 40.0, 0.25):
      ours = ANGLE_PATH_OFFSET_MAX * float(np.interp(v_ego, ANGLE_PATH_OFFSET_SPEED_BP, ANGLE_PATH_OFFSET_SPEED_V))
      panda = float(np.interp(v_ego - 1.0, self.bp, self.v)) + 0.02
      self.assertGreaterEqual(panda, ours, f"{v_ego} m/s")
      # Below 4 m/s openpilot fades c0 out as a modelling caution (the geometry behind its gain
      # grows without bound at a crawl); the panda keeps the 1.0 m bound it enforces at 4-8 m/s
      # rather than mirror a rising limit, which would need the speed fudged the other way.
      if v_ego >= ANGLE_PATH_OFFSET_SPEED_BP[1]:
        self.assertLessEqual(panda, ours * 1.02 + 0.2 + 0.02, f"{v_ego} m/s: panda far looser than openpilot")

  def test_highway_ceiling_is_packing_slack_only(self):
    self.assertEqual(self.v[-1], 0.0)
    self.assertLessEqual(self.bp[1], ANGLE_PATH_OFFSET_SPEED_BP[-1])

  def test_rate_is_mirrored(self):
    """0.01 m per LSB: openpilot's rate x1.02, plus one LSB of packing slack."""
    self.assertEqual(self.roc_can, int(ANGLE_PATH_OFFSET_ROC * 1.02 * 100 + 1))


if __name__ == "__main__":
  unittest.main()
