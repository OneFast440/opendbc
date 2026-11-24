"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Steering module rows: the law gives the module's own lookups, angle mode on a row makes the module target exactly the
curvature asked for, a row needs both part numbers, and the firmware query asks the steering module for its data file.
"""
import math
import unittest

import numpy as np

from opendbc.car import structs
from opendbc.car.ford.values import FORD_DATA_FILE_REQUEST, FORD_DATA_FILE_RESPONSE, FW_QUERY_CONFIG
from opendbc.car.fw_query_definitions import StdQueries
from opendbc.car.lateral import MAX_LATERAL_ACCEL
from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt
from opendbc.sunnypilot.car.ford.pscm_rows import PSCM_ROWS, pscm_row
from opendbc.sunnypilot.car.ford.tests.helpers import make_actuators, make_car_params, make_cc, make_cc_sp, make_cs
from opendbc.sunnypilot.car.ford.values_ext import FordSafetyFlagsSP, PrimaryLateralControl
from opendbc.sunnypilot.car.interfaces import _initialize_ford

Ecu = structs.CarParams.Ecu
RAPTOR = PSCM_ROWS[(b'ML3V-14D003-BC', b'PL3V-14D007-CB')]

# K2 at T = 5 s (km/h, 1/m per rad), from an independent implementation of the module's law (not pscm_rows) run on
# PL3V-14D007-CB's float32 tables as the file stores them
K2_FROM_THE_FILE = (
  (12, 0.2887401395535166), (20, 0.16038097942367813), (30, 0.10587629616821914), (40, 0.07891879624958165),
  (48.6, 0.06466626285171287), (60, 0.05223879705334866), (70, 0.04468089251908225), (80, 0.03943480655150609),
  (96.6, 0.032913487654205986), (110, 0.02951872539542516), (120, 0.026992980188309354), (130, 0.02446723498119354),
  (150, 0.023099999874830246),
)


def eps_fw(request, version, logging=True, ecu=Ecu.eps):
  return structs.CarParams.CarFw(ecu=ecu, fwVersion=version, address=0x730, bus=0, logging=logging,
                                 request=[StdQueries.TESTER_PRESENT_REQUEST, request])


PROGRAM = eps_fw(StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST, b'ML3V-14D003-BC' + b'\x00' * 10, logging=False)
DATA_FILE = eps_fw(FORD_DATA_FILE_REQUEST, b'PL3V-14D007-CB' + b'\x00' * 10)


class TestPscmRows(unittest.TestCase):
  def test_k2_is_the_files_own(self):
    for v_kph, k2 in K2_FROM_THE_FILE:
      self.assertAlmostEqual(RAPTOR.k2_at(v_kph, 5.0) / k2, 1.0, delta=1e-6, msg=v_kph)

  def test_the_module_targets_the_curvature_asked_for(self):
    """The module's target is K2 * path angle (c0, c2 and c3 all zero in angle mode)."""
    hold = RAPTOR.hold_kph(5.0)
    for v_kph in range(12, 200, 7):
      v = v_kph / 3.6
      for kappa in (0.0005, 0.002, 0.01):
        path_angle = kappa * v * RAPTOR.angle_gain(v, 5.0, hold)
        self.assertAlmostEqual(RAPTOR.k2_at(v_kph, 5.0) * path_angle, kappa, delta=1e-12)

  def test_extended_mode_changes_nothing_on_this_row(self):
    """Mode 2 takes 0.3 s off T, but the preview distance stays on its ceiling either way."""
    t2 = RAPTOR.t_for_mode(2)
    self.assertAlmostEqual(t2, 4.7, places=6)
    self.assertEqual(RAPTOR.t_for_mode(1), 5.0)
    for v_kph in range(10, 200, 5):
      self.assertEqual(RAPTOR.k2_at(v_kph, t2), RAPTOR.k2_at(v_kph, 5.0))

  def test_the_gain_is_held_below_the_k2_table(self):
    """Below about 9 km/h K2 sits at the end of its table; the gain is held, so the path angle stays proportional to
    speed at a crawl instead of growing as speed falls to zero."""
    hold = RAPTOR.hold_kph(5.0)
    self.assertTrue(8.5 < hold < 9.5, hold)
    held = RAPTOR.angle_gain(hold / 3.6, 5.0, hold)
    for v in (0.0, 0.5, 1.0, 2.0):
      self.assertEqual(RAPTOR.angle_gain(v, 5.0, hold), held)
    self.assertTrue(1.0 < held < 1.3, held)

  def test_the_platform_gains_fall_short_on_this_row(self):
    """The hand-set gains (1.0 below 30 mph, 0.95 above 60 mph) leave this module targeting 0.84-0.89 of the request,
    the ~0.87 delivery ratio this platform's drives measured."""
    hold = RAPTOR.hold_kph(5.0)
    for v_kph, platform_gain in ((48.6, 1.0), (96.6, 0.95), (120, 0.95)):
      ratio = platform_gain / RAPTOR.angle_gain(v_kph / 3.6, 5.0, hold)
      self.assertTrue(0.83 < ratio < 0.9, (v_kph, ratio))

  def test_a_row_needs_both_part_numbers(self):
    self.assertIs(pscm_row([PROGRAM, DATA_FILE]), RAPTOR)
    self.assertIs(pscm_row([DATA_FILE, PROGRAM]), RAPTOR)
    self.assertIsNone(pscm_row([]))
    self.assertIsNone(pscm_row([PROGRAM]))
    self.assertIsNone(pscm_row([DATA_FILE]))
    self.assertIsNone(pscm_row([PROGRAM, eps_fw(FORD_DATA_FILE_REQUEST, b'ML34-14D007-BDH\x00')]))
    self.assertIsNone(pscm_row([eps_fw(StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST, b'ML3V-14D003-BD\x00'), DATA_FILE]))
    # the same identifier from another module is not the steering module's
    self.assertIsNone(pscm_row([PROGRAM, eps_fw(FORD_DATA_FILE_REQUEST, b'PL3V-14D007-CB', ecu=Ecu.abs)]))

  def test_the_firmware_query_asks_the_steering_module_for_its_data_file(self):
    requests = [r for r in FW_QUERY_CONFIG.requests if FORD_DATA_FILE_REQUEST in r.request]
    self.assertEqual(len(requests), 1)
    self.assertEqual(requests[0].whitelist_ecus, [Ecu.eps])
    self.assertIn(FORD_DATA_FILE_RESPONSE, requests[0].response)
    self.assertTrue(requests[0].logging)   # never part of fingerprinting
    self.assertEqual(requests[0].bus, 0)


class TestAngleModeOnARow(unittest.TestCase):
  @staticmethod
  def _settle(lat, v, kappa, frames=60):
    result = None
    for _ in range(frames):
      result = lat.update(make_cc(), make_cc_sp(model_curvature=kappa), make_cs(v_ego=v, yaw_rate=-kappa * v),
                          make_actuators(kappa))
    return result

  @staticmethod
  def _lat(car_fw=(), **tuning):
    CP, CP_SP = make_car_params(mode=PrimaryLateralControl.angle, **tuning)
    CP.carFw = list(car_fw)
    return LateralAngleExt(CP, CP_SP)

  def test_path_angle_is_sized_by_the_modules_gain(self):
    v = 30.0
    gain = RAPTOR.angle_gain(v, 5.0, RAPTOR.hold_kph(5.0))
    for kappa in (0.0005, 0.001, 0.002):
      with self.subTest(kappa=kappa):
        on_row = self._settle(self._lat([PROGRAM, DATA_FILE]), v, kappa)
        self.assertAlmostEqual(on_row.path_angle, kappa * v * gain, places=6)
        self.assertAlmostEqual(RAPTOR.k2_at(v * 3.6, 5.0) * on_row.path_angle, kappa, places=7)

  def test_every_other_truck_keeps_the_platform_gains(self):
    v, kappa = 30.0, 0.001
    for car_fw in ((), (PROGRAM,), (DATA_FILE,), (PROGRAM, eps_fw(FORD_DATA_FILE_REQUEST, b'ML34-14D007-BDH'))):
      with self.subTest(car_fw=len(car_fw)):
        lat = self._lat(car_fw)
        self.assertIsNone(lat.module_row)
        self.assertAlmostEqual(self._settle(lat, v, kappa).path_angle, kappa * v * 0.95, places=6)

  def test_the_users_factors_scale_the_modules_gain(self):
    v = 30.0
    gain = RAPTOR.angle_gain(v, 5.0, RAPTOR.hold_kph(5.0))
    straight = self._settle(self._lat([PROGRAM, DATA_FILE], high_speed_dampening=0.9), v, 0.0005)
    self.assertAlmostEqual(straight.path_angle, 0.0005 * v * gain * 0.9, places=6)
    curve = self._settle(self._lat([PROGRAM, DATA_FILE], high_speed_factor=1.2), v, 0.002)
    self.assertAlmostEqual(curve.path_angle, 0.002 * v * gain * 1.2, places=6)

  def test_extended_mode_test_uses_mode_2s_preview_time(self):
    lat = self._lat([PROGRAM, DATA_FILE], extended_mode_test=True)
    self.assertAlmostEqual(lat.module_t, 4.7, places=6)

  def test_path_angle_is_held_inside_the_bound(self):
    """A request past comma's lateral acceleration cap stops at the controller's bound, inside the panda's."""
    v, kappa = 30.0, 6.0 / 30.0 ** 2   # 6 m/s^2
    lat = self._lat([PROGRAM, DATA_FILE])
    result = self._settle(lat, v, kappa, frames=120)
    self.assertAlmostEqual(result.path_angle, RAPTOR.max_path_angle(v), places=6)
    self.assertLess(result.path_angle, RAPTOR.panda_max_path_angle(v))
    # what the module then targets, at its own gain, stays under the cap
    self.assertLess(RAPTOR.k2_at(v * 3.6, 5.0) * result.path_angle * v ** 2, MAX_LATERAL_ACCEL)
    # and every other truck is unbounded here, as before
    self.assertAlmostEqual(self._settle(self._lat(), v, kappa, frames=120).path_angle, kappa * v * 0.95, places=6)


class TestPscmRowBound(unittest.TestCase):
  def test_k2_max_never_rises_with_speed(self):
    speeds = np.arange(0.0, 60.0, 0.05)
    k2 = [RAPTOR.k2_max(s) for s in speeds]
    self.assertTrue(all(a >= b for a, b in zip(k2, k2[1:], strict=False)))
    self.assertEqual(RAPTOR.k2_max(0.0), RAPTOR.k2[0])

  def test_k2_max_covers_every_gain_the_module_can_use(self):
    """Over every preview time T can reach, at every speed, the module's K2 is under the bound's (before the margin)."""
    for v_kph in np.arange(0.5, 200.0, 0.5):
      for t in np.arange(RAPTOR.t_min, RAPTOR.t_max + 1e-9, 0.05):
        self.assertLessEqual(RAPTOR.k2_at(v_kph, t), RAPTOR.k2_max(v_kph / 3.6) + 1e-12, (v_kph, t))

  def test_the_controllers_bound_is_inside_the_pandas(self):
    """With the panda's lowest recent speed anywhere within 1 m/s of the controller's, the controller's bound as the
    packer rounds it is at least one count under the panda's (ford.h: the bound in counts plus one, truncated)."""
    for v in np.arange(0.0, 50.0, 0.05):
      sent = math.floor(RAPTOR.max_path_angle(v) * 2000.0 + 0.5)
      for v_low in np.arange(max(v - 1.0, 0.0), v + 1.0 + 1e-9, 0.1):
        allowed = int(RAPTOR.panda_max_path_angle(v_low) * 2000.0 + 1.0)
        self.assertGreaterEqual(allowed - sent, 1, (v, v_low))

  def test_a_full_flat_road_request_passes_at_winding_road_speeds(self):
    """controlsd asks for at most 3.0 m/s^2 on a flat road. From 8 to 22 m/s (29-79 km/h) the module's target for that
    request is inside the controller's bound; higher, the bound's preview-time floor and margin still trim it a little,
    and lower, the path angle signal's own range is what is left."""
    for v in np.arange(8.0, 22.0 + 1e-9, 0.25):
      kappa = 3.0 / v ** 2
      self.assertLess(kappa / RAPTOR.k2_at(v * 3.6, 5.0), RAPTOR.max_path_angle(v), v)

  def test_normal_driving_is_left_alone(self):
    """Up to 2.5 m/s^2 the controller's own gain stays inside the bound everywhere the bound is under the signal's range."""
    for v in np.arange(9.0, 45.0, 0.5):
      kappa = 2.5 / v ** 2
      self.assertLess(kappa / RAPTOR.k2_at(v * 3.6, 5.0), RAPTOR.max_path_angle(v), v)

  def test_the_panda_hears_of_the_row_only_in_angle_mode(self):
    for mode in PrimaryLateralControl:
      for car_fw, row_bits in (([PROGRAM, DATA_FILE], RAPTOR.panda_row), ([PROGRAM], 0), ([DATA_FILE], 0), ([], 0)):
        with self.subTest(mode=mode, entries=len(car_fw)):
          CP, CP_SP = make_car_params()
          CP.carFw = car_fw
          _initialize_ford(CP, CP_SP, {"FordPrefLateralControl": str(int(mode))})
          expected = row_bits if mode == PrimaryLateralControl.angle else 0
          self.assertEqual((CP_SP.safetyParam & FordSafetyFlagsSP.PSCM_ROW_MASK) >> FordSafetyFlagsSP.PSCM_ROW_SHIFT,
                           expected)
          self.assertEqual(CP_SP.safetyParam & FordSafetyFlagsSP.LATERAL_MODE_MASK, int(mode))


if __name__ == '__main__':
  unittest.main()
