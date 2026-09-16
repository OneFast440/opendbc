"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import unittest

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car import structs
from opendbc.car.ford.carstate import CarState
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import CAR, CarControllerParams, DBC
from opendbc.car import Bus
from opendbc.sunnypilot.car.ford.fordcan_ext import (
  SHADOW_CURVATURE_MAX_RAW,
  SHADOW_CURVATURE_SCALE,
  create_lka_msg,
)
from opendbc.sunnypilot.car.ford.values_ext import CURVATURE_MAX

# ford.h: CurvatureSteeringLimits for Ford
CURVATURE_TO_CAN = 50000
MAX_CURVATURE_CAN = 1000


def shadow_from_frame(dat: bytes) -> int:
  """Decode bytes 5-6 the way safety/modes/ford.h does."""
  raw = (dat[5] << 8) | dat[6]
  return raw - 65536 if raw > 32767 else raw


def to_can(raw: int) -> int:
  """ford_shadow_curvature_to_can()"""
  return round(raw * 0.05)


class TestShadowCurvatureSaturation(unittest.TestCase):
  """The shadow must never leave the range ford.h checks it against.

  Regression for a real drive: in a 90 degree turn at ~3 m/s the measured curvature reaches about
  0.03 1/m, well past Ford's 0.02 1/m curvature signal range. The shadow tracked it honestly, the
  panda's safety_max_limit_check rejected it, and every LateralMotionControl frame was blocked for
  the length of the corner. Angle mode steers with path_angle, not curvature, so the c2 signal
  range has no business vetoing the command.
  """
  def setUp(self):
    CP = structs.CarParams.new_message(carFingerprint=CAR.FORD_F_150_MK14)
    self.packer = CANPacker(DBC[CAR.FORD_F_150_MK14][Bus.pt])
    self.CAN = CanBus(CP)

  def _shadow_can(self, curvature: float) -> int:
    _, dat, _ = create_lka_msg(self.packer, self.CAN, True, curvature)
    return to_can(shadow_from_frame(bytes(dat)))

  def test_saturates_at_the_panda_limit(self):
    for curvature in (0.0294, 0.05, 0.2, 1.0):
      for sign in (1, -1):
        with self.subTest(curvature=sign * curvature):
          self.assertEqual(self._shadow_can(sign * curvature), sign * MAX_CURVATURE_CAN)

  def test_in_range_values_are_untouched(self):
    for curvature in (0.0, 0.001, 0.01, CURVATURE_MAX):
      for sign in (1, -1):
        with self.subTest(curvature=sign * curvature):
          expected = round(round(sign * curvature / SHADOW_CURVATURE_SCALE) * 0.05)
          self.assertEqual(self._shadow_can(sign * curvature), expected)

  def test_every_curvature_passes_the_max_check(self):
    # sweep past the signal range and the int16 wrap point in both directions
    curvature = -0.5
    while curvature <= 0.5:
      self.assertLessEqual(abs(self._shadow_can(curvature)), MAX_CURVATURE_CAN,
                           f"shadow for {curvature} 1/m would be blocked by ford.h")
      curvature += 0.0005

  def test_saturation_point_matches_the_safety_limit(self):
    self.assertEqual(SHADOW_CURVATURE_MAX_RAW, round(CURVATURE_MAX / SHADOW_CURVATURE_SCALE))
    self.assertEqual(to_can(SHADOW_CURVATURE_MAX_RAW), MAX_CURVATURE_CAN)
    self.assertEqual(MAX_CURVATURE_CAN, round(CURVATURE_MAX * CURVATURE_TO_CAN))

  def test_sign_survives_saturation(self):
    self.assertGreater(self._shadow_can(0.1), 0)
    self.assertLess(self._shadow_can(-0.1), 0)


class TestIpmaDataFrequencyIsDeclared(unittest.TestCase):
  """IPMA_Data's rate is declared, not inferred.

  Regression for a real drive: the camera sends this at a metronomic 1Hz, the parser inferred
  ~12Hz from a startup burst, and the resulting sub-second timeout made the message stale at the
  end of every 1Hz gap. canValid dropped, selfdrived raised canError, and openpilot immediate
  disabled once a second for the whole drive over a passthrough cluster message.
  """
  def setUp(self):
    self.CP = structs.CarParams.new_message(carFingerprint=CAR.FORD_F_150_MK14)
    self.CP_SP = structs.CarParamsSP()

  def test_declared_frequency_survives_a_startup_burst(self):
    parsers = CarState.get_can_parsers(self.CP, self.CP_SP)
    state = parsers[Bus.cam].message_states[0x3D8]
    self.assertEqual(state.frequency, 1)
    # 10 periods of 1Hz, so a 1Hz message with jitter is never stale
    self.assertEqual(state.timeout_threshold, 10e9)

  def test_a_1hz_cadence_stays_valid(self):
    parsers = CarState.get_can_parsers(self.CP, self.CP_SP)
    cam = parsers[Bus.cam]
    packer = CANPacker(DBC[CAR.FORD_F_150_MK14][Bus.pt])
    bus = CanBus(self.CP).camera

    # a startup burst, the shape that used to poison the inferred frequency
    t = 0
    for _ in range(5):
      _, dat, _ = packer.make_can_msg("IPMA_Data", bus, {})
      cam.update([(t, [(0x3D8, dat, bus)])])
      t += 20_000_000  # 20ms apart

    # then the real 1Hz cadence, with the jitter seen on the car
    for gap_ms in (1004, 996, 1008, 995, 1009, 993, 1005):
      t += gap_ms * 1_000_000
      _, dat, _ = packer.make_can_msg("IPMA_Data", bus, {})
      cam.update([(t, [(0x3D8, dat, bus)])])
      self.assertTrue(cam.message_states[0x3D8].valid(t, False))

      # and stays valid right up to the next frame
      cam.update([(t + 1_000_000_000 - 1_000_000, [(0x186, b"\x00" * 8, bus)])])
      self.assertTrue(cam.message_states[0x3D8].valid(cam._last_update_nanos, False))

  def test_a_genuinely_dead_camera_still_goes_invalid(self):
    parsers = CarState.get_can_parsers(self.CP, self.CP_SP)
    cam = parsers[Bus.cam]
    packer = CANPacker(DBC[CAR.FORD_F_150_MK14][Bus.pt])
    bus = CanBus(self.CP).camera

    t = 1_000_000_000
    _, dat, _ = packer.make_can_msg("IPMA_Data", bus, {})
    cam.update([(t, [(0x3D8, dat, bus)])])
    self.assertTrue(cam.message_states[0x3D8].valid(t, False))
    self.assertFalse(cam.message_states[0x3D8].valid(t + 11_000_000_000, False))


if __name__ == "__main__":
  unittest.main()
