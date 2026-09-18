"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ford brake lamp indicator, ported from BluePilot.
"""
import math
import unittest

from opendbc.can.packer import CANPacker
from opendbc.car import Bus
from opendbc.car.car_helpers import interfaces
from opendbc.car.ford.carstate import CarState
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import CAR, DBC
from opendbc.sunnypilot.car.ford.carstate_ext import BRAKE_LAMP_MSGS

BCM = 0x3C3           # BCM_Lamp_Stat_FD1
BRAKE_SYS_2 = 0x416   # BrakeSysFeatures_2


def make_params(enabled: bool):
  CI_cls = interfaces[CAR.FORD_F_150_MK14]
  fp = dict.fromkeys(range(7), {})
  CP = CI_cls.get_params(CAR.FORD_F_150_MK14, fp, [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CI_cls.get_params_sp(CP, CAR.FORD_F_150_MK14, fp, [], alpha_long=False, is_release_sp=False, docs=False)
  CP_SP.fordHud.brakeLightStatus = enabled
  return CP, CP_SP


class TestFordBrakeLights(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker(DBC[CAR.FORD_F_150_MK14][Bus.pt])

  def _feed(self, parsers, CP, t, stop_light=0, brk_lamp=0):
    bus = CanBus(CP).main
    frames = []
    for name, values in (("BCM_Lamp_Stat_FD1", {"StopLghtOn_B_Stat": stop_light}),
                         ("BrakeSysFeatures_2", {"BrkLamp_B_Rq": brk_lamp})):
      _, dat, _ = self.packer.make_can_msg(name, bus, values)
      frames.append((BCM if name == "BCM_Lamp_Stat_FD1" else BRAKE_SYS_2, dat, bus))
    parsers[Bus.pt].update([(t, frames)])

  def test_sources_are_not_registered_when_off(self):
    """Off must cost nothing: a message this parser would otherwise never touch would put its
    cadence into canValid for every Ford."""
    CP, CP_SP = make_params(False)
    parsers = CarState.get_can_parsers(CP, CP_SP)
    for addr in (BCM, BRAKE_SYS_2):
      self.assertFalse(addr in parsers[Bus.pt].message_states, f"0x{addr:X} registered while off")

  def test_sources_are_registered_alive_exempt_when_on(self):
    """A cosmetic indicator must never be able to take canValid down."""
    CP, CP_SP = make_params(True)
    parsers = CarState.get_can_parsers(CP, CP_SP)
    for addr in (BCM, BRAKE_SYS_2):
      self.assertTrue(addr in parsers[Bus.pt].message_states, f"0x{addr:X} not registered")
      self.assertTrue(parsers[Bus.pt].message_states[addr].ignore_alive, f"0x{addr:X} gates canValid")

  def test_rates_are_declared_not_inferred(self):
    for _, freq in BRAKE_LAMP_MSGS:
      self.assertTrue(math.isnan(freq), "a guessed rate is what broke IPMA_Data")

  def test_reports_nothing_while_disabled(self):
    CP, CP_SP = make_params(False)
    CS = CarState(CP, CP_SP)
    parsers = CarState.get_can_parsers(CP, CP_SP)
    self._feed(parsers, CP, 0, stop_light=1)
    _, ret_sp = CS.update(parsers)
    self.assertFalse(ret_sp.fordBrakeLights.dataAvailable)
    self.assertFalse(ret_sp.fordBrakeLights.brakeLightsOn)

  def test_follows_the_bcm_lamp_signal(self):
    CP, CP_SP = make_params(True)
    CS = CarState(CP, CP_SP)
    parsers = CarState.get_can_parsers(CP, CP_SP)
    t = 0
    for stop_light in (0, 1, 1, 0, 1, 0):
      t += 1_000_000_000
      self._feed(parsers, CP, t, stop_light=stop_light)
      _, ret_sp = CS.update(parsers)
      self.assertTrue(ret_sp.fordBrakeLights.dataAvailable)
      self.assertEqual(bool(stop_light), ret_sp.fordBrakeLights.brakeLightsOn)

  def test_bcm_wins_over_the_faster_but_dead_fallback(self):
    """Measured on a 2023 Raptor R: BrkLamp_B_Rq never goes high even under heavy braking, while
    the 1Hz BCM signal tracks the lamps. Rate alone would pick the wrong source."""
    CP, CP_SP = make_params(True)
    CS = CarState(CP, CP_SP)
    parsers = CarState.get_can_parsers(CP, CP_SP)
    self._feed(parsers, CP, 1_000_000_000, stop_light=1, brk_lamp=0)
    _, ret_sp = CS.update(parsers)
    self.assertTrue(ret_sp.fordBrakeLights.brakeLightsOn)
    self.assertEqual(CS.brake_lamp_source, "BCM_Lamp_Stat_FD1")

  def test_state_is_carried_between_the_slow_source_frames(self):
    """The BCM transmits about once a second; the readout must hold, not flicker off, in between."""
    CP, CP_SP = make_params(True)
    CS = CarState(CP, CP_SP)
    parsers = CarState.get_can_parsers(CP, CP_SP)
    self._feed(parsers, CP, 1_000_000_000, stop_light=1)
    for _ in range(50):   # half a second of 100Hz updates with no new lamp frame
      _, ret_sp = CS.update(parsers)
      self.assertTrue(ret_sp.fordBrakeLights.brakeLightsOn)


if __name__ == "__main__":
  unittest.main()
