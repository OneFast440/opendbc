"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

End to end through the real Ford CarController: what actually lands on the wire has to match
what safety/modes/ford.h expects for the selected mode.
"""
import unittest
from collections import defaultdict

from opendbc.car.ford import fordcan
from opendbc.car.ford.values import CAR, FordFlags
from opendbc.sunnypilot.car.ford.fordcan_ext import SHADOW_CURVATURE_SCALE
from opendbc.sunnypilot.car.ford.hud_ext import HANDS_ON, HANDS_WARN_CHIME, TJA_WARN_RESUME_CONTROL
from opendbc.sunnypilot.car.ford.tests.helpers import (
  INACTIVE_CURVATURE,
  INACTIVE_PATH_ANGLE,
  INACTIVE_PATH_OFFSET,
  MSG_ACCDATA,
  MSG_ACCDATA_3,
  MSG_IPMA_Data,
  MSG_Lane_Assist_Data1,
  MSG_LateralMotionControl,
  MSG_LateralMotionControl2,
  MSG_Steering_Data_FD1,
  SendButtonState,
  VisualAlert,
  make_car_controller,
  make_cc,
  make_cc_sp,
  make_cs,
  make_lead,
  unpack_lat_ctl,
)
from opendbc.sunnypilot.car.ford.values_ext import PrimaryLateralControl

PLATFORMS = (CAR.FORD_F_150_MK14, CAR.FORD_ESCAPE_MK4)  # CAN FD and CAN


def drive(controller, CP, frames=40, cc=None, cc_sp=None, cs=None):
  sends = []
  for _ in range(frames):
    _, can_sends = controller.update(cc or make_cc(), cc_sp or make_cc_sp(), cs or make_cs(), 0)
    sends.extend(can_sends)
  return sends


def msgs(sends, addr):
  return [m for m in sends if m[0] == addr]


def steer_addr(CP):
  return MSG_LateralMotionControl2 if CP.flags & FordFlags.CANFD else MSG_LateralMotionControl


class TestFordCarControllerLateral(unittest.TestCase):
  def test_stock_mode_sends_curvature_only(self):
    for platform in PLATFORMS:
      with self.subTest(platform=platform):
        controller, CP = make_car_controller(platform, mode=PrimaryLateralControl.stock)
        sends = drive(controller, CP, cc=make_cc(curvature=0.002))
        addr = steer_addr(CP)
        self.assertTrue(msgs(sends, addr))
        for _, dat, _ in msgs(sends, addr):
          sig = unpack_lat_ctl(addr, dat)
          self.assertEqual(sig['path_angle'], INACTIVE_PATH_ANGLE)
          self.assertEqual(sig['path_offset'], INACTIVE_PATH_OFFSET)
          self.assertEqual(sig['curvature_rate'], sig['inactive_curvature_rate'])
        # and the LKA message stays empty
        for _, dat, _ in msgs(sends, MSG_Lane_Assist_Data1):
          self.assertEqual(bytes(dat), b"\x00" * 8)

  def test_angle_mode_sends_path_angle_only(self):
    for platform in PLATFORMS:
      with self.subTest(platform=platform):
        controller, CP = make_car_controller(platform, mode=PrimaryLateralControl.angle)
        sends = drive(controller, CP, cc=make_cc(curvature=0.002))
        addr = steer_addr(CP)
        self.assertTrue(msgs(sends, addr))
        for _, dat, _ in msgs(sends, addr):
          sig = unpack_lat_ctl(addr, dat)
          self.assertEqual(sig['curvature'], INACTIVE_CURVATURE)
          self.assertEqual(sig['curvature_rate'], sig['inactive_curvature_rate'])
          self.assertEqual(sig['path_offset'], INACTIVE_PATH_OFFSET)
          self.assertNotEqual(sig['path_angle'], INACTIVE_PATH_ANGLE)

  def test_curvature_mode_sends_curvature_and_trim(self):
    for platform in PLATFORMS:
      with self.subTest(platform=platform):
        controller, CP = make_car_controller(platform, mode=PrimaryLateralControl.curvature,
                                             lane_positioning=True, custom_profile=1)
        cc_sp = make_cc_sp(model_curvature=0.004, model_position_y=[0.5] * 33)
        sends = drive(controller, CP, cc=make_cc(curvature=0.004), cc_sp=cc_sp,
                      cs=make_cs(v_ego=12.0))
        addr = steer_addr(CP)
        sig = [unpack_lat_ctl(addr, dat) for _, dat, _ in msgs(sends, addr)]
        self.assertTrue(sig)
        # c0 never leaves its sentinel, and by the end of the run c1 and c2 are both live
        for s in sig:
          self.assertEqual(s['path_offset'], INACTIVE_PATH_OFFSET)
        self.assertNotEqual(sig[-1]['curvature'], INACTIVE_CURVATURE)
        self.assertNotEqual(sig[-1]['path_angle'], INACTIVE_PATH_ANGLE)

  def test_lka_carries_the_shadow_only_in_angle_mode(self):
    for mode in PrimaryLateralControl:
      with self.subTest(mode=mode):
        controller, CP = make_car_controller(mode=mode)
        sends = drive(controller, CP, cc=make_cc(curvature=0.002), cs=make_cs(v_ego=25.0, yaw_rate=0.05))
        _, dat, _ = msgs(sends, MSG_Lane_Assist_Data1)[-1]
        if mode == PrimaryLateralControl.angle:
          self.assertEqual(dat[4] & 0x1, 1)
          shadow = int.from_bytes(dat[5:7], 'big', signed=True) * SHADOW_CURVATURE_SCALE
          # published in the CAN sign convention, i.e. negated from openpilot's
          self.assertAlmostEqual(shadow, -controller.lat_angle.shadow_curvature, places=5)
        else:
          self.assertEqual(bytes(dat), b"\x00" * 8)

  def test_hands_back_to_the_driver_drops_the_mode(self):
    """Both BluePilot strategies send mode 0 rather than freezing a command, so the panda needs
    no bypass for those frames."""
    for mode in (PrimaryLateralControl.curvature, PrimaryLateralControl.angle):
      with self.subTest(mode=mode):
        controller, CP = make_car_controller(mode=mode)
        addr = steer_addr(CP)
        cs = make_cs(v_ego=15.0, steering_pressed=True, steering_angle=60.0)
        # the wheel is already past the angle threshold when the press starts, which the detector
        # treats as a mid-curve nudge and holds longer before latching
        sends = drive(controller, CP, frames=500, cc=make_cc(curvature=0.01), cs=cs)
        last = unpack_lat_ctl(addr, msgs(sends, addr)[-1][1])
        self.assertFalse(last['enabled'])
        self.assertEqual(last['path_angle'], INACTIVE_PATH_ANGLE)
        self.assertEqual(last['curvature'], INACTIVE_CURVATURE)

  def test_reported_curvature_is_meaningful_in_every_mode(self):
    for mode in PrimaryLateralControl:
      with self.subTest(mode=mode):
        controller, CP = make_car_controller(mode=mode)
        out = None
        for _ in range(20):
          out, _ = controller.update(make_cc(curvature=0.004), make_cc_sp(),
                                     make_cs(v_ego=20.0, yaw_rate=0.02), 0)
        self.assertNotEqual(out.curvature, 0.0)


class TestFordCarControllerLongitudinal(unittest.TestCase):
  def test_acc_message_only_with_openpilot_longitudinal(self):
    controller, CP = make_car_controller(alpha_long=False)
    self.assertFalse(CP.openpilotLongitudinalControl)
    self.assertFalse(msgs(drive(controller, CP), MSG_ACCDATA))

    controller, CP = make_car_controller(alpha_long=True)
    self.assertTrue(CP.openpilotLongitudinalControl)
    self.assertTrue(msgs(drive(controller, CP, cc=make_cc(long_active=True)), MSG_ACCDATA))

  def test_follow_control_reaches_the_wire(self):
    controller, CP = make_car_controller(alpha_long=True, follow_control=True)
    lead = make_lead(status=True, d_rel=27.0, v_rel=-3.0, v_lead=27.0)
    cc = make_cc(long_active=True, accel=0.5)
    sends = drive(controller, CP, cc=cc, cc_sp=make_cc_sp(lead=lead), cs=make_cs(v_ego=27.0))
    self.assertTrue(msgs(sends, MSG_ACCDATA))
    # closing inside 1.5 s of headway: gas is cut to zero rather than trimmed
    self.assertEqual(controller.gas, 0.0)


class TestFordCarControllerHud(unittest.TestCase):
  def test_cluster_messages_are_sent(self):
    controller, CP = make_car_controller()
    sends = drive(controller, CP, frames=120)
    self.assertTrue(msgs(sends, MSG_IPMA_Data))
    self.assertTrue(msgs(sends, MSG_ACCDATA_3))

  def test_steer_required_raises_the_silent_prompt(self):
    controller, CP = make_car_controller()
    drive(controller, CP, frames=5, cc=make_cc(visual_alert=VisualAlert.steerRequired))
    self.assertNotEqual(controller.hud.hands, HANDS_ON)

  def test_driver_monitoring_drives_the_cluster_when_enabled(self):
    controller, CP = make_car_controller(driver_monitor_cluster=True)
    cc_sp = make_cc_sp(alert_type="promptDriverDistracted/none")
    drive(controller, CP, frames=40, cc_sp=cc_sp)
    self.assertEqual(controller.hud.hands, HANDS_WARN_CHIME)

    controller, CP = make_car_controller(driver_monitor_cluster=True)
    cc_sp = make_cc_sp(alert_type="none/softDisable")
    drive(controller, CP, frames=40, cc_sp=cc_sp)
    self.assertEqual(controller.hud.tja_warn, TJA_WARN_RESUME_CONTROL)

  def test_hands_free_cluster_is_can_fd_only(self):
    _, _ = make_car_controller(CAR.FORD_F_150_MK14, hands_free_cluster=True)
    canfd, _ = make_car_controller(CAR.FORD_F_150_MK14, hands_free_cluster=True)
    can, _ = make_car_controller(CAR.FORD_ESCAPE_MK4, hands_free_cluster=True)
    self.assertTrue(canfd.hud.hands_free_cluster)
    self.assertFalse(can.hud.hands_free_cluster)


class TestFordCarControllerIcbm(unittest.TestCase):
  @staticmethod
  def _expected(controller, signal):
    _, dat, _ = fordcan.create_button_msg(controller.packer, controller.CAN.main,
                                          defaultdict(int), icbm_button=signal)
    return bytes(dat)

  def test_button_press_is_injected(self):
    for button, signal in ((SendButtonState.increase, "CcAslButtnSetIncPress"),
                           (SendButtonState.decrease, "CcAslButtnSetDecPress")):
      with self.subTest(button=button):
        controller, CP = make_car_controller()
        sends = drive(controller, CP, frames=40, cc_sp=make_cc_sp(send_button=button))
        sent = {bytes(dat) for _, dat, _ in msgs(sends, MSG_Steering_Data_FD1)}
        self.assertTrue(self._expected(controller, signal) in sent, f"{signal} was not sent")

  def test_nothing_is_injected_when_idle(self):
    controller, CP = make_car_controller()
    sends = drive(controller, CP, frames=40, cc_sp=make_cc_sp(send_button=SendButtonState.none))
    sent = {bytes(dat) for _, dat, _ in msgs(sends, MSG_Steering_Data_FD1)}
    for signal in ("CcAslButtnSetIncPress", "CcAslButtnSetDecPress"):
      self.assertNotIn(self._expected(controller, signal), sent)


if __name__ == "__main__":
  unittest.main()
