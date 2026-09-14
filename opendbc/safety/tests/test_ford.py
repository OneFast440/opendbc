#!/usr/bin/env python3
import numpy as np
import random
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.lateral import MAX_LATERAL_ACCEL, MAX_LATERAL_JERK
from opendbc.safety.tests.common import RT_INTERVAL
from opendbc.car.ford.values import FordSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.ford.fordcan_ext import SHADOW_CURVATURE_SCALE
from opendbc.sunnypilot.car.ford.values_ext import (
  CURV_MODE_PATH_ANGLE_MAX,
  FORD_DBC_PATH_ANGLE_MAX,
  FORD_DBC_PATH_ANGLE_MIN,
  PrimaryLateralControl,
)
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety

MSG_BrakeSysFeatures = 0x415       # RX from ABS, for vehicle speed
MSG_EngVehicleSpThrottle2 = 0x202  # RX from PCM, for second vehicle speed
MSG_Yaw_Data_FD1 = 0x91            # RX from RCM, for yaw rate
MSG_Steering_Data_FD1 = 0x083      # TX by OP, various driver switches and LKAS/CC buttons
MSG_ACCDATA = 0x186                # TX by OP, ACC controls
MSG_ACCDATA_3 = 0x18A              # TX by OP, ACC/TJA user interface
MSG_Lane_Assist_Data1 = 0x3CA      # TX by OP, Lane Keep Assist
MSG_LateralMotionControl = 0x3D3   # TX by OP, Lateral Control message
MSG_LateralMotionControl2 = 0x3D6  # TX by OP, alternate Lateral Control message
MSG_IPMA_Data = 0x3D8              # TX by OP, IPMA and LKAS user interface


def checksum(msg):
  addr, dat, bus = msg
  ret = bytearray(dat)

  if addr == MSG_Yaw_Data_FD1:
    chksum = dat[0] + dat[1]  # VehRol_W_Actl
    chksum += dat[2] + dat[3]  # VehYaw_W_Actl
    chksum += dat[5]  # VehRollYaw_No_Cnt
    chksum += dat[6] >> 6  # VehRolWActl_D_Qf
    chksum += (dat[6] >> 4) & 0x3  # VehYawWActl_D_Qf
    chksum = 0xff - (chksum & 0xff)
    ret[4] = chksum

  elif addr == MSG_BrakeSysFeatures:
    chksum = dat[0] + dat[1]  # Veh_V_ActlBrk
    chksum += (dat[2] >> 2) & 0xf  # VehVActlBrk_No_Cnt
    chksum += dat[2] >> 6  # VehVActlBrk_D_Qf
    chksum = 0xff - (chksum & 0xff)
    ret[3] = chksum

  elif addr == MSG_EngVehicleSpThrottle2:
    chksum = (dat[2] >> 3) & 0xf  # VehVActlEng_No_Cnt
    chksum += (dat[4] >> 5) & 0x3  # VehVActlEng_D_Qf
    chksum += dat[6] + dat[7]  # Veh_V_ActlEng
    chksum = 0xff - (chksum & 0xff)
    ret[1] = chksum

  return addr, ret, bus


class Buttons:
  CANCEL = 0
  RESUME = 1
  TJA_TOGGLE = 2


# Ford safety has four different configurations tested here:
#  * CAN with openpilot longitudinal
#  * CAN FD with stock longitudinal
#  * CAN FD with openpilot longitudinal

class TestFordSafetyBase(common.CarSafetyTest):
  STANDSTILL_THRESHOLD = 1
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                                 MSG_LateralMotionControl2, MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                               MSG_LateralMotionControl2, MSG_IPMA_Data]}

  STEER_MESSAGE = 0

  # Curvature control limits
  DEG_TO_CAN = 50000   # CAN units per rad/m
  MAX_CURVATURE = 0.02 # rad/m, 1000 CAN units
  MAX_CURVATURE_ERROR = 0.002         # rad/m, 100 CAN units
  CURVATURE_ERROR_MIN_SPEED = 10.0    # m/s
  LATERAL_FREQUENCY = 20              # Hz, for per-frame jerk limit

  cnt_speed = 0
  cnt_speed_2 = 0
  cnt_yaw_rate = 0
  cnt_lat_ctl = 0

  packer: CANPackerSafety
  safety: libsafety_py.LibSafety

  def _get_max_curvature_can(self, speed):
    fudged_speed = max(speed - 1.0, 1.0)
    return int(MAX_LATERAL_ACCEL / (fudged_speed * fudged_speed) * self.DEG_TO_CAN) + 1

  def _get_max_curvature_delta_can(self, speed):
    fudged_speed = max(speed - 1.0, 1.0)
    return int(MAX_LATERAL_JERK / (fudged_speed * fudged_speed) / self.LATERAL_FREQUENCY * self.DEG_TO_CAN) + 1

  def _get_max_curvature_delta_relaxed_can(self, speed):
    # flipped fudge, this is the least movement toward the error bounds safety requires
    fudged_speed = speed + 1.0
    return int(MAX_LATERAL_JERK / (fudged_speed * fudged_speed) / self.LATERAL_FREQUENCY * self.DEG_TO_CAN) - 1

  def _get_max_curvature_relaxed_can(self, speed):
    # flipped fudge, safety never requires commanding more curvature than openpilot can send
    fudged_speed = speed + 1.0
    max_curvature_accel_can = int(MAX_LATERAL_ACCEL / (fudged_speed * fudged_speed) * self.DEG_TO_CAN) - 1
    return min(max_curvature_accel_can, round(self.MAX_CURVATURE * self.DEG_TO_CAN))

  def _set_prev_desired_angle(self, t):
    t = round(t * self.DEG_TO_CAN)
    self.safety.set_desired_curvature_last(t)

  def _reset_curvature_measurement(self, curvature, speed):
    for _ in range(6):
      self._rx(self._speed_msg(speed))
      self._rx(self._speed_msg_2(speed))
      self._rx(self._yaw_rate_msg(curvature, speed))

  # Driver brake pedal
  def _user_brake_msg(self, brake: bool):
    # brake pedal and cruise state share same message, so we have to send
    # the other signal too
    enable = self.safety.get_controls_allowed()
    values = {
      "BpedDrvAppl_D_Actl": 2 if brake else 1,
      "CcStat_D_Actl": 5 if enable else 0,
    }
    return self.packer.make_can_msg_safety("EngBrakeData", 0, values)

  # ABS vehicle speed
  def _speed_msg(self, speed: float, quality_flag=True):
    values = {"Veh_V_ActlBrk": speed * 3.6, "VehVActlBrk_D_Qf": 3 if quality_flag else 0, "VehVActlBrk_No_Cnt": self.cnt_speed % 16}
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("BrakeSysFeatures", 0, values, fix_checksum=checksum)

  # PCM vehicle speed
  def _speed_msg_2(self, speed: float, quality_flag=True):
    # Ford relies on speed for driver curvature limiting, so it checks two sources
    values = {"Veh_V_ActlEng": speed * 3.6, "VehVActlEng_D_Qf": 3 if quality_flag else 0, "VehVActlEng_No_Cnt": self.cnt_speed_2 % 16}
    self.__class__.cnt_speed_2 += 1
    return self.packer.make_can_msg_safety("EngVehicleSpThrottle2", 0, values, fix_checksum=checksum)

  # Standstill state
  def _vehicle_moving_msg(self, speed: float):
    values = {"VehStop_D_Stat": 1 if speed <= self.STANDSTILL_THRESHOLD else random.choice((0, 2, 3))}
    return self.packer.make_can_msg_safety("DesiredTorqBrk", 0, values)

  # Current curvature
  def _yaw_rate_msg(self, curvature: float, speed: float, quality_flag=True):
    values = {"VehYaw_W_Actl": curvature * speed, "VehYawWActl_D_Qf": 3 if quality_flag else 0,
              "VehRollYaw_No_Cnt": self.cnt_yaw_rate % 256}
    self.__class__.cnt_yaw_rate += 1
    return self.packer.make_can_msg_safety("Yaw_Data_FD1", 0, values, fix_checksum=checksum)

  # Drive throttle input
  def _user_gas_msg(self, gas: float):
    values = {"ApedPos_Pc_ActlArb": gas}
    return self.packer.make_can_msg_safety("EngVehicleSpThrottle", 0, values)

  # Cruise status
  def _pcm_status_msg(self, enable: bool):
    # brake pedal and cruise state share same message, so we have to send
    # the other signal too
    brake = self.safety.get_brake_pressed_prev()
    values = {
      "BpedDrvAppl_D_Actl": 2 if brake else 1,
      "CcStat_D_Actl": 5 if enable else 0,
    }
    return self.packer.make_can_msg_safety("EngBrakeData", 0, values)

  # LKAS command
  def _lkas_command_msg(self, action: int):
    values = {
      "LkaActvStats_D2_Req": action,
    }
    return self.packer.make_can_msg_safety("Lane_Assist_Data1", 0, values)

  # LCA command
  def _lat_ctl_msg(self, enabled: bool, path_offset: float, path_angle: float, curvature: float, curvature_rate: float,
                   increment_timer: bool = True):
    if increment_timer:
      self.safety.set_timer(self.cnt_lat_ctl * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_lat_ctl += 1
    if self.STEER_MESSAGE == MSG_LateralMotionControl:
      values = {
        "LatCtl_D_Rq": 1 if enabled else 0,
        "LatCtlPathOffst_L_Actl": path_offset,     # Path offset [-5.12|5.11] meter
        "LatCtlPath_An_Actl": path_angle,          # Path angle [-0.5|0.5235] radians
        "LatCtlCurv_NoRate_Actl": curvature_rate,  # Curvature rate [-0.001024|0.00102375] 1/meter^2
        "LatCtlCurv_No_Actl": curvature,           # Curvature [-0.02|0.02094] 1/meter
      }
      return self.packer.make_can_msg_safety("LateralMotionControl", 0, values)
    elif self.STEER_MESSAGE == MSG_LateralMotionControl2:
      values = {
        "LatCtl_D2_Rq": 1 if enabled else 0,
        "LatCtlPathOffst_L_Actl": path_offset,     # Path offset [-5.12|5.11] meter
        "LatCtlPath_An_Actl": path_angle,          # Path angle [-0.5|0.5235] radians
        "LatCtlCrv_NoRate2_Actl": curvature_rate,  # Curvature rate [-0.001024|0.001023] 1/meter^2
        "LatCtlCurv_No_Actl": curvature,           # Curvature [-0.02|0.02094] 1/meter
      }
      return self.packer.make_can_msg_safety("LateralMotionControl2", 0, values)

  # Cruise control buttons
  def _acc_button_msg(self, button: int, bus: int):
    values = {
      "CcAslButtnCnclPress": 1 if button == Buttons.CANCEL else 0,
      "CcAsllButtnResPress": 1 if button == Buttons.RESUME else 0,
      "TjaButtnOnOffPress": 1 if button == Buttons.TJA_TOGGLE else 0,
    }
    return self.packer.make_can_msg_safety("Steering_Data_FD1", bus, values)

  def test_rx_hook_speed_mismatch(self):
    for speed in np.arange(0, 40, 0.5):
      for speed_delta in np.arange(-5, 5, 0.1):
        speed_2 = round(max(speed + speed_delta, 0), 1)
        self._rx(self._speed_msg(speed))
        self._rx(self._speed_msg_2(speed_2))
        self.safety.set_controls_allowed(True)
        self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0))

        within_delta = abs(speed - speed_2) <= common.MAX_SPEED_DELTA
        self.assertEqual(self.safety.get_controls_allowed(), within_delta)

  def test_rx_hook(self):
    # checksum, counter, and quality flag checks
    for quality_flag in [True, False]:
      for msg_type in ["speed", "speed_2", "yaw"]:
        self.safety.set_controls_allowed(True)
        # send multiple times to verify counter checks
        for _ in range(10):
          if msg_type == "speed":
            msg = self._speed_msg(0, quality_flag=quality_flag)
          elif msg_type == "speed_2":
            msg = self._speed_msg_2(0, quality_flag=quality_flag)
          elif msg_type == "yaw":
            msg = self._yaw_rate_msg(0, 0, quality_flag=quality_flag)

          self.assertEqual(quality_flag, self._rx(msg))
          self.assertEqual(quality_flag, self.safety.get_controls_allowed())

        # Mess with checksum to make it fail, checksum is not checked for 2nd speed
        msg[0].data[3] = 0  # Speed checksum & half of yaw signal
        should_rx = msg_type == "speed_2" and quality_flag
        self.assertEqual(should_rx, self._rx(msg))
        self.assertEqual(should_rx, self.safety.get_controls_allowed())

  def test_angle_measurements(self):
    """Tests rx hook correctly parses the curvature measurement from the vehicle speed and yaw rate"""
    for speed in np.arange(0.5, 40, 0.5):
      for curvature in np.arange(0, self.MAX_CURVATURE * 2, 2e-3):
        self._rx(self._speed_msg(speed))
        for c in (curvature, -curvature, 0, 0, 0, 0):
          self._rx(self._yaw_rate_msg(c, speed))

        self.assertEqual(self.safety.get_curvature_meas_min(), round(-curvature * self.DEG_TO_CAN))
        self.assertEqual(self.safety.get_curvature_meas_max(), round(curvature * self.DEG_TO_CAN))

        self._rx(self._yaw_rate_msg(0, speed))
        self.assertEqual(self.safety.get_curvature_meas_min(), round(-curvature * self.DEG_TO_CAN))
        self.assertEqual(self.safety.get_curvature_meas_max(), 0)

        self._rx(self._yaw_rate_msg(0, speed))
        self.assertEqual(self.safety.get_curvature_meas_min(), 0)
        self.assertEqual(self.safety.get_curvature_meas_max(), 0)

  def test_max_lateral_acceleration(self):
    # Ford CAN FD can achieve a higher max lateral acceleration than CAN so we limit curvature based on speed
    max_curvature_can = round(self.MAX_CURVATURE * self.DEG_TO_CAN)
    for speed in np.arange(0, 40, 0.5):
      max_can = min(self._get_max_curvature_can(speed), max_curvature_can)
      for offset in (-5, -1, 0, 1, 5):
        curvature_can = max_can + offset
        curvature = curvature_can / self.DEG_TO_CAN

        for sign in (-1, 1):
          signed_curvature = sign * curvature
          self.safety.set_controls_allowed(True)
          self._set_prev_desired_angle(signed_curvature)
          self._reset_curvature_measurement(signed_curvature, speed)

          should_tx = abs(curvature_can) <= max_can
          self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(True, 0, 0, signed_curvature, 0)))

  def test_steer_allowed(self):
    path_offsets = np.arange(-5.12, 5.11, 2.5).round()
    path_angles = np.arange(-0.5, 0.5235, 0.25).round(1)
    curvature_rates = np.arange(-0.001024, 0.00102375, 0.001).round(3)
    curvatures = np.arange(-0.02, 0.02094, 0.01).round(2)

    for speed in (self.CURVATURE_ERROR_MIN_SPEED - 1,
                  self.CURVATURE_ERROR_MIN_SPEED + 1):
      max_curvature_can = self._get_max_curvature_can(speed)
      for controls_allowed in (True, False):
        for steer_control_enabled in (True, False):
          for path_offset in path_offsets:
            for path_angle in path_angles:
              for curvature_rate in curvature_rates:
                for curvature in curvatures:
                  self.safety.set_controls_allowed(controls_allowed)
                  self._set_prev_desired_angle(curvature)
                  self._reset_curvature_measurement(curvature, speed)

                  should_tx = path_offset == 0 and path_angle == 0 and curvature_rate == 0
                  # when request bit is 0, only allow curvature of 0 since the signal range
                  # is not large enough to enforce it tracking measured
                  should_tx = should_tx and (controls_allowed if steer_control_enabled else curvature == 0)
                  should_tx = should_tx and abs(round(curvature * self.DEG_TO_CAN)) <= max_curvature_can

                  with self.subTest(controls_allowed=controls_allowed, steer_control_enabled=steer_control_enabled,
                                    path_offset=float(path_offset), path_angle=float(path_angle), curvature_rate=float(curvature_rate),
                                    curvature=float(curvature)):
                    self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(steer_control_enabled, path_offset, path_angle, curvature, curvature_rate)))

  def test_curvature_rate_limits(self):
    """
    When the curvature error is exceeded, commanded curvature must start moving towards meas respecting rate limits.
    Since safety allows higher rate limits to avoid false positives, we need to allow a lower rate to move towards meas.
    """
    self.safety.set_controls_allowed(True)
    # safety fudges the speed (1 m/s) and rate limits (1 CAN unit) to avoid false positives
    small_curvature = 1 / self.DEG_TO_CAN  # significant small amount of curvature to cross boundary

    for speed in np.arange(0, 40, 0.5):
      curvature_accel_limit = self._get_max_curvature_can(speed) / self.DEG_TO_CAN
      limit_command = speed > self.CURVATURE_ERROR_MIN_SPEED
      # ensure our limits match the safety's rounded limits
      # lateral jerk is symmetric, so the wind up and wind down limits are the same
      max_delta = self._get_max_curvature_delta_can(speed) / self.DEG_TO_CAN
      max_delta_relaxed = self._get_max_curvature_delta_relaxed_can(speed) / self.DEG_TO_CAN

      up_cases = (self.MAX_CURVATURE_ERROR * 2, [
        (not limit_command, 0, 0),
        (not limit_command, 0, max_delta_relaxed - small_curvature),
        (True, 0, max_delta_relaxed),
        (True, 0, max_delta),
        (False, 0, max_delta + small_curvature),
        # stay at boundary limit
        (True, self.MAX_CURVATURE_ERROR - small_curvature, self.MAX_CURVATURE_ERROR - small_curvature),
        # 1 unit below boundary limit
        (not limit_command, self.MAX_CURVATURE_ERROR - small_curvature * 2, self.MAX_CURVATURE_ERROR - small_curvature * 2),
        # shouldn't allow command to move outside the boundary limit if last was inside
        (not limit_command, self.MAX_CURVATURE_ERROR - small_curvature, self.MAX_CURVATURE_ERROR - small_curvature * 2),
      ])

      down_cases = (self.MAX_CURVATURE - self.MAX_CURVATURE_ERROR * 2, [
        (not limit_command, self.MAX_CURVATURE, self.MAX_CURVATURE),
        (not limit_command, self.MAX_CURVATURE, self.MAX_CURVATURE - max_delta_relaxed + small_curvature),
        (True, self.MAX_CURVATURE, self.MAX_CURVATURE - max_delta_relaxed),
        (True, self.MAX_CURVATURE, self.MAX_CURVATURE - max_delta),
        (False, self.MAX_CURVATURE, self.MAX_CURVATURE - max_delta - small_curvature),
      ])

      # the driver can hold a curvature openpilot may not command, safety must never require moving past
      # the most it can send: the lower of the lateral acceleration limit and what the EPS accepts
      max_curvature_relaxed = self._get_max_curvature_relaxed_can(speed) / self.DEG_TO_CAN
      # safety fudges the speed down for the accel check and up for the cap, so the last command can sit above the cap
      max_curvature_allowed_can = min(self._get_max_curvature_can(speed), round(self.MAX_CURVATURE * self.DEG_TO_CAN))
      winds_down_within_jerk = (max_curvature_allowed_can - self._get_max_curvature_relaxed_can(speed) <=
                                self._get_max_curvature_delta_can(speed))
      relaxed_cases = (self.MAX_CURVATURE * 2, [
        (True, max_curvature_relaxed, max_curvature_relaxed),
        (not limit_command, max_curvature_relaxed, max_curvature_relaxed - small_curvature),
        # no longer requiring the command to wind towards meas doesn't stop rate limiting it winding away
        (winds_down_within_jerk, max_curvature_allowed_can / self.DEG_TO_CAN, max_curvature_relaxed),
      ])

      for sign in (-1, 1):
        for angle_meas, cases in (up_cases, down_cases, relaxed_cases):
          self._reset_curvature_measurement(sign * angle_meas, speed)
          for should_tx, initial_curvature, desired_curvature in cases:

            # at low speeds one frame of jerk exceeds the curvature signal, so the should_tx=False cases will rightly not fail.
            # assert we never drop a case at a speed where the curvature error is enforced
            if abs(desired_curvature) > self.MAX_CURVATURE:
              self.assertLess(speed, self.CURVATURE_ERROR_MIN_SPEED)
              continue

            # can not send if the curvature is above the max lateral acceleration
            should_tx = should_tx and abs(desired_curvature) <= curvature_accel_limit

            self._set_prev_desired_angle(sign * initial_curvature)
            self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(True, 0, 0, sign * desired_curvature, 0)))

    # the newest speed sample gates the check, not the whole sample window
    max_error = self.MAX_CURVATURE_ERROR + small_curvature * 2
    for sign in (-1, 1):
      self._reset_curvature_measurement(0, self.CURVATURE_ERROR_MIN_SPEED - 1)
      self._rx(self._speed_msg(self.CURVATURE_ERROR_MIN_SPEED + 1))
      self._set_prev_desired_angle(sign * self.MAX_CURVATURE_ERROR)
      self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, sign * max_error, 0)))

  def test_curvature_violation(self):
    # If violation occurs, curvature cmd is blocked until reset to 0
    self.safety.set_controls_allowed(True)
    speed = 25.
    max_delta_can = self._get_max_curvature_delta_can(speed)
    self._reset_curvature_measurement(0, speed)

    self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0))
    over_curvature = (max_delta_can + 5) / self.DEG_TO_CAN
    for _ in range(20):
      self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, over_curvature, 0)))
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0)))

    # prev tracks the commanded curvature on a passing tx (not reset to 0 every frame)
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, max_delta_can / self.DEG_TO_CAN, 0)))
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 2 * max_delta_can / self.DEG_TO_CAN, 0)))

  def test_rt_limits(self):
    # send rate is limited over a rolling 250ms window split into two half-interval buckets
    self.safety.set_controls_allowed(True)
    self._reset_curvature_measurement(0, 0)
    max_rt_msgs = int(self.LATERAL_FREQUENCY * common.RT_INTERVAL / 1e6 * 1.2 + 1)
    half = common.RT_INTERVAL // 2

    # too many messages within one window is blocked
    self.safety.set_timer(0)
    for i in range(max_rt_msgs * 2):
      self.assertEqual(i <= max_rt_msgs, self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0, increment_timer=False)))

    # shift the overflow into the previous bucket
    self.safety.set_timer(half)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0, increment_timer=False)))

    # previous bucket still counts within the half interval
    self.safety.set_timer(half + 2 * common.RT_INTERVAL // 5)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0, increment_timer=False)))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0, increment_timer=False)))

    # both buckets clear after a full interval
    self.safety.set_timer(half + common.RT_INTERVAL)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0, increment_timer=False)))
    self.safety.set_timer(half + 2 * common.RT_INTERVAL)
    for _ in range(max_rt_msgs):
      self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0, increment_timer=False)))

  def test_prevent_lkas_action(self):
    self.safety.set_controls_allowed(1)
    self.assertFalse(self._tx(self._lkas_command_msg(1)))

    self.safety.set_controls_allowed(0)
    self.assertFalse(self._tx(self._lkas_command_msg(1)))

  def test_acc_buttons(self):
    for allowed in (0, 1):
      self.safety.set_controls_allowed(allowed)
      for enabled in (True, False):
        self._rx(self._pcm_status_msg(enabled))
        self.assertTrue(self._tx(self._acc_button_msg(Buttons.TJA_TOGGLE, 2)))

    for allowed in (0, 1):
      self.safety.set_controls_allowed(allowed)
      for bus in (0, 2):
        self.assertEqual(allowed, self._tx(self._acc_button_msg(Buttons.RESUME, bus)))

    for enabled in (True, False):
      self._rx(self._pcm_status_msg(enabled))
      for bus in (0, 2):
        self.assertEqual(enabled, self._tx(self._acc_button_msg(Buttons.CANCEL, bus)))

  def test_enable_control_allowed_from_acc_main_on(self):
    for enable_mads in (True, False):
      with self.subTest("enable_mads", mads_enabled=enable_mads):
        for main_button_msg_valid in (True, False):
          with self.subTest("main_button_msg_valid", state_valid=main_button_msg_valid):
            self.safety.set_mads_params(enable_mads, False, False)
            self._rx(self._pcm_status_msg(main_button_msg_valid))
            self.assertEqual(enable_mads and main_button_msg_valid, self.safety.get_controls_allowed_lateral())


class TestFordCANFDStockSafety(TestFordSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl2

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl2, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                               MSG_IPMA_Data]}

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.CANFD)
    self.safety.init_tests()


class TestFordLongitudinalSafetyBase(TestFordSafetyBase):
  MAX_ACCEL = 2.0  # accel is used for brakes, but openpilot can set positive values
  MIN_ACCEL = -3.5
  INACTIVE_ACCEL = 0.0

  MAX_GAS = 2.0
  MIN_GAS = -0.5
  INACTIVE_GAS = -5.0

  # ACC command
  def _acc_command_msg(self, gas: float, brake: float, brake_actuation: bool, cmbb_deny: bool = False):
    values = {
      "AccPrpl_A_Rq": gas,                              # [-5|5.23] m/s^2
      "AccPrpl_A_Pred": gas,                            # [-5|5.23] m/s^2
      "AccBrkTot_A_Rq": brake,                          # [-20|11.9449] m/s^2
      "AccBrkPrchg_B_Rq": 1 if brake_actuation else 0,  # Pre-charge brake request: 0=No, 1=Yes
      "AccBrkDecel_B_Rq": 1 if brake_actuation else 0,  # Deceleration request: 0=Inactive, 1=Active
      "CmbbDeny_B_Actl": 1 if cmbb_deny else 0,         # [0|1] deny AEB actuation
    }
    return self.packer.make_can_msg_safety("ACCDATA", 0, values)

  def test_stock_aeb(self):
    # Test that CmbbDeny_B_Actl is never 1, it prevents the ABS module from actuating AEB requests from ACCDATA_2
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for cmbb_deny in (True, False):
        should_tx = not cmbb_deny
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.INACTIVE_GAS, self.INACTIVE_ACCEL, controls_allowed, cmbb_deny)))
        should_tx = controls_allowed and not cmbb_deny
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.MAX_GAS, self.MAX_ACCEL, controls_allowed, cmbb_deny)))

  def test_gas_safety_check(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for gas in np.concatenate((np.arange(self.MIN_GAS - 2, self.MAX_GAS + 2, 0.05), [self.INACTIVE_GAS])):
        gas = round(gas, 2)  # floats might not hit exact boundary conditions without rounding
        should_tx = (controls_allowed and self.MIN_GAS <= gas <= self.MAX_GAS) or gas == self.INACTIVE_GAS
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(gas, self.INACTIVE_ACCEL, controls_allowed)))

  def test_brake_safety_check(self):
    brake_values = self._boundary_values([self.MIN_ACCEL, self.MAX_ACCEL, self.INACTIVE_ACCEL],
                                         self.MIN_ACCEL - 2, self.MAX_ACCEL + 2, 0.05)
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for brake_actuation in (True, False):
        for brake in brake_values:
          should_tx = (controls_allowed and self.MIN_ACCEL <= brake <= self.MAX_ACCEL) or brake == self.INACTIVE_ACCEL
          should_tx = should_tx and (controls_allowed or not brake_actuation)
          self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.INACTIVE_GAS, brake, brake_actuation)))


class TestFordStockSafety(TestFordSafetyBase):
  """CAN vehicle on Ford's own ACC. Upstream always allowed ACCDATA here; sunnypilot follows the
  alpha longitudinal toggle, so with it off the message is not in the allowlist."""
  STEER_MESSAGE = MSG_LateralMotionControl

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl, MSG_IPMA_Data)}
  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl, MSG_IPMA_Data]}

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, 0)
    self.safety.init_tests()


class TestFordLongitudinalSafety(TestFordLongitudinalSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA, 0], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                               MSG_IPMA_Data]}

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()


class TestFordCANFDLongitudinalSafety(TestFordLongitudinalSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl2

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA, 0], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl2, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                               MSG_IPMA_Data]}

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.LONG_CONTROL | FordSafetyFlags.CANFD)
    self.safety.init_tests()


# *** sunnypilot: BluePilot lateral control ***
#
# A separate branch in ford.h, reached only when the SP safety param selects it. The stock path
# above is unchanged and is covered by every test in this file; these cover the two BluePilot
# modes. Curvature mode additionally runs the whole stock suite, at the bottom of this file,
# because c2 is still the actuator there and must keep every stock protection.

PATH_ANGLE_TO_CAN = 2000            # 1 / 0.0005 rad per LSB
PATH_ANGLE_ROC_BP = [10., 15., 25.]
PATH_ANGLE_ROC_V = [0.0561, 0.04335, 0.00918]


class FordBluePilotSafetyHarness(unittest.TestCase):
  """Shared setup for the two BluePilot lateral modes."""
  STEER_MESSAGE = 0
  SAFETY_PARAM = 0
  LATERAL_MODE = 0

  MAX_CURVATURE = 0.02
  MAX_CURVATURE_ERROR = 0.002
  CURVATURE_ERROR_MIN_SPEED = 10.0
  DEG_TO_CAN = 50000

  cnt_speed = 0
  cnt_speed_2 = 0
  cnt_yaw_rate = 0

  @classmethod
  def setUpClass(cls):
    if cls.__name__ in ("FordBluePilotSafetyHarness", "TestFordAngleControlSafetyBase",
                        "TestFordCurvatureControlSafetyBase"):
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_current_safety_param_sp(self.LATERAL_MODE)
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, self.SAFETY_PARAM)
    self.safety.init_tests()
    # init_tests zeroes the SP param; restore it so a mid-test _reset_safety_hooks keeps the mode
    self.safety.set_current_safety_param_sp(self.LATERAL_MODE)

  def _rx(self, msg):
    return self.safety.safety_rx_hook(msg)

  def _tx(self, msg):
    return self.safety.safety_tx_hook(msg)

  def _speed_msg(self, speed: float):
    values = {"Veh_V_ActlBrk": speed * 3.6, "VehVActlBrk_D_Qf": 3, "VehVActlBrk_No_Cnt": self.cnt_speed % 16}
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("BrakeSysFeatures", 0, values, fix_checksum=checksum)

  def _speed_msg_2(self, speed: float):
    values = {"Veh_V_ActlEng": speed * 3.6, "VehVActlEng_D_Qf": 3, "VehVActlEng_No_Cnt": self.cnt_speed_2 % 16}
    self.__class__.cnt_speed_2 += 1
    return self.packer.make_can_msg_safety("EngVehicleSpThrottle2", 0, values, fix_checksum=checksum)

  def _yaw_rate_msg(self, curvature: float, speed: float):
    values = {"VehYaw_W_Actl": curvature * speed, "VehYawWActl_D_Qf": 3,
              "VehRollYaw_No_Cnt": self.cnt_yaw_rate % 256}
    self.__class__.cnt_yaw_rate += 1
    return self.packer.make_can_msg_safety("Yaw_Data_FD1", 0, values, fix_checksum=checksum)

  def _set_speed_and_curvature(self, speed: float, curvature: float = 0.0):
    for _ in range(6):
      self._rx(self._speed_msg(speed))
      self._rx(self._speed_msg_2(speed))
      self._rx(self._yaw_rate_msg(curvature, speed))

  def _lka_msg(self, angle_mode_engaged: bool = True, shadow_curvature: float = 0.0, action: int = 0):
    """Lane_Assist_Data1 with the angle-control bits packed the way fordcan_ext.create_lka_msg does."""
    addr, dat, bus = self.packer.make_can_msg("Lane_Assist_Data1", 0, {"LkaActvStats_D2_Req": action})
    dat = bytearray(dat)
    raw = max(-32768, min(32767, int(round(shadow_curvature / SHADOW_CURVATURE_SCALE)))) & 0xFFFF
    dat[4] |= 1 if angle_mode_engaged else 0
    dat[5] = (raw >> 8) & 0xFF
    dat[6] = raw & 0xFF
    return libsafety_py.make_CANPacket(addr, bus, dat)

  cnt_lat_ctl = 0
  LATERAL_FREQUENCY = 20  # Hz

  def _lat_ctl_msg(self, enabled: bool, path_angle: float, path_offset: float = 0.0,
                   curvature: float = 0.0, curvature_rate: float = 0.0, increment_timer: bool = True):
    if increment_timer:
      self.safety.set_timer(self.cnt_lat_ctl * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_lat_ctl += 1
    if self.STEER_MESSAGE == MSG_LateralMotionControl:
      values = {
        "LatCtl_D_Rq": 1 if enabled else 0,
        "LatCtlPathOffst_L_Actl": path_offset,
        "LatCtlPath_An_Actl": path_angle,
        "LatCtlCurv_NoRate_Actl": curvature_rate,
        "LatCtlCurv_No_Actl": curvature,
      }
      return self.packer.make_can_msg_safety("LateralMotionControl", 0, values)
    values = {
      "LatCtl_D2_Rq": 1 if enabled else 0,
      "LatCtlPathOffst_L_Actl": path_offset,
      "LatCtlPath_An_Actl": path_angle,
      "LatCtlCrv_NoRate2_Actl": curvature_rate,
      "LatCtlCurv_No_Actl": curvature,
    }
    return self.packer.make_can_msg_safety("LateralMotionControl2", 0, values)

  def _engage(self, speed: float = 30.0, curvature: float = 0.0, shadow: float = 0.0):
    """Speed/measurement primed, controls allowed, LKA corroborating, path_angle history at zero."""
    self._set_speed_and_curvature(speed, curvature)
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._lat_ctl_msg(False, 0.0)))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._lka_msg(True, shadow)))

  @staticmethod
  def _path_angle_roc(speed: float) -> float:
    return float(np.interp(speed - 1.0, PATH_ANGLE_ROC_BP, PATH_ANGLE_ROC_V))


class TestFordAngleControlSafetyBase(FordBluePilotSafetyHarness):
  LATERAL_MODE = int(PrimaryLateralControl.angle)

  def test_angle_mode_is_active(self):
    """The SP param actually selected the angle branch, so the rest of these tests mean something.
    In the stock branch a nonzero path_angle is always blocked; here a small one is allowed."""
    self._engage()
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0.005)))

  def test_curvature_must_stay_inactive(self):
    """path_angle is the only actuator in angle mode; c2 must sit at its sentinel."""
    self._engage()
    for curvature in (-0.02, -0.001, 0.001, 0.02):
      self._engage()
      self.assertFalse(self._tx(self._lat_ctl_msg(True, 0.0, curvature=curvature)),
                       f"curvature {curvature} was allowed in angle mode")

  def test_curvature_rate_and_path_offset_must_stay_inactive(self):
    for path_offset in (-1.0, -0.02, 0.02, 1.0):
      self._engage()
      self.assertFalse(self._tx(self._lat_ctl_msg(True, 0.0, path_offset=path_offset)))
    for curvature_rate in (-0.001, -0.00005, 0.00005, 0.001):
      self._engage()
      self.assertFalse(self._tx(self._lat_ctl_msg(True, 0.0, curvature_rate=curvature_rate)))

  def test_path_angle_can_reach_the_full_dbc_range(self):
    """path_angle is the actuator in angle mode, so the whole signal range must be reachable one
    rate-limited step at a time. Nothing beyond it needs checking: the signal is 11 bits, so
    [-0.5, 0.5235] rad is exactly what the wire can express."""
    for speed in (12.0, 30.0):
      roc = self._path_angle_roc(speed)
      for limit in (FORD_DBC_PATH_ANGLE_MAX, FORD_DBC_PATH_ANGLE_MIN):
        sign = 1.0 if limit > 0 else -1.0
        self._engage(speed)
        angle = 0.0
        while abs(angle) < abs(limit) - roc:
          angle += sign * roc
          self.assertTrue(self._tx(self._lat_ctl_msg(True, angle)),
                          f"blocked at {angle} ramping to {limit} at {speed} m/s")
        self.assertTrue(self._tx(self._lat_ctl_msg(True, limit)))

  def test_path_angle_rate_limit(self):
    for speed in (12.0, 20.0, 30.0):
      roc = self._path_angle_roc(speed)
      for sign in (1.0, -1.0):
        self._engage(speed)
        # one step inside the limit is fine
        self.assertTrue(self._tx(self._lat_ctl_msg(True, sign * roc * 0.9)))
        # a step well past it is not
        self._engage(speed)
        self.assertFalse(self._tx(self._lat_ctl_msg(True, sign * roc * 3.0)),
                         f"step of {roc * 3.0} allowed at {speed} m/s")

  def test_path_angle_inactive_when_not_steering(self):
    self._engage()
    self.assertTrue(self._tx(self._lat_ctl_msg(False, 0.0)))
    self.assertFalse(self._tx(self._lat_ctl_msg(False, 0.01)))

  def test_no_steering_without_controls_allowed(self):
    self._engage()
    self.safety.set_controls_allowed(False)
    self.safety.set_controls_allowed_lateral(False)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0.0)))
    self.assertTrue(self._tx(self._lat_ctl_msg(False, 0.0)))

  def test_requires_lka_corroboration(self):
    """A LateralMotionControl frame cannot steer unless the LKA message independently says angle
    mode is engaged, so one crafted frame cannot unlock path_angle on its own."""
    self._engage()
    self.assertTrue(self._tx(self._lka_msg(False, 0.0)))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0.0)))
    self.assertTrue(self._tx(self._lka_msg(True, 0.0)))
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0.0)))

  def test_shadow_curvature_absolute_limit(self):
    # below CURVATURE_ERROR_MIN_SPEED the deviation band is off and the lateral accel limit is
    # far wider than the signal range, so this isolates the absolute cap
    speed = 5.0
    for shadow in (-0.03, -0.021, 0.021, 0.03):
      self._engage(speed, shadow=shadow)
      self.assertFalse(self._tx(self._lat_ctl_msg(True, 0.0)), f"shadow curvature {shadow} allowed")
    for shadow in (-0.019, 0.0, 0.019):
      self._engage(speed, shadow=shadow)
      self.assertTrue(self._tx(self._lat_ctl_msg(True, 0.0)), f"shadow curvature {shadow} blocked")

  def test_no_lateral_accel_ceiling(self):
    """BluePilot does not apply the ISO lateral acceleration ceiling in angle mode, so a command
    well past it is accepted as long as the car is actually tracking it."""
    # speeds where the envelope sits well inside the DBC's own 0.02 1/m range, so it really is
    # the ceiling being tested and not the signal cap
    for speed in (25.0, 35.0):
      fudged = max(speed - 1.0, 1.0)
      iso_envelope = MAX_LATERAL_ACCEL / (fudged * fudged)
      shadow = min(iso_envelope * 2.0, self.MAX_CURVATURE * 0.9)
      self.assertGreater(shadow, iso_envelope, f"test is not exercising the ceiling at {speed} m/s")
      self._engage(speed, curvature=shadow, shadow=shadow)
      self.assertTrue(self._tx(self._lat_ctl_msg(True, 0.0)), f"{speed} m/s, shadow {shadow}")

  def test_shadow_curvature_deviation_from_measured(self):
    """The car's steering intent is checked against measured curvature even though the curvature
    signal itself is pinned at zero -- otherwise angle mode has no deviation protection at all."""
    speed = 30.0
    for measured in (-0.01, 0.0, 0.01):
      for delta in (-0.006, -0.0025, 0.0, 0.0025, 0.006):
        shadow = measured + delta
        self._engage(speed, curvature=measured, shadow=shadow)
        # the measurement is sampled over 6 frames, hence the extra tolerance on the boundary
        if abs(abs(delta) - self.MAX_CURVATURE_ERROR) < 1e-6:
          continue
        self.assertEqual(abs(delta) <= self.MAX_CURVATURE_ERROR, self._tx(self._lat_ctl_msg(True, 0.0)),
                         f"measured {measured} shadow {shadow}")

  def test_deviation_band_unchecked_below_min_speed(self):
    speed = self.CURVATURE_ERROR_MIN_SPEED - 2.0
    self._engage(speed, curvature=0.0, shadow=0.015)
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0.0)))

  def test_real_time_rate_limit(self):
    """path_angle is rate limited per message, so the message rate itself has to be bounded --
    otherwise sending at 100 Hz instead of 20 Hz slews five times faster than intended. Same
    rolling two-bucket window the stock curvature path uses."""
    self._engage()
    max_rt_msgs = int(self.LATERAL_FREQUENCY * RT_INTERVAL / 1e6 * 1.2 + 1)
    half = RT_INTERVAL // 2

    self.safety.set_timer(0)
    for i in range(max_rt_msgs * 2):
      self.assertEqual(i <= max_rt_msgs, self._tx(self._lat_ctl_msg(True, 0.0, increment_timer=False)))

    # shift the overflow into the previous bucket
    self.safety.set_timer(half)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0.0, increment_timer=False)))

    # the previous bucket still counts within the half interval
    self.safety.set_timer(half + 2 * RT_INTERVAL // 5)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0.0, increment_timer=False)))

    # both buckets clear after a full interval
    self.safety.set_timer(half + RT_INTERVAL)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0.0, increment_timer=False)))
    self.safety.set_timer(half + 2 * RT_INTERVAL)
    for _ in range(max_rt_msgs):
      self.assertTrue(self._tx(self._lat_ctl_msg(True, 0.0, increment_timer=False)))

  def test_speed_mismatch_drops_controls(self):
    """Ford scales its limits by speed, so a disagreement between the ABS and PCM speed sources
    must drop controls in angle mode exactly as it does on the stock path."""
    for speed in np.arange(0, 40, 2.0):
      for speed_delta in (-3.0, -1.0, 0.0, 1.0, 3.0):
        speed_2 = round(max(speed + speed_delta, 0), 1)
        self._rx(self._speed_msg(speed))
        self._rx(self._speed_msg_2(speed_2))
        self.safety.set_controls_allowed(True)
        self._tx(self._lat_ctl_msg(True, 0.0))
        within_tolerance = abs(speed_2 - speed) <= 2.0
        self.assertEqual(within_tolerance, self.safety.get_controls_allowed(),
                         f"speed {speed} vs {speed_2}")

  def test_lka_action_still_blocked(self):
    """Packing angle-control state into the unused bits must not weaken the existing check that
    Lane_Assist_Data1 never requests a lane-keeping action."""
    self._engage()
    for action in range(1, 8):
      self.assertFalse(self._tx(self._lka_msg(True, 0.0, action=action)))
    self.assertTrue(self._tx(self._lka_msg(True, 0.0, action=0)))


class TestFordAngleControlSafety(TestFordAngleControlSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl
  SAFETY_PARAM = 0


class TestFordCANFDAngleControlSafety(TestFordAngleControlSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl2
  SAFETY_PARAM = FordSafetyFlags.CANFD


class TestFordCurvatureControlSafetyBase(FordBluePilotSafetyHarness):
  """Curvature mode keeps c2 as the actuator, so the stock curvature protections still apply and
  are covered by the whole-suite subclasses below. These cover what curvature mode adds: a c1
  trim under a far tighter cap than the signal allows, and c0 still pinned."""
  LATERAL_MODE = int(PrimaryLateralControl.curvature)

  # mirrors lateral_curv_ext.py _LC_PATH_ANGLE_ROC_*, x1.02
  CURV_ROC_BP = [5., 15., 25.]
  CURV_ROC_V = [0.00306, 0.00153, 0.00204]

  def _curv_roc(self, speed: float) -> float:
    return float(np.interp(speed - 1.0, self.CURV_ROC_BP, self.CURV_ROC_V))

  def test_path_angle_capped_far_below_the_dbc_range(self):
    """c1 only trims lane position here. Angle mode may use the whole signal range; this must not."""
    speed = 20.0
    roc = self._curv_roc(speed)
    for sign in (1.0, -1.0):
      self._engage(speed)
      angle = 0.0
      # ramp to the cap, which must be reachable
      while abs(angle) < CURV_MODE_PATH_ANGLE_MAX - roc:
        angle += sign * roc
        self.assertTrue(self._tx(self._lat_ctl_msg(True, angle)), f"blocked at {angle}")
      self.assertTrue(self._tx(self._lat_ctl_msg(True, sign * CURV_MODE_PATH_ANGLE_MAX)))
      # and one step past it must not be
      self.assertFalse(self._tx(self._lat_ctl_msg(True, sign * (CURV_MODE_PATH_ANGLE_MAX + roc))),
                       f"path_angle past {CURV_MODE_PATH_ANGLE_MAX} was allowed")

  def test_angle_mode_range_is_not_reachable(self):
    """The wide path_angle range belongs to angle mode alone; selecting curvature must not give
    access to it, whatever the LKA message claims."""
    self._engage(20.0)
    self.assertTrue(self._tx(self._lka_msg(True, 0.0)))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, FORD_DBC_PATH_ANGLE_MAX * 0.9)))

  def test_path_angle_rate_limit(self):
    for speed in (10.0, 20.0, 30.0):
      roc = self._curv_roc(speed)
      for sign in (1.0, -1.0):
        self._engage(speed)
        self.assertTrue(self._tx(self._lat_ctl_msg(True, sign * roc * 0.9)))
        self._engage(speed)
        self.assertFalse(self._tx(self._lat_ctl_msg(True, sign * roc * 4.0)),
                         f"step of {roc * 4.0} allowed at {speed} m/s")

  def test_path_angle_inactive_when_not_steering(self):
    self._engage()
    self.assertTrue(self._tx(self._lat_ctl_msg(False, 0.0)))
    self.assertFalse(self._tx(self._lat_ctl_msg(False, 0.01)))

  def test_path_offset_must_stay_inactive(self):
    """c0 and c1 fight each other on this platform, so openpilot never sends c0."""
    for path_offset in (-1.0, -0.02, 0.02, 1.0):
      self._engage()
      self.assertFalse(self._tx(self._lat_ctl_msg(True, 0.0, path_offset=path_offset)))

  def test_curvature_rate_is_allowed(self):
    """c3 is a real signal in curvature mode, unlike angle mode where it stays at its sentinel."""
    self._engage()
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0.0, curvature_rate=0.0005)))


class TestFordCurvatureControlSafety(TestFordCurvatureControlSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl
  SAFETY_PARAM = 0


class TestFordCANFDCurvatureControlSafety(TestFordCurvatureControlSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl2
  SAFETY_PARAM = FordSafetyFlags.CANFD


class CurvatureModeMixin:
  """Re-runs a whole stock test class with curvature mode selected. c2 is still the actuator
  there, so every protection the stock branch has must survive the switch."""

  def setUp(self):
    # before the parent's set_safety_hooks, which is what reads it, and again afterwards because
    # init_tests zeroes it and a later _reset_safety_hooks would otherwise drop back to stock
    libsafety_py.libsafety.set_current_safety_param_sp(int(PrimaryLateralControl.curvature))
    super().setUp()
    self.safety.set_current_safety_param_sp(int(PrimaryLateralControl.curvature))

  # BluePilot's rate table, which replaces the ISO envelope in this mode
  BP_ROC_BP = [5., 16., 25.]
  BP_ROC_V = [0.0025, 0.0014, 0.00018]

  def _bp_max_delta_can(self, speed: float) -> int:
    return int(np.interp(speed - 1.0, self.BP_ROC_BP, self.BP_ROC_V) * self.DEG_TO_CAN) + 1

  def test_max_lateral_acceleration(self):
    """No acceleration ceiling in this mode: the DBC's own 0.02 1/m range is the only absolute
    cap on c2, at every speed."""
    max_curvature_can = round(self.MAX_CURVATURE * self.DEG_TO_CAN)
    for speed in np.arange(0, 40, 2.0):
      for offset in (-5, -1, 0, 1, 5):
        curvature = (max_curvature_can + offset) / self.DEG_TO_CAN
        for sign in (-1, 1):
          signed = sign * curvature
          self.safety.set_controls_allowed(True)
          self._set_prev_desired_angle(signed)
          self._reset_curvature_measurement(signed, speed)
          should_tx = abs(max_curvature_can + offset) <= max_curvature_can
          self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(True, 0, 0, signed, 0)),
                           f"{speed} m/s, {signed}")

  def test_curvature_rate_limits(self):
    """c2 is rate limited by BluePilot's table rather than the ISO lateral jerk envelope."""
    small = 1 / self.DEG_TO_CAN
    band = (round(self.MAX_CURVATURE_ERROR * self.DEG_TO_CAN) + 1) / self.DEG_TO_CAN
    for speed in np.arange(2.0, 40, 2.0):
      # from a measurement of zero, whichever of the rate limit and the deviation band is tighter
      limit = self._bp_max_delta_can(speed) / self.DEG_TO_CAN
      if speed > self.CURVATURE_ERROR_MIN_SPEED:
        limit = min(limit, band)
      for sign in (-1, 1):
        # a couple of CAN units of slack either side, so float packing is not what is under test
        for delta, should_tx in ((limit - 2 * small, True), (limit + 2 * small, False)):
          self.safety.set_controls_allowed(True)
          self._reset_curvature_measurement(0, speed)
          self._set_prev_desired_angle(0)
          self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(True, 0, 0, sign * delta, 0)),
                           f"{speed} m/s, step {sign * delta}")

  def test_rate_limit_is_looser_than_the_iso_envelope(self):
    """The point of the change: a step the ISO lateral jerk envelope would reject is accepted
    where BluePilot's table is the looser of the two, which is the 11-28 m/s band."""
    for speed in (12.0, 16.0, 20.0, 25.0):
      fudged = max(speed - 1.0, 1.0)
      iso_delta = MAX_LATERAL_JERK / (fudged * fudged) / self.LATERAL_FREQUENCY
      bp_delta = self._bp_max_delta_can(speed) / self.DEG_TO_CAN
      self.assertGreater(bp_delta, iso_delta, f"test is not exercising the change at {speed} m/s")

      # a step the envelope would have rejected, still inside the deviation band
      band = (round(self.MAX_CURVATURE_ERROR * self.DEG_TO_CAN) + 1) / self.DEG_TO_CAN
      step = min((iso_delta + bp_delta) / 2, band - 2 / self.DEG_TO_CAN)
      self.assertGreater(step, iso_delta, f"test is not exercising the change at {speed} m/s")
      self.safety.set_controls_allowed(True)
      self._reset_curvature_measurement(0, speed)
      self._set_prev_desired_angle(0)
      self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, step, 0)), f"{speed} m/s, step {step}")

  def test_steer_allowed(self):
    """The stock version of this asserts the three trim signals must be zero, which is exactly
    what curvature mode relaxes. Everything it says about c2 still holds, so sweep that with the
    trim signals at their sentinels."""
    for speed in (self.CURVATURE_ERROR_MIN_SPEED - 1, self.CURVATURE_ERROR_MIN_SPEED + 1):
      max_curvature_can = self._get_max_curvature_can(speed)
      for controls_allowed in (True, False):
        for steer_control_enabled in (True, False):
          for curvature in (-self.MAX_CURVATURE, -0.001, 0, 0.001, self.MAX_CURVATURE):
            self.safety.set_controls_allowed(controls_allowed)
            self._set_prev_desired_angle(curvature)
            self._reset_curvature_measurement(curvature, speed)

            should_tx = controls_allowed if steer_control_enabled else curvature == 0
            should_tx = should_tx and abs(round(curvature * self.DEG_TO_CAN)) <= max_curvature_can

            with self.subTest(controls_allowed=controls_allowed, steer_control_enabled=steer_control_enabled,
                              curvature=float(curvature)):
              self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(steer_control_enabled, 0, 0, curvature, 0)))


class TestFordCurvatureModeStockSuite(CurvatureModeMixin, TestFordLongitudinalSafety):
  pass


class TestFordCANFDCurvatureModeStockSuite(CurvatureModeMixin, TestFordCANFDLongitudinalSafety):
  pass


if __name__ == "__main__":
  unittest.main()
