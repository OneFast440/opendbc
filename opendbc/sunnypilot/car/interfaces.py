"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import os
import numpy as np
from typing import NamedTuple
from collections.abc import Callable

from opendbc.car import structs
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
from opendbc.car.hyundai.values import HyundaiFlags
from opendbc.car.subaru.values import SubaruFlags
from opendbc.car.toyota.values import ToyotaSafetyFlags
from opendbc.sunnypilot.car.hyundai.enable_radar_tracks import enable_radar_tracks as hyundai_enable_radar_tracks
from opendbc.sunnypilot.car.hyundai.longitudinal.helpers import LongitudinalTuningType
from opendbc.sunnypilot.car.ford.values_ext import (
  BLEND_RATIO_RANGE,
  FordSafetyFlagsSP,
  HIGH_SPEED_DAMPENING_RANGE,
  HIGH_SPEED_FACTOR_RANGE,
  LANE_CHANGE_FACTOR_CURV_RANGE,
  LANE_CHANGE_FACTOR_RANGE,
  LANE_POSITIONING_GAIN_RANGE,
  LOW_SPEED_FACTOR_RANGE,
  PATH_OFFSET_RANGE,
  PrimaryLateralControl,
)
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP, SubaruSafetyFlagsSP
from opendbc.sunnypilot.car.tesla.values import MadsScreenButtonType, TeslaFlagsSP, TeslaSafetyFlagsSP
from opendbc.sunnypilot.car.toyota.values import ToyotaFlagsSP


class LatControlInputs(NamedTuple):
  lateral_acceleration: float
  roll_compensation: float
  vego: float
  aego: float


TorqueFromLateralAccelCallbackTypeTorqueSpace = Callable[[LatControlInputs, structs.CarParams.LateralTorqueTuning, bool], float]


class CarInterfaceBaseSP:
  @staticmethod
  def torque_from_lateral_accel_linear_in_torque_space(latcontrol_inputs: LatControlInputs, torque_params: structs.CarParams.LateralTorqueTuning,
                                                        gravity_adjusted: bool) -> float:
    # The default is a linear relationship between torque and lateral acceleration (accounting for road roll and steering friction)
    return latcontrol_inputs.lateral_acceleration / float(torque_params.latAccelFactor)

  def torque_from_lateral_accel_in_torque_space(self) -> TorqueFromLateralAccelCallbackTypeTorqueSpace:
    return self.torque_from_lateral_accel_linear_in_torque_space


class NanoFFModel:
  def __init__(self, weights_loc: str, platform: str):
    self.weights_loc = weights_loc
    self.platform = platform
    self.load_weights(platform)

  def load_weights(self, platform: str):
    with open(self.weights_loc) as fob:
      self.weights = {k: np.array(v) for k, v in json.load(fob)[platform].items()}

  def relu(self, x: np.ndarray):
    return np.maximum(0.0, x)

  def forward(self, x: np.ndarray):
    assert x.ndim == 1
    x = (x - self.weights['input_norm_mat'][:, 0]) / (self.weights['input_norm_mat'][:, 1] - self.weights['input_norm_mat'][:, 0])
    x = self.relu(np.dot(x, self.weights['w_1']) + self.weights['b_1'])
    x = self.relu(np.dot(x, self.weights['w_2']) + self.weights['b_2'])
    x = self.relu(np.dot(x, self.weights['w_3']) + self.weights['b_3'])
    x = np.dot(x, self.weights['w_4']) + self.weights['b_4']
    return x

  def predict(self, x: list[float], do_sample: bool = False):
    x = self.forward(np.array(x))
    if do_sample:
      pred = np.random.laplace(x[0], np.exp(x[1]) / self.weights['temperature'])
    else:
      pred = x[0]
    pred = pred * (self.weights['output_norm_mat'][1] - self.weights['output_norm_mat'][0]) + self.weights['output_norm_mat'][0]
    return pred


def setup_interfaces(CI, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                     params_list: list[dict[str, str]] | None = None,
                     can_recv: CanRecvCallable | None = None, can_send: CanSendCallable | None = None) -> None:
  if params_list is None:
    params_list = []

  params_dict = {k: v for param in params_list for k, v in param.items()}

  _initialize_custom_longitudinal_tuning(CI, CP, CP_SP, params_dict)
  _initialize_coop_steering(CP, CP_SP, params_dict)
  _initialize_tesla_mads_screen_button(CP, CP_SP, params_dict)
  _initialize_radar_tracks(CP, CP_SP, can_recv, can_send)
  _initialize_stop_and_go(CP, CP_SP, params_dict)
  _initialize_toyota(CP, CP_SP, params_dict)
  _initialize_ford(CP, CP_SP, params_dict)


def _initialize_custom_longitudinal_tuning(CI, CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                                           params_dict: dict[str, str]) -> None:

  # Hyundai Custom Longitudinal Tuning
  if CP.brand == 'hyundai':
    hyundai_longitudinal_tuning = int(params_dict.get("HyundaiLongitudinalTuning", 0))
    if hyundai_longitudinal_tuning == LongitudinalTuningType.DYNAMIC:
      CP_SP.flags |= HyundaiFlagsSP.LONG_TUNING_DYNAMIC.value
    if hyundai_longitudinal_tuning == LongitudinalTuningType.PREDICTIVE:
      CP_SP.flags |= HyundaiFlagsSP.LONG_TUNING_PREDICTIVE.value

  _ = CI.get_longitudinal_tuning_sp(CP, CP_SP)


def _initialize_coop_steering(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                              params_dict: dict[str, str]) -> None:
  if CP.brand == 'tesla':
    coop_steering = int(params_dict.get("TeslaCoopSteering", 0)) == 1
    if coop_steering:
      CP_SP.flags |= TeslaFlagsSP.COOP_STEERING.value


def _initialize_tesla_mads_screen_button(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                                         params_dict: dict[str, str]) -> None:
  if CP.brand == 'tesla' and CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS:
    selection = int(params_dict.get("TeslaMadsScreenButton", MadsScreenButtonType.OFF))
    if selection == MadsScreenButtonType.THREE_FINGER:
      CP_SP.flags |= TeslaFlagsSP.MADS_SCREEN_BUTTON_3_FINGER.value
      CP_SP.safetyParam |= TeslaSafetyFlagsSP.MADS_SCREEN_BUTTON_3_FINGER
    elif selection == MadsScreenButtonType.FOUR_FINGER:
      CP_SP.flags |= TeslaFlagsSP.MADS_SCREEN_BUTTON_4_FINGER.value
      CP_SP.safetyParam |= TeslaSafetyFlagsSP.MADS_SCREEN_BUTTON_4_FINGER
    elif selection == MadsScreenButtonType.FIVE_FINGER:
      CP_SP.flags |= TeslaFlagsSP.MADS_SCREEN_BUTTON_5_FINGER.value
      CP_SP.safetyParam |= TeslaSafetyFlagsSP.MADS_SCREEN_BUTTON_5_FINGER


def _initialize_radar_tracks(CP: structs.CarParams, CP_SP: structs.CarParamsSP,
                             can_recv: CanRecvCallable | None = None, can_send: CanSendCallable | None = None) -> None:
  if can_recv is None or can_send is None or os.environ.get("REPLAY"):
    return

  if CP.brand == 'hyundai':
    if CP.flags & HyundaiFlags.MANDO_RADAR and (CP.radarUnavailable or CP_SP.flags & HyundaiFlagsSP.ENHANCED_SCC):
      tracks_enabled = hyundai_enable_radar_tracks(can_recv, can_send, bus=0, addr=0x7d0)
      CP.radarUnavailable = not tracks_enabled


def _initialize_stop_and_go(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params_dict: dict[str, str]) -> None:
  if CP.brand == 'subaru' and not CP.flags & (SubaruFlags.GLOBAL_GEN2 | SubaruFlags.HYBRID):
    stop_and_go = int(params_dict.get("SubaruStopAndGo", 0)) == 1
    stop_and_go_manual_parking_brake = int(params_dict.get("SubaruStopAndGoManualParkingBrake", 0)) == 1

    if stop_and_go:
      CP_SP.flags |= SubaruFlagsSP.STOP_AND_GO.value
    if stop_and_go_manual_parking_brake:
      CP_SP.flags |= SubaruFlagsSP.STOP_AND_GO_MANUAL_PARKING_BRAKE.value
    if stop_and_go or stop_and_go_manual_parking_brake:
      CP_SP.safetyParam |= SubaruSafetyFlagsSP.STOP_AND_GO


def _clamp_tuning(raw, spec: tuple[float, float, float]) -> float:
  default, lo, hi = spec
  try:
    value = float(raw)
  except (TypeError, ValueError):
    return default
  if value == 0.0:
    return default
  return float(np.clip(value, lo, hi))


def _bool_param(params_dict: dict[str, str], key: str, default: bool = False) -> bool:
  raw = params_dict.get(key)
  if raw is None or raw == "":
    return default
  try:
    return int(raw) == 1
  except (TypeError, ValueError):
    return default


def _initialize_ford(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params_dict: dict[str, str]) -> None:
  """Ford lateral, longitudinal and cluster settings (BluePilot).

  Read once here rather than live in the car controller: the panda's lateral mode and
  longitudinal allowlist come from the same read, and a live flip against stale firmware would
  have openpilot fighting the panda. Changing any of these needs an onroad cycle.
  """
  if CP.brand != 'ford':
    return

  try:
    mode = PrimaryLateralControl(int(params_dict.get("FordPrefLateralControl", 0) or 0))
  except ValueError:
    mode = PrimaryLateralControl.stock

  lateral = CP_SP.fordLateralTuning
  lateral.primaryControl = int(mode)
  # bits 0-1 of the SP safety param; the panda falls back to stock on anything it does not know
  CP_SP.safetyParam |= int(mode) & FordSafetyFlagsSP.LATERAL_MODE_MASK

  if mode == PrimaryLateralControl.angle:
    lateral.lowSpeedFactor = _clamp_tuning(params_dict.get("FordLowSpeedFactor_ang"), LOW_SPEED_FACTOR_RANGE)
    lateral.highSpeedFactor = _clamp_tuning(params_dict.get("FordHighSpeedFactor_ang"), HIGH_SPEED_FACTOR_RANGE)
    lateral.highSpeedDampening = _clamp_tuning(params_dict.get("FordHighSpeedDampening_ang"), HIGH_SPEED_DAMPENING_RANGE)
    lateral.laneChangeFactor = _clamp_tuning(params_dict.get("FordLaneChangeFactor_ang"), LANE_CHANGE_FACTOR_RANGE)
  elif mode == PrimaryLateralControl.curvature:
    lateral.humanTurnDetection = _bool_param(params_dict, "FordHumanTurnDetection_curv", True)
    lateral.laneChangeFactorCurv = _clamp_tuning(params_dict.get("FordLaneChangeFactor_curv"), LANE_CHANGE_FACTOR_CURV_RANGE)
    lateral.blendRatioLow = _clamp_tuning(params_dict.get("FordBlendRatioLow_curv"), BLEND_RATIO_RANGE)
    lateral.blendRatioHigh = _clamp_tuning(params_dict.get("FordBlendRatioHigh_curv"), BLEND_RATIO_RANGE)
    lateral.lanePositioning = _bool_param(params_dict, "FordLanePositioning_curv")
    lateral.laneFullMode = _bool_param(params_dict, "FordLaneFullMode_curv")
    # zero is a legitimate path offset, so it is clamped rather than defaulted
    lateral.pathOffset = float(np.clip(float(params_dict.get("FordPathOffset_curv") or 0.0),
                                       PATH_OFFSET_RANGE[1], PATH_OFFSET_RANGE[2]))
    lateral.customProfile = int(params_dict.get("FordCustomProfile_curv", 0) or 0)
    lateral.lanePositioningGain = _clamp_tuning(params_dict.get("FordLanePositioningGain_curv"),
                                                LANE_POSITIONING_GAIN_RANGE)

  CP_SP.fordLongitudinalTuning.followControl = _bool_param(params_dict, "FordFollowControl", True)
  CP_SP.fordLongitudinalTuning.downhillCompensation = _bool_param(params_dict, "FordDownhillCompensation", True)

  CP_SP.fordHud.handsFreeClusterMsg = _bool_param(params_dict, "FordHandsFreeClusterMsg")
  CP_SP.fordHud.driverMonitorCanMsg = _bool_param(params_dict, "FordDriverMonitorCanMsg")


def _initialize_toyota(CP: structs.CarParams, CP_SP: structs.CarParamsSP, params_dict: dict[str, str]) -> None:
  if CP.brand == 'toyota':
    toyota_stock_long = int(params_dict.get("ToyotaEnforceStockLongitudinal", 0)) == 1
    toyota_stop_and_go_hack = int(params_dict.get("ToyotaStopAndGoHack", 0)) == 1

    if toyota_stock_long:
      CP_SP.flags |= ToyotaFlagsSP.STOCK_LONGITUDINAL.value
      CP.alphaLongitudinalAvailable = False
      CP.openpilotLongitudinalControl = False
      CP.safetyConfigs[0].safetyParam |= ToyotaSafetyFlags.STOCK_LONGITUDINAL.value

    if toyota_stop_and_go_hack and CP.openpilotLongitudinalControl:
      CP_SP.flags |= ToyotaFlagsSP.STOP_AND_GO_HACK.value
