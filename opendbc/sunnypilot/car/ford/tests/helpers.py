"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Shared fakes for the Ford control tests.
"""
from collections import defaultdict
from types import SimpleNamespace

from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.ford.values import CAR
from opendbc.sunnypilot.car.ford.values_ext import T_IDXS, PrimaryLateralControl

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState
LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

MSG_Steering_Data_FD1 = 0x083
MSG_ACCDATA = 0x186
MSG_ACCDATA_3 = 0x18A
MSG_Lane_Assist_Data1 = 0x3CA
MSG_LateralMotionControl = 0x3D3
MSG_LateralMotionControl2 = 0x3D6
MSG_IPMA_Data = 0x3D8

# Inactive sentinels in raw CAN units
INACTIVE_CURVATURE = 1000
INACTIVE_PATH_ANGLE = 1000
INACTIVE_PATH_OFFSET = 512
INACTIVE_CURVATURE_RATE = 4096
INACTIVE_CURVATURE_RATE_CANFD = 1024


def make_cc(lat_active=True, long_active=False, curvature=0.0, accel=0.0,
            lead_distance_bars=0, visual_alert=VisualAlert.none):
  actuators = SimpleNamespace(
    curvature=curvature, accel=accel, gas=0.0, longControlState=LongCtrlState.off,
    as_builder=lambda: SimpleNamespace(curvature=0.0, accel=0.0, gas=0.0),
  )
  hud = SimpleNamespace(visualAlert=visual_alert, leadDistanceBars=lead_distance_bars,
                        leftLaneDepart=False, rightLaneDepart=False,
                        leftLaneVisible=True, rightLaneVisible=True, lanesVisible=True,
                        setSpeed=0.0, speedVisible=False, leadVisible=False)
  return SimpleNamespace(
    latActive=lat_active, longActive=long_active, enabled=lat_active or long_active,
    actuators=actuators, hudControl=hud,
    cruiseControl=SimpleNamespace(cancel=False, resume=False, override=False),
    orientationNED=[],
  )


def make_lead(status=False, d_rel=40.0, v_rel=0.0, v_lead=30.0):
  return SimpleNamespace(status=status, dRel=d_rel, vRel=v_rel, vLead=v_lead)


def make_cc_sp(model_curvature=0.0, lateral_delay=0.12, lane_change_state=0, lane_change_direction=0,
               model_curvatures=None, model_position_y=None, lane_line_left_y=-1.85,
               lane_line_right_y=1.85, lane_line_left_prob=0.9, lane_line_right_prob=0.9,
               alert_type="", lead=None, send_button=SendButtonState.none):
  curvatures = model_curvatures if model_curvatures is not None else [model_curvature] * len(T_IDXS)
  position_y = model_position_y if model_position_y is not None else [0.0] * len(T_IDXS)
  return SimpleNamespace(
    fordLateral=SimpleNamespace(
      modelCurvatures=curvatures,
      modelPositionY=position_y,
      lateralDelay=lateral_delay,
      laneChangeState=lane_change_state,
      laneChangeDirection=lane_change_direction,
      laneLineLeftY=lane_line_left_y,
      laneLineRightY=lane_line_right_y,
      laneLineLeftProb=lane_line_left_prob,
      laneLineRightProb=lane_line_right_prob,
      alertType=alert_type,
    ),
    leadOne=lead if lead is not None else make_lead(),
    leadTwo=make_lead(),
    intelligentCruiseButtonManagement=SimpleNamespace(sendButton=send_button, vTarget=0.0),
  )


def make_cs(v_ego=30.0, yaw_rate=0.0, steering_pressed=False, steering_angle=0.0,
            gas_pressed=False, brake_pressed=False, standstill=False, main_on=True):
  out = SimpleNamespace(
    vEgoRaw=v_ego, vEgo=v_ego, yawRate=yaw_rate,
    steeringPressed=steering_pressed, steeringAngleDeg=steering_angle,
    gasPressed=gas_pressed, brakePressed=brake_pressed,
    cruiseState=SimpleNamespace(available=main_on, standstill=standstill),
  )
  return SimpleNamespace(
    out=out,
    buttons_stock_values=defaultdict(int),
    acc_tja_status_stock_values=defaultdict(int),
    lkas_status_stock_values=defaultdict(int),
  )


def make_actuators(curvature=0.0):
  return SimpleNamespace(curvature=curvature)


def make_car_params(platform=CAR.FORD_F_150_MK14, mode=PrimaryLateralControl.stock,
                    alpha_long=False, **tuning):
  CI_cls = interfaces[platform]
  fingerprint = dict.fromkeys(range(7), {})
  CP = CI_cls.get_params(platform, fingerprint, [], alpha_long=alpha_long, is_release=False, docs=False)
  CP_SP = CI_cls.get_params_sp(CP, platform, fingerprint, [], alpha_long=alpha_long,
                               is_release_sp=False, docs=False)

  lateral = CP_SP.fordLateralTuning
  lateral.primaryControl = int(mode)
  if mode == PrimaryLateralControl.angle:
    lateral.lowSpeedFactor = tuning.get('low_speed_factor', 1.0)
    lateral.highSpeedFactor = tuning.get('high_speed_factor', 1.0)
    lateral.highSpeedDampening = tuning.get('high_speed_dampening', 1.0)
    lateral.laneChangeFactor = tuning.get('lane_change_factor', 1.0)
  elif mode == PrimaryLateralControl.curvature:
    lateral.humanTurnDetection = tuning.get('human_turn_detection', True)
    lateral.laneChangeFactorCurv = tuning.get('lane_change_factor_curv', 0.85)
    lateral.blendRatioLow = tuning.get('blend_ratio_low', 0.4)
    lateral.blendRatioHigh = tuning.get('blend_ratio_high', 0.4)
    lateral.lanePositioning = tuning.get('lane_positioning', False)
    lateral.laneFullMode = tuning.get('lane_full_mode', False)
    lateral.pathOffset = tuning.get('path_offset', 0.0)
    lateral.customProfile = tuning.get('custom_profile', 0)
    lateral.lanePositioningGain = tuning.get('lane_positioning_gain', 3.0)

  CP_SP.fordLongitudinalTuning.followControl = tuning.get('follow_control', True)
  CP_SP.fordLongitudinalTuning.downhillCompensation = tuning.get('downhill_compensation', True)
  CP_SP.fordHud.handsFreeClusterMsg = tuning.get('hands_free_cluster', False)
  CP_SP.fordHud.driverMonitorCanMsg = tuning.get('driver_monitor_cluster', False)
  return CP, CP_SP


def make_car_controller(platform=CAR.FORD_F_150_MK14, **kwargs):
  CP, CP_SP = make_car_params(platform, **kwargs)
  return interfaces[platform](CP, CP_SP).CC, CP


def unpack_lat_ctl(addr, dat):
  """The four polynomial signals in raw CAN units, exactly as safety/modes/ford.h reads them."""
  if addr == MSG_LateralMotionControl:
    return {
      'enabled': ((dat[4] >> 2) & 0x7) != 0,
      'curvature': (dat[0] << 3) | (dat[1] >> 5),
      'curvature_rate': ((dat[1] & 0x1F) << 8) | dat[2],
      'path_angle': (dat[3] << 3) | (dat[4] >> 5),
      'path_offset': (dat[5] << 2) | (dat[6] >> 6),
      'inactive_curvature_rate': INACTIVE_CURVATURE_RATE,
    }
  return {
    'enabled': ((dat[0] >> 4) & 0x7) != 0,
    'curvature': (dat[2] << 3) | (dat[3] >> 5),
    'curvature_rate': (dat[6] << 3) | (dat[7] >> 5),
    'path_angle': ((dat[3] & 0x1F) << 6) | (dat[4] >> 2),
    'path_offset': ((dat[4] & 0x3) << 8) | dat[5],
    'inactive_curvature_rate': INACTIVE_CURVATURE_RATE_CANFD,
  }
