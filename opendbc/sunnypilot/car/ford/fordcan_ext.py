"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car.ford import fordcan
from opendbc.car.ford.fordcan import CanBus

# shadow_curvature wire scale, 1/m per LSB. int16 -> +-0.0327 1/m, comfortably past the +-0.02
# DBC curvature range plus the deviation band.
SHADOW_CURVATURE_SCALE = 1e-6


def create_lka_msg(packer, CAN: CanBus, angle_mode_engaged: bool = False, shadow_curvature: float = 0.0):
  """
  Creates the Ford LKA Command, optionally carrying angle-control state for the panda.

  Upstream sends this message empty. Angle control needs two values to reach safety/modes/ford.h
  that have nowhere else to go:

    * angle_mode_engaged -- corroborates that the wide path_angle range is legitimate, so a frame
      cannot unlock it by merely zeroing curvature.
    * shadow_curvature -- the kappa path_angle was derived from. Angle mode holds the real
      curvature signal at its inactive sentinel, so without this there is no commanded-vs-measured
      deviation check for angle mode at all.

  They ride in bits of Lane_Assist_Data1 that no DBC signal maps to (byte 4 bits 4-0, bytes 5-7;
  see ford_lincoln_base_pt.dbc). ford_tx_hook already reads LkaActvStats_D2_Req straight out of
  these same bytes, in the same call, so the values are read synchronously off the message being
  transmitted -- no separate CAN ID and no RX round trip, which matters because panda does not
  self-receive its own TX.

  Byte layout, and it must match the decode in ford.h exactly:
    byte 4 bit 0:    angle_mode_engaged
    byte 4 bits 1-4: reserved
    bytes 5-6:       shadow_curvature, int16 big-endian, scale SHADOW_CURVATURE_SCALE
    byte 7:          reserved

  shadow_curvature is in the CAN sign convention, i.e. negated from openpilot's, the same way
  path_angle and curvature are negated when they go on the wire. ford.h's measured curvature comes
  from the raw yaw rate with no negation, so an un-negated shadow reads as a permanent divergence.

  Frequency is 33Hz.
  """
  addr, dat, bus = packer.make_can_msg("Lane_Assist_Data1", CAN.main, {})
  dat = bytearray(dat)

  raw = int(round(shadow_curvature / SHADOW_CURVATURE_SCALE))
  raw = max(-32768, min(32767, raw)) & 0xFFFF

  dat[4] |= 1 if angle_mode_engaged else 0
  dat[5] = (raw >> 8) & 0xFF
  dat[6] = raw & 0xFF

  return addr, bytes(dat), bus


def create_acc_ui_msg(packer, CAN: CanBus, CP, main_on: bool, enabled: bool, fcw_alert: bool,
                      standstill: bool, hud_control, stock_values: dict, hands_free_cluster: bool,
                      show_distance_bars: bool, tja_warn: int, tja_msg: int):
  """
  Ford IPC adaptive cruise, forward collision warning and TJA status.

  Differs from the upstream builder in three ways:
    * a BlueCruise cluster status (7), on vehicles where that UI exists
    * the TJA warning and message text come from driver monitoring rather than being forwarded
      from the stock camera
    * the FCW audible alert is raised alongside the visible one

  Frequency is 5Hz.
  """
  if enabled:
    if hud_control.leftLaneDepart:
      status = 3    # ActiveInterventionLeft
    elif hud_control.rightLaneDepart:
      status = 4    # ActiveInterventionRight
    elif hands_free_cluster:
      status = 7    # BlueCruise hands-free UI
    else:
      status = 2    # Active
  elif main_on:
    if hud_control.leftLaneDepart:
      status = 5    # ActiveWarningLeft
    elif hud_control.rightLaneDepart:
      status = 6    # ActiveWarningRight
    else:
      status = 1    # Standby
  elif standstill:
    status = 0      # Off
  else:
    status = 1      # Standby

  values = {s: stock_values[s] for s in fordcan.ACC_UI_PASSTHROUGH}
  values.update({
    "Tja_D_Stat": status,
    "TjaWarn_D_Rq": tja_warn,
    "TjaMsgTxt_D_Dsply": tja_msg,
  })

  if CP.openpilotLongitudinalControl:
    values.update({
      "AccStopStat_D_Dsply": 2 if standstill else 0,              # Stopping status text
      "AccMsgTxt_D2_Rq": 0,                                       # ACC text
      "AccTGap_B_Dsply": 1 if show_distance_bars else 0,          # Show time gap control UI
      "AccFllwMde_B_Dsply": 1 if hud_control.leadVisible else 0,  # Lead indicator
      "AccStopMde_B_Dsply": 1 if standstill else 0,
      "AccWarn_D_Dsply": 0,                                       # ACC warning
      "AccTGap_D_Dsply": hud_control.leadDistanceBars,            # Time gap
    })

  if fcw_alert:
    values["FcwVisblWarn_B_Rq"] = 1
    values["FcwAudioWarn_B_Rq"] = 1

  return packer.make_can_msg("ACCDATA_3", CAN.main, values)


def create_lkas_ui_msg(packer, CAN: CanBus, hands: int, hud_control, stock_values: dict):
  """
  Ford IPC IPMA/LKAS status.

  Differs from the upstream builder in two ways: the two lane lines are shown independently
  rather than driven off a single enabled/departing state, and the hands-off level is an explicit
  0-3 rather than a bool, so driver monitoring can escalate from a silent warning to a chime.

    hands: 0 HandsOn, 1 Level1 (no chime), 2 Level2 (chime), 3 Suppressed

  LaActvStats_D_Dsply is a single value encoding both sides (left major, right minor):

    right ->    Intervene  Warning  Suppress  Available  None
    Intervene      24        19        14         9        4
    Warning        23        18        13         8        3
    Suppress       22        17        12         7        2
    Available      21        16        11         6        1
    None           20        15        10         5        0

  Frequency is 1Hz.
  """
  if hud_control.leftLaneDepart:
    left = 4    # Intervene
  elif hud_control.leftLaneVisible:
    left = 1    # Available
  else:
    left = 2    # Suppress

  if hud_control.rightLaneDepart:
    right = 20  # Intervene
  elif hud_control.rightLaneVisible:
    right = 5   # Available
  else:
    right = 10  # Suppress

  values = {s: stock_values[s] for s in fordcan.LKAS_UI_PASSTHROUGH}
  values.update({
    "LaActvStats_D_Dsply": left + right,
    "LaHandsOff_D_Dsply": hands,
  })
  return packer.make_can_msg("IPMA_Data", CAN.main, values)
