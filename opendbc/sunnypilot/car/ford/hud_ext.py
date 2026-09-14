"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ford instrument cluster messaging, ported from BluePilot bp-7.0.

Three things upstream does not do:

  * the two lane lines are drawn independently, so the cluster shows what the model actually
    sees on each side rather than a single combined state;
  * the hands-off warning escalates through the cluster's own levels, driven by openpilot's
    driver monitoring rather than by a single "steer required" bool, so the driver gets the
    silent prompt before the chime;
  * the time-gap control stays on screen for four seconds after it changes, instead of one
    message, which is otherwise too brief to read.

On CAN FD vehicles with BlueCruise the cluster also has a hands-free presentation, which this
can drive.
"""
from opendbc.car import structs
from opendbc.car.ford.values import CarControllerParams, FordFlags
from opendbc.sunnypilot.car.ford import fordcan_ext

VisualAlert = structs.CarControl.HUDControl.VisualAlert

# Cluster hands-off levels
HANDS_ON = 0
HANDS_WARN_SILENT = 1
HANDS_WARN_CHIME = 2

# TjaMsgTxt_D_Dsply
TJA_MSG_NONE = 0
TJA_MSG_UNAVAILABLE = 1

# TjaWarn_D_Rq
TJA_WARN_NONE = 0
TJA_WARN_CANCELLED = 1
TJA_WARN_RESUME_CONTROL = 3
TJA_WARN_RIGHT_LANE_DEPARTURE = 4
TJA_WARN_LEFT_LANE_DEPARTURE = 5

# The time-gap control is unreadable if it flashes past in one 5 Hz message.
_DISTANCE_BAR_FRAMES = 400  # 4 s at 100 Hz


def split_alert_type(alert_type: str, main_on: bool) -> tuple[str, str]:
  """selfdriveState.alertType is "<name>/<state>"; with cruise main off nothing is shown."""
  if not main_on:
    return "none", "none"
  parts = alert_type.split("/")
  return parts[0], parts[-1]


def driver_monitoring_msg(alert_type: str, hud_control, hands_free_cluster: bool,
                          main_on: bool, standstill: bool) -> tuple[int, int, int]:
  """Map openpilot's driver monitoring state onto the cluster's TJA signals.

  Returns (tja_msg, tja_warn, hands). The escalation is deliberately ordered: a distracted or
  unresponsive driver gets the cluster's own prompt before openpilot's soft disable turns into a
  "resume control" warning, and the chime is suppressed at a standstill where it is only noise.
  """
  driver_state, disable_state = split_alert_type(alert_type, main_on)
  tja_msg, tja_warn, hands = TJA_MSG_NONE, TJA_WARN_NONE, HANDS_ON

  if disable_state == "noEntry":
    tja_msg = TJA_MSG_UNAVAILABLE
  elif driver_state in ("driverDistracted", "driverUnresponsive") or \
       disable_state in ("softDisable", "immediateDisable"):
    tja_warn = TJA_WARN_RESUME_CONTROL
  elif disable_state == "userDisable":
    tja_warn = TJA_WARN_CANCELLED
  elif driver_state in ("preDriverDistracted", "preDriverUnresponsive"):
    hands = HANDS_WARN_SILENT
  elif driver_state in ("promptDriverDistracted", "promptDriverUnresponsive"):
    hands = HANDS_WARN_SILENT if standstill else HANDS_WARN_CHIME
  elif hands_free_cluster and hud_control.leftLaneDepart:
    tja_warn = TJA_WARN_LEFT_LANE_DEPARTURE
  elif hands_free_cluster and hud_control.rightLaneDepart:
    tja_warn = TJA_WARN_RIGHT_LANE_DEPARTURE

  return tja_msg, tja_warn, hands


class HudExt:
  """Mixed into the Ford CarController. Owns the cluster messaging state."""

  def __init__(self, CP, CP_SP):
    hud = CP_SP.fordHud
    # The cluster's hands-free presentation only exists on CAN FD vehicles.
    self.hands_free_cluster = bool(hud.handsFreeClusterMsg) and bool(CP.flags & FordFlags.CANFD)
    self.driver_monitor_msg = bool(hud.driverMonitorCanMsg)

    self.tja_msg = TJA_MSG_NONE
    self.tja_warn = TJA_WARN_NONE
    self.hands = HANDS_ON

    self.main_on_last = False
    self.lat_active_last = False
    self.steer_alert_last = False
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0

  def update(self, CC, CC_SP, CS, hud_control, main_on, fcw_alert, frame, packer, CAN, CP) -> list:
    """Build the cluster messages for this frame."""
    can_sends = []
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    standstill = CS.out.cruiseState.standstill

    if self.driver_monitor_msg:
      if (frame % CarControllerParams.ACC_UI_STEP) == 0:
        self.tja_msg, self.tja_warn, self.hands = driver_monitoring_msg(
          CC_SP.fordLateral.alertType, hud_control, self.hands_free_cluster, main_on, standstill)
    else:
      self.tja_msg, self.tja_warn = TJA_MSG_NONE, TJA_WARN_NONE
      self.hands = HANDS_WARN_SILENT if steer_alert else HANDS_ON

    send_ui = ((self.main_on_last != main_on) or
               (self.lat_active_last != CC.latActive) or
               (self.steer_alert_last != steer_alert))

    if (frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan_ext.create_lkas_ui_msg(
        packer, CAN, self.hands, hud_control, CS.lkas_status_stock_values))

    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True
      self.distance_bar_frame = frame
    show_distance_bars = (frame - self.distance_bar_frame) < _DISTANCE_BAR_FRAMES
    send_ui |= show_distance_bars

    if (frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan_ext.create_acc_ui_msg(
        packer, CAN, CP, main_on, CC.latActive, fcw_alert, standstill, hud_control,
        CS.acc_tja_status_stock_values, self.hands_free_cluster, show_distance_bars,
        self.tja_warn, self.tja_msg))

    self.main_on_last = main_on
    self.lat_active_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    return can_sends
