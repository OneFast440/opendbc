"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ford Intelligent Cruise Button Management.

Moves the stock cruise setpoint by pressing the steering wheel's own set/increase and
set/decrease buttons, so speed control features work on vehicles running Ford's ACC rather than
openpilot longitudinal.
"""
from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.ford import fordcan
from opendbc.sunnypilot.car.intelligent_cruise_button_management_interface_base import (
  IntelligentCruiseButtonManagementInterfaceBase,
)

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

# Signals in Steering_Data_FD1 (CAN id 131)
BUTTON_SIGNALS = {
  SendButtonState.increase: "CcAslButtnSetIncPress",
  SendButtonState.decrease: "CcAslButtnSetDecPress",
}

# Ford's own SCCM sends this message at 10 Hz; openpilot sends it twice as fast, so space the
# injected presses out to roughly what the stock module would produce.
_MIN_BUTTON_INTERVAL_S = 0.05


class IntelligentCruiseButtonManagementInterface(IntelligentCruiseButtonManagementInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)

  def update(self, CC_SP, CS, packer, CAN, frame, last_button_frame) -> tuple[list[CanData], int]:
    can_sends: list[CanData] = []
    self.CC_SP = CC_SP
    self.ICBM = CC_SP.intelligentCruiseButtonManagement
    self.frame = frame
    self.last_button_frame = last_button_frame

    if self.ICBM.sendButton != SendButtonState.none:
      button_signal = BUTTON_SIGNALS[self.ICBM.sendButton]
      if (self.frame - self.last_button_frame) * DT_CTRL > _MIN_BUTTON_INTERVAL_S:
        # Both buses, same as cancel and resume
        can_sends.append(fordcan.create_button_msg(packer, CAN.camera, CS.buttons_stock_values,
                                                   icbm_button=button_signal))
        can_sends.append(fordcan.create_button_msg(packer, CAN.main, CS.buttons_stock_values,
                                                   icbm_button=button_signal))
        self.last_button_frame = self.frame

    return can_sends, self.last_button_frame
