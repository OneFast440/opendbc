"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ford cruise control button events, ported from BluePilot bp-7.0.

Upstream Ford only reports the gap and lane-centering buttons. Intelligent Cruise Button
Management and Speed Limit Assist both need to see the driver's own set and resume presses, so
they can tell a speed the driver chose from one they injected.

Three of Ford's buttons are combos whose meaning depends on whether cruise is already engaged,
so the event a press produces is chosen at the moment it goes down, and the matching release
reports that same event rather than re-deciding against a cruise state that may have changed in
between.
"""
from enum import StrEnum

from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.sunnypilot.car.ford.values_ext import BUTTONS

ButtonType = structs.CarState.ButtonEvent.Type

# signal -> (event while cruise is engaged, event while it is not)
COMBO_BUTTONS = {
  "CcAslButtnSetIncPress": (ButtonType.accelCruise, ButtonType.setCruise),
  "CcAslButtnSetDecPress": (ButtonType.decelCruise, ButtonType.setCruise),
  "CcAslButtnCnclResPress": (ButtonType.cancel, ButtonType.resumeCruise),
}

SIMPLE_BUTTONS = {b.can_msg: b for b in BUTTONS if b.can_msg not in COMBO_BUTTONS}


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP

    self.pressed = dict.fromkeys({b.can_msg for b in BUTTONS}, False)
    # What a combo button reported when it went down, so its release matches.
    self.emitted: dict[str, ButtonType] = {}
    self.cruise_enabled_last = False
    self.main_cruise_pressed = False

  def update(self, ret: structs.CarState, ret_sp: structs.CarStateSP,
             can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]
    values = cp.vl["Steering_Data_FD1"]

    cruise_enabled = ret.cruiseState.enabled
    events: list[structs.CarState.ButtonEvent] = []
    main_cruise_just_pressed = False

    for signal, (enabled_type, disabled_type) in COMBO_BUTTONS.items():
      pressed = values[signal] == 1
      if pressed == self.pressed[signal]:
        continue
      self.pressed[signal] = pressed

      if pressed:
        event_type = enabled_type if cruise_enabled else disabled_type
        self.emitted[signal] = event_type
        events.append(structs.CarState.ButtonEvent(pressed=True, type=event_type))
      elif signal in self.emitted:
        events.append(structs.CarState.ButtonEvent(pressed=False, type=self.emitted.pop(signal)))

    for signal, button in SIMPLE_BUTTONS.items():
      pressed = values[signal] in button.values
      if pressed == self.pressed[signal]:
        continue
      self.pressed[signal] = pressed
      events.append(structs.CarState.ButtonEvent(pressed=pressed, type=button.event_type))
      if pressed and button.event_type == ButtonType.mainCruise:
        main_cruise_just_pressed = True

    # Turning cruise on with the main button should also set the speed to the current speed.
    # The car can take a frame or two to report engaged, so the press is remembered until it does.
    self.main_cruise_pressed |= main_cruise_just_pressed
    if cruise_enabled and not self.cruise_enabled_last and self.main_cruise_pressed:
      events.append(structs.CarState.ButtonEvent(pressed=True, type=ButtonType.setCruise))
      events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.setCruise))
      self.main_cruise_pressed = False
    elif cruise_enabled:
      self.main_cruise_pressed = False  # the press was for something else

    self.cruise_enabled_last = cruise_enabled
    ret.buttonEvents = list(ret.buttonEvents) + events
