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


# Brake lamp sources, in the order they are tried. Registered alive-exempt (NaN rate) rather than
# with a declared rate: this is a cosmetic indicator, and nothing cosmetic should be able to put
# canValid down and disengage the car. Declaring a rate instead costs about half a second of
# invalid CAN at every startup while the 1Hz source is waited for, and leaves the door open to the
# same staleness class of bug that IPMA_Data hit (see get_can_parsers in
# opendbc/car/ford/carstate.py). The cost of the exemption is that a dead source reads as "lamps
# last seen off" instead of raising an error, which for an indicator is the right trade.
#
# Measured on a 2023 F-150 Raptor R across two drives: BrakeSysFeatures_2.BrkLamp_B_Rq never went
# high at all, through 399 frames of openpilot brake requests and plenty of driver braking, while
# BCM_Lamp_Stat_FD1.StopLghtOn_B_Stat was high for 83% of those same openpilot brake frames. So
# the slow message is the one that carries the signal on this platform and the fast one is dead.
# BCM only transmits at about 1.1Hz, which is the real limit on this indicator: it is a coarse
# "are the lamps lit" readout, and it can lag the lamps by up to a second.
BRAKE_LAMP_MSGS = (("BCM_Lamp_Stat_FD1", float("nan")), ("BrakeSysFeatures_2", float("nan")))


class CarStateExt:
  def __init__(self, CP, CP_SP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.brake_lamp_source: str | None = None
    # Raw accelerator pedal position, percent. ret.gasPressed is this > 1e-6, so the lightest
    # possible touch reads as a full override; the longitudinal extension needs the number
    # itself to tell a feathered pedal from a real one.
    self.accelerator_pedal_pc = 0.0

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

    self.accelerator_pedal_pc = float(cp.vl["EngVehicleSpThrottle"]["ApedPos_Pc_ActlArb"])

    if self.CP_SP.fordHud.brakeLightStatus:
      self._update_brake_lights(ret_sp, cp)

  def _update_brake_lights(self, ret_sp: structs.CarStateSP, cp: CANParser) -> None:
    """Report whether the vehicle's brake lamps are lit.

    Read off the bus rather than inferred from openpilot's own braking, because the lamps also
    light for the driver, for regen and for the stock ACC. There is deliberately no overlay of
    openpilot's brake request here: with openpilot longitudinal enabled the camera's ACCDATA
    carries the stock system's intent, not ours, so folding it in would light the indicator for a
    deceleration that is not happening.
    """
    status = ret_sp.fordBrakeLights
    lit: bool | None = None

    values = cp.vl["BCM_Lamp_Stat_FD1"]
    if "StopLghtOn_B_Stat" in values:
      lit = bool(values["StopLghtOn_B_Stat"])
      self.brake_lamp_source = "BCM_Lamp_Stat_FD1"
    else:
      values = cp.vl["BrakeSysFeatures_2"]
      if "BrkLamp_B_Rq" in values:
        lit = values["BrkLamp_B_Rq"] == 1
        self.brake_lamp_source = "BrakeSysFeatures_2"

    if lit is None:
      self.brake_lamp_source = None
      status.dataAvailable = False
      status.brakeLightsOn = False
      return

    status.dataAvailable = True
    status.brakeLightsOn = lit
