"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Ford parameter overrides, ported from BluePilot bp-7.0.
"""
from opendbc.car import structs
from opendbc.car.ford.values import FordSafetyFlags

# BluePilot runs a slightly longer actuator delay than upstream's 0.2 s. The PSCM is the actuator
# on Ford, and it takes a little longer to act on a command than a torque-controlled EPS does.
STEER_ACTUATOR_DELAY = 0.22


def apply_ford_params(ret: structs.CarParams, alpha_long: bool) -> None:
  """Applied at the end of CarInterface._get_params_sp, after all stock setup."""
  ret.steerActuatorDelay = STEER_ACTUATOR_DELAY

  # BluePilot makes the alpha longitudinal toggle authoritative on every Ford, rather than
  # forcing openpilot longitudinal on whenever the car has a usable radar. Two consequences:
  # a radar-equipped car can stay on Ford's own ACC, which upstream does not allow, and a CAN FD
  # car can run openpilot longitudinal, which upstream gates behind a debug panda build. The
  # matching gate in safety/modes/ford.h is lifted to match.
  ret.alphaLongitudinalAvailable = True
  ret.openpilotLongitudinalControl = bool(alpha_long)
  if ret.openpilotLongitudinalControl:
    ret.safetyConfigs[-1].safetyParam |= FordSafetyFlags.LONG_CONTROL.value
  else:
    ret.safetyConfigs[-1].safetyParam &= ~FordSafetyFlags.LONG_CONTROL.value


def apply_ford_params_sp(ret: structs.CarParamsSP) -> None:
  """Applied at the end of CarInterface._get_params_sp."""
  # Ford's cruise buttons can be pressed by openpilot, so speed control works on vehicles left
  # on the stock ACC.
  ret.intelligentCruiseButtonManagementAvailable = True
