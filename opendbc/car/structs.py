from dataclasses import dataclass as _dataclass, field, is_dataclass
from enum import Enum, StrEnum as _StrEnum, auto
from typing import dataclass_transform, get_origin

import os
import capnp
from opendbc.car.common.basedir import BASEDIR

capnp.remove_import_hook()
car = capnp.load(os.path.join(BASEDIR, "car.capnp"), imports=[BASEDIR])

CarState = car.CarState
RadarData = car.RadarData
CarControl = car.CarControl
CarParams = car.CarParams

CarStateT = capnp.lib.capnp._StructModule
RadarDataT = capnp.lib.capnp._StructModule
CarControlT = capnp.lib.capnp._StructModule
CarParamsT = capnp.lib.capnp._StructModule

# sunnypilot structs

AUTO_OBJ = object()


def auto_field():
  return AUTO_OBJ


@dataclass_transform()
def auto_dataclass(cls=None, /, **kwargs):
  cls_annotations = cls.__dict__.get('__annotations__', {})
  for name, typ in cls_annotations.items():
    current_value = getattr(cls, name)
    if current_value is AUTO_OBJ:
      origin_typ = get_origin(typ) or typ
      if isinstance(origin_typ, str):
        raise TypeError(f"Forward references are not supported for auto_field: '{origin_typ}'. Use a default_factory with lambda instead.")
      elif origin_typ in (int, float, str, bytes, list, tuple, bool) or is_dataclass(origin_typ):
        setattr(cls, name, field(default_factory=origin_typ))
      elif issubclass(origin_typ, Enum):  # first enum is the default
        setattr(cls, name, field(default=next(iter(origin_typ))))
      else:
        raise TypeError(f"Unsupported type for auto_field: {origin_typ}")

  # TODO: use slots, this prevents accidentally setting attributes that don't exist
  return _dataclass(cls, **kwargs)


class StrEnum(_StrEnum):
  @staticmethod
  def _generate_next_value_(name, *args):
    # auto() defaults to name.lower()
    return name


@auto_dataclass
class CarParamsSP:
  flags: int = auto_field()        # flags for car specific quirks
  safetyParam: int = auto_field()  # flags for custom safety flags
  pcmCruiseSpeed: bool = auto_field()
  intelligentCruiseButtonManagementAvailable: bool = auto_field()
  enableGasInterceptor: bool = auto_field()

  neuralNetworkLateralControl: 'CarParamsSP.NeuralNetworkLateralControl' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl())
  fordLateralTuning: 'CarParamsSP.FordLateralTuning' = field(default_factory=lambda: CarParamsSP.FordLateralTuning())
  fordLongitudinalTuning: 'CarParamsSP.FordLongitudinalTuning' = field(default_factory=lambda: CarParamsSP.FordLongitudinalTuning())
  fordHud: 'CarParamsSP.FordHud' = field(default_factory=lambda: CarParamsSP.FordHud())

  @auto_dataclass
  class FordLateralTuning:
    primaryControl: int = auto_field()
    lowSpeedFactor: float = auto_field()
    highSpeedFactor: float = auto_field()
    highSpeedDampening: float = auto_field()
    laneChangeFactor: float = auto_field()
    humanTurnDetection: bool = auto_field()
    laneChangeFactorCurv: float = auto_field()
    blendRatioLow: float = auto_field()
    blendRatioHigh: float = auto_field()
    lanePositioning: bool = auto_field()
    pathOffset: float = auto_field()
    laneFullMode: bool = auto_field()
    customProfile: int = auto_field()
    lanePositioningGain: float = auto_field()

  @auto_dataclass
  class FordLongitudinalTuning:
    followControl: bool = auto_field()
    downhillCompensation: bool = auto_field()

  @auto_dataclass
  class FordHud:
    handsFreeClusterMsg: bool = auto_field()
    driverMonitorCanMsg: bool = auto_field()

  @auto_dataclass
  class NeuralNetworkLateralControl:
    model: 'CarParamsSP.NeuralNetworkLateralControl.Model' = field(default_factory=lambda: CarParamsSP.NeuralNetworkLateralControl.Model())
    fuzzyFingerprint: bool = auto_field()

    @auto_dataclass
    class Model:
      path: str = auto_field()
      name: str = auto_field()


@auto_dataclass
class ModularAssistiveDrivingSystem:
  state: 'ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState' = field(
    default_factory=lambda: ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState.disabled
  )
  enabled: bool = auto_field()
  active: bool = auto_field()
  available: bool = auto_field()

  class ModularAssistiveDrivingSystemState(StrEnum):
    disabled = auto()
    paused = auto()
    enabled = auto()
    softDisabling = auto()
    overriding = auto()


@auto_dataclass
class IntelligentCruiseButtonManagement:
  state: 'IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState' = field(
    default_factory=lambda: IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState.inactive
  )
  sendButton: 'IntelligentCruiseButtonManagement.SendButtonState' = field(
    default_factory=lambda: IntelligentCruiseButtonManagement.SendButtonState.none
  )
  vTarget: float = auto_field()

  class IntelligentCruiseButtonManagementState(StrEnum):
    inactive = auto()
    preActive = auto()
    increasing = auto()
    decreasing = auto()
    holding = auto()

  class SendButtonState(StrEnum):
    none = auto()
    increase = auto()
    decrease = auto()


@auto_dataclass
class LeadData:
  dRel: float = auto_field()
  yRel: float = auto_field()
  vRel: float = auto_field()
  aRel: float = auto_field()
  vLead: float = auto_field()
  dPath: float = auto_field()
  vLat: float = auto_field()
  vLeadK: float = auto_field()
  aLeadK: float = auto_field()
  fcw: bool = auto_field()
  status: bool = auto_field()
  aLeadTau: float = auto_field()
  modelProb: float = auto_field()
  radar: bool = auto_field()
  radarTrackId: int = auto_field()

  aLeadDEPRECATED: float = auto_field()


@auto_dataclass
class CarControlSP:
  mads: 'ModularAssistiveDrivingSystem' = field(default_factory=lambda: ModularAssistiveDrivingSystem())
  params: list['CarControlSP.Param'] = auto_field()
  leadOne: 'LeadData' = field(default_factory=lambda: LeadData())
  leadTwo: 'LeadData' = field(default_factory=lambda: LeadData())
  intelligentCruiseButtonManagement: 'IntelligentCruiseButtonManagement' = field(default_factory=lambda: IntelligentCruiseButtonManagement())
  fordLateral: 'CarControlSP.FordLateral' = field(default_factory=lambda: CarControlSP.FordLateral())

  @auto_dataclass
  class FordLateral:
    modelCurvatures: list[float] = auto_field()
    lateralDelay: float = auto_field()
    laneChangeState: int = auto_field()
    laneChangeDirection: int = auto_field()
    modelPositionY: list[float] = auto_field()
    laneLineLeftY: float = auto_field()
    laneLineRightY: float = auto_field()
    laneLineLeftProb: float = auto_field()
    laneLineRightProb: float = auto_field()
    alertType: str = auto_field()

  @auto_dataclass
  class Param:
    key: str = auto_field()
    value: bytes = auto_field()
    type: 'CarControlSP.ParamType' = field(
      default_factory=lambda: CarControlSP.ParamType.string
    )

  class ParamType(StrEnum):
    string = auto()
    bool = auto()
    int = auto()
    float = auto()
    time = auto()
    json = auto()
    bytes = auto()


@auto_dataclass
class CarStateSP:
  speedLimit: float = auto_field()
