from dataclasses import dataclass, field
from enum import IntFlag
from opendbc.car import Bus, DbcDict, PlatformConfig, Platforms, CarSpecs, STD_CARGO_KG
from opendbc.car.structs import CarParams, CarState
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


class CANBUS:
  main_bus = 0
  radar_bus = 1
  cam_bus = 2


class BydSafetyFlags(IntFlag):
  LONG_CONTROL = 1


@dataclass
class BydCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))
  #todo add docs and harness info


@dataclass
class BydPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt:"byd_common"})


class CAR(Platforms):
  BYD_HAN_EV_23 = BydPlatformConfig(
    [BydCarDocs("BYD HAN EV 23")],
    [CarSpecs(mass=1940 + STD_CARGO_KG, wheelbase=2.92, steerRatio=16.5,
              centerToFrontRatio=0.44, tireStiffnessFactor=1.0)]
  )


class CarControllerParams:
  def __init__(self, CP):
    self.STEER_SETP = 2 # 50Hz

    self.STEER_ERROR_MAX = 46
    if CP.carFingerprint == CAR.BYD_HAN_EV_23:
      self.STEER_MAX = 300
      self.STEER_DELTA_UP = 6
      self.STEER_DELTA_DOWN = 6


STEER_THRESHOLD = 1

GEAR_MAP = {
  4: CarState.GearShifter.drive,
  2: CarState.GearShifter.reverse,
  3: CarState.GearShifter.neutral,
  1: CarState.GearShifter.park,
}

FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.UDS_VERSION_REQUEST],
      [StdQueries.UDS_VERSION_RESPONSE],
      bus=0,
    )],
)

DBC = CAR.create_dbc_map()
