from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.values import CAR

NON_LINEAR_TORQUE_PARAMS = {
  CAR.BYD_HAN_EV_23: [1.807, 1.674, 0.04],
}


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "byd"
    ret.dashcamOnly = False

    ret.radarUnavailable = True
    ret.steerActuatorDelay = 0.3
    ret.steerLimitTimer = 0.5

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]

    if candidate in (CAR.BYD_HAN_EV_23):
      ret.steerControlType = structs.CarParams.SteerControlType.torque

    ret.minEnableSpeed = -1.
    ret.minSteerSpeed = 0.1 * CV.KPH_TO_MS
    ret.steerLimitTimer = 0.5

    if ret.steerControlType == structs.CarParams.SteerControlType.torque:
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    return ret