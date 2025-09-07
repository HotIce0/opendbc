from math import exp
from opendbc.car import get_safety_config, structs, get_friction
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase, TorqueFromLateralAccelCallbackType, FRICTION_THRESHOLD, LatControlInputs
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.values import BydSafetyFlags, CAR

NON_LINEAR_TORQUE_PARAMS = {
  CAR.BYD_HAN_EV_23: [1.807, 1.674, 0.04],
}


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def torque_from_lateral_accel_siglin(self, latcontrol_inputs: LatControlInputs, torque_params: structs.CarParams.LateralTorqueTuning,
                  lateral_accel_error: float, lateral_accel_deadzone: float, friction_compensation: bool, gravity_adjusted: bool) -> float:
    friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD, torque_params, friction_compensation)

    def sig(val):
      # https://timvieira.github.io/blog/post/2014/02/11/exp-normalize-trick
      if val >= 0:
        return 1 / (1 + exp(-val)) - 0.5
      else:
        z = exp(val)
        return z / (1 + z) - 0.5

    # The "lat_accel vs torque" relationship is assumed to be the sum of "sigmoid + linear" curves
    # An important thing to consider is that the slope at 0 should be > 0 (ideally >1)
    # This has big effect on the stability about 0 (noise when going straight)
    non_linear_torque_params = NON_LINEAR_TORQUE_PARAMS.get(self.CP.carFingerprint)
    assert non_linear_torque_params, "The params are not defined"
    a, b, c = non_linear_torque_params
    steer_torque = (sig(latcontrol_inputs.lateral_acceleration * a) * b) + (latcontrol_inputs.lateral_acceleration * c)
    return float(steer_torque) + friction

  def torque_from_lateral_accel(self) -> TorqueFromLateralAccelCallbackType:
    if self.CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:
      return self.torque_from_lateral_accel_siglin
    else:
      return self.torque_from_lateral_accel_linear

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