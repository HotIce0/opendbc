from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_meas_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd.bydcan import BydCAN
from opendbc.car.byd.values import CarControllerParams

VisualAlert = structs.CarControl.HUDControl.VisualAlert
ButtonType = structs.CarState.ButtonEvent.Type
LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.can = BydCAN(self.packer)
    self.apply_torque_last = 0

  def update(self, CC, CS, now_nanos):
    # car control running in 100Hz
    actuators = CC.actuators
    can_sends = []

    if (self.frame % self.params.STEER_SETP) == 0:
      # steer torque
      apply_torque = int(round(actuators.torque * self.params.STEER_MAX))
      apply_torque = apply_meas_steer_torque_limits(apply_torque, self.apply_torque_last,
        CS.out.steeringTorqueEps, self.params)
      pack = self.can.create_steering_control_torque(apply_torque,
                                                    CC.enabled)
      can_sends.append(pack)

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.apply_torque_last = apply_torque
    self.frame += 1
    return new_actuators, can_sends
