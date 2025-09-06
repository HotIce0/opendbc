from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd.bydcan import BydCAN
# from opendbc.car.byd.bydcan import create_can_steer_command, create_lkas_hud, create_accel_command

VisualAlert = structs.CarControl.HUDControl.VisualAlert
ButtonType = structs.CarState.ButtonEvent.Type
LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.can = BydCAN(self.packer)

  def _do_update_50hz(self, CC, CS, now_nanos):
    pass

  def update(self, CC, CS, now_nanos):
    # CC CarControl
    # CS CarState
    actuators = CC.actuators
    can_sends = []

    if (self.frame % 2) == 0:
      can_send = self._do_update_50hz(CC, CS, now_nanos)
      can_sends.append(can_send)

    new_actuators = actuators.as_builder()
    
