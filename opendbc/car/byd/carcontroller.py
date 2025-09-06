from opendbc.can.packer import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarControllerBase
# from opendbc.car.byd.bydcan import create_can_steer_command, create_lkas_hud, create_accel_command

VisualAlert = structs.CarControl.HUDControl.VisualAlert
ButtonType = structs.CarState.ButtonEvent.Type
LongCtrlState = structs.CarControl.Actuators.LongControlState

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.apply_angle = 0

  def update(self, CC, CS, now_nanos):
    can_sends = []
