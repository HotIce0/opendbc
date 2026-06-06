import copy
from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, CANBUS, STEER_THRESHOLD, GEAR_MAP

GearShifter = structs.CarState.GearShifter

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

    self.mpc_lkas_cmd_msg = None
    self.eps_steering_torque_msg = None
    self.eps_prepared = False
    self.eps_activated = False
    self.acc_cmd_msg = None

    self.mpc_lkas_output = 0
    self.mpc_lkas_active = False
    self.mpc_lkas_request_prepare = False

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # car Speed
    ret.vEgoRaw = cp.vl["ESC"]["VEHICLE_SPEED"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = cp.vl["BRAKE_APPLIED"]["STANDSTILL"] == 1

    # Gas pedal
    ret.gasPressed = cp.vl["PEDAL"]["GAS_PEDAL"] > 0

    # Breake pedal
    ret.brake = cp.vl["PEDAL"]["BRAKE_PEDAL"]
    ret.brakePressed = ret.brake > 0

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["STEER_MODULE"]["STEER_ANGLE"]
    ret.steeringRateDeg = cp.vl["STEER_MODULE"]["STEERING_RATE"]
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]["Steer_Torque_Sensor"]
    ret.steeringTorqueEps = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"]
    # TODO: looks like STEER_THRESHOLD is too small
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)
    ret.steerFaultPermanent = cp.vl["STEERING_TORQUE"]["TORQUE_FAILED"] != 0
    ret.steerFaultTemporary = cp.vl["STEERING_TORQUE"]["TORQUE_FAILED"] != 0

    ret.stockAeb = cp_cam.vl["ACC_HUD_ADAS"]["AEB"] == 1
    ret.stockFcw = cp_cam.vl["ACC_HUD_ADAS"]["FCW"] == 1 # Forward Collision Warning

    # Cruise state
    ret.cruiseState.enabled = cp_cam.vl["ACC_HUD_ADAS"]["CRUISE_STATE"] in (3, 5) # (Active, Override)
    ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SET_SPEED"] * 10 * CV.KPH_TO_MS
    ret.cruiseState.available = cp_cam.vl["ACC_HUD_ADAS"]["CRUISE_STATE"] not in (8, 9) # (Failure, PermanentFailure)
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special

    # Gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["DRIVE_STATE"]["GEAR"]), GearShifter.unknown)

    # button presses
    ret.leftBlinker = cp.vl["STALKS"]["TURN_SIGNAL_SWITCH"] == 3 # LeftSteeringLongLift
    ret.rightBlinker = cp.vl["STALKS"]["TURN_SIGNAL_SWITCH"] == 5 # RightSteeringLongLift

    # lock info
    ret.doorOpen = any([cp.vl["METER_CLUSTER"]["FRONT_LEFT_DOOR"],
                        cp.vl["METER_CLUSTER"]["FRONT_RIGHT_DOOR"],
                        cp.vl["METER_CLUSTER"]["BACK_LEFT_DOOR"],
                        cp.vl["METER_CLUSTER"]["BACK_RIGHT_DOOR"]])
    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]["SEATBELT_DRIVER"] != 1

    # blindspot sensors
    # ret.leftBlindspot = False
    # ret.rightBlindspot = False

    # Messages needed by carcontroller
    # for generate MPC_LKAS_CMD
    self.mpc_lkas_cmd_msg = copy.copy(cp_cam.vl["MPC_LKAS_CMD"])
    self.eps_prepared = cp.vl["STEERING_TORQUE"]["LKSPrepare"] != 0
    self.eps_activated = cp.vl["STEERING_TORQUE"]["Cruise_Activated"] != 0
    # for generate STEERING_TORQUE
    self.eps_steering_torque_msg = copy.copy(cp.vl["STEERING_TORQUE"])
    self.mpc_lkas_output = cp_cam.vl["MPC_LKAS_CMD"]["LKAS_Output"]
    self.mpc_lkas_active = cp_cam.vl["MPC_LKAS_CMD"]["LKAS_ACTIVE"] != 0
    self.mpc_lkas_request_prepare = cp_cam.vl["MPC_LKAS_CMD"]["LKASPrepare"] != 0
    # for generate ACC_CMD
    self.acc_cmd_msg = copy.copy(cp_cam.vl["ACC_CMD"])
    return ret

  @staticmethod
  def get_can_parsers(CP):
    # TODO specify including messages
    # pt_signals = [
    #   ("STEER_MODULE", 100),
    #   ("STALKS", 20),
    #   ("DRIVE_STATE", 50),
    #   ("METER_CLUSTER", 20),
    #   ("STEERING_TORQUE", 50),
    #   ("PEDAL", 50),
    #   ("BRAKE_APPLIED", 50),
    # ]

    # cam_signals = [
    #   ("ACC_HUD_ADAS", 50),
    # ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.main_bus),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.cam_bus),
    }
