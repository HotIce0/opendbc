from opendbc.car import Bus, structs
from opendbc.can.parser import CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, CANBUS, STEER_THRESHOLD, GEAR_MAP


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    # car Speed
    ret.vEgoRaw = ret.vl["DRIVE_STATE"]["WHEELSPEED_HR"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = ret.vl["BRAKE_APPLIED"]["STANDSTILL"] == 1

    # Gas pedal
    ret.gasPressed = cp.vl["PEDAL"]["GAS_PEDAL"] > 0

    # Breake pedal
    ret.brake = cp.vl["PEDAL"]["BRAKE_PEDAL"]
    ret.brakePressed = ret.brake > 0

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["STEER_MODULE"]["STEER_ANGLE"]
    ret.steeringRateDeg = cp.vl["STEER_MODULE"]["STEERING_RATE"]
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]["Steer_Torque_Sensor"]
    ret.steeringTorqueEps = cp.vl["STEER_MODULE"]["MAIN_TORQUE"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)
    ret.steerFaultPermanent = cp.vl["STEERING_TORQUE"]["TORQUE_FAILED"] != 0
    ret.steerFaultTemporary = cp.vl["STEERING_TORQUE"]["TORQUE_FAILED"] != 0

    ret.stockAeb = cp_cam.vl["ACC_HUD_ADAS"]["AEB"] == 1
    ret.stockFcw = cp_cam.vl["ACC_HUD_ADAS"]["FCW"] == 1 # Forward Collision Warning

    # Cruise state
    ret.cruiseState.enabled = cp_cam.vl["ACC_HUD_ADAS"]["CRUISE_STATE"] in (3, 5) # (Active, Override)
    ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SET_SPEED"] * CV.KPH_TO_MS
    ret.cruiseState.available = cp_cam.vl["ACC_HUD_ADAS"]["CRUISE_STATE"] not in (8, 9) # (Failure, PermanentFailure)
    ret.cruiseState.standstill = ret.standstill

    # Gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["DRIVE_STATE"]["GEAR"]), CarState.GearShifter.unknown)

    # button presses
    ret.leftBlinker = cp.vl["STALKS"]["LEFT_BLINKER"] == 1
    ret.rightBlinker = cp.vl["STALKS"]["RIGHT_BLINKER"] == 1

    # lock info
    ret.doorOpen = any(cp.vl["METER_CLUSTER"]["FRONT_LEFT_DOOR"],
                       cp.vl["METER_CLUSTER"]["FRONT_RIGHT_DOOR"],
                       cp.vl["METER_CLUSTER"]["BACK_LEFT_DOOR"],
                       cp.vl["METER_CLUSTER"]["BACK_RIGHT_DOOR"])
    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]["SEATBELT_DRIVER"] == 1

    # blindspot sensors
    ret.leftBlindspot = False
    ret.rightBlindspot = False

    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_signals = [
      ("STEER_MODULE", 50),
      ("STALKS", 50),
      ("DRIVE_STATE", 50),
      ("METER_CLUSTER", 50),
      ("STEERING_TORQUE", 50),
      ("PEDAL", 50),
    ]

    cam_signals = [
      ("ACC_HUD_ADAS", 50),
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [pt_signals], CANBUS.main_bus),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [cam_signals], CANBUS.cam_bus),
    }
