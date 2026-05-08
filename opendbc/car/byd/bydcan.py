from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.byd.values import CANBUS


def byd_checksum(data: bytearray) -> int:
  byte_key = 0xAF
  sum_first = sum(byte >> 4 for byte in data) # Extract upper nibble.
  sum_second = sum(byte & 0xF for byte in data) # Extract lower nibble.

  remainder = sum_second >> 4

  sum_first += (byte_key & 0xF) # Low nibble of byte_key.
  sum_second += (byte_key >> 4) # High nibble of byte_key.

  # Inline inverse computation for each sum:
  # inv = (-sum + 0x9) & 0xF
  inv_first = ((-sum_first + 0x9) & 0xF)
  inv_second = ((-sum_second + 0x9) & 0xF)

  return (((inv_first + (5 - remainder)) << 4) + inv_second) & 0xFF


def byd_checksum_short(data: bytearray) -> int:
  # checksum for CHECKSUM_S 4byte
  pass


class BydCAN:
  LKAS_MODE_ACTIVE2 = 3
  LKAS_MODE_PASSIVE = 1

  def __init__(self, packer):
    self.packer = packer
    self.mpc_lkas_counter = 0

  def update_mpc_lkas_counter(self, val):
    self.mpc_lkas_counter = val

  def _generate_mpc_lkas_new_counter(self):
    counter = self.mpc_lkas_counter
    self.mpc_lkas_counter = int(self.mpc_lkas_counter + 1) & 0xF
    return counter

  # MPC -> Panda -> EPS
  def create_steering_control_torque(self, mpc_lkas_cmd_msg, torque, request_prepare, active, mode, eps_active):
    lkas_output = torque if active and eps_active else 0
    lkas_prepare = request_prepare
    lkas_active = active
    lkas_mode = mode
    left_lane = mpc_lkas_cmd_msg["LeftLane"]
    right_lane = mpc_lkas_cmd_msg["RightLane"]
    if active or request_prepare:
      left_lane = 1
      right_lane = 1
    lkas_counter = self._generate_mpc_lkas_new_counter()

    values = {
      "SETME_0x1": mpc_lkas_cmd_msg["SETME_0x1"],
      "LeftLane": left_lane,
      "Config": mpc_lkas_cmd_msg["Config"],
      "SETME2_0x1": mpc_lkas_cmd_msg["SETME2_0x1"],
      "Keep_Hands_On_Wheel": 0,
      "MPCErr": mpc_lkas_cmd_msg["MPCErr"],
      "SETME3_0x1": mpc_lkas_cmd_msg["SETME3_0x1"],
      "LKAS_Output": lkas_output,
      "LKASPrepare": lkas_prepare,
      "LKAS_ACTIVE": lkas_active,
      "TSRStatus": mpc_lkas_cmd_msg["TSRStatus"],
      "SETME4_0x1": mpc_lkas_cmd_msg["SETME4_0x1"],
      "RightLane": right_lane,
      "LKAS_Mode": lkas_mode,
      "SETME_0x0": mpc_lkas_cmd_msg["SETME_0x0"],
      "TSRResult": mpc_lkas_cmd_msg["TSRResult"],
      "Unknow1": mpc_lkas_cmd_msg["Unknow1"],
      "COUNTER": lkas_counter,
    }
    data = self.packer.make_can_msg("MPC_LKAS_CMD", CANBUS.main_bus, values)[1]
    values["CHECKSUM"] = byd_checksum(data)
    return self.packer.make_can_msg("MPC_LKAS_CMD", CANBUS.main_bus, values)
