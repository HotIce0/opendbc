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
  def __init__(self, packer):
    self.packer = packer

    self.mpc_lkas_counter = 0
    self.mpc_lkas_prepared = 0

  def generate_mpc_lkas_new_counter(self):
    counter = self.mpc_lkas_counter
    self.mpc_lkas_counter = int(self.mpc_lkas_counter + 1) & 0xF
    return counter

  # MPC -> Panda -> EPS
  def create_steering_control_torque(self, torque, enabled, mpc_lkas_msg):
    lkas_output = mpc_lkas_msg["LKAS_Output"]
    lkas_prepare = mpc_lkas_msg["LKASPrepare"]
    lkas_active = mpc_lkas_msg["LKAS_ACTIVE"]
    lkas_mode = mpc_lkas_msg["LKAS_Mode"]

    if enabled:
      if self.mpc_lkas_prepared == 0:
        self.mpc_lkas_prepared = 1
        lkas_output = 0
        lkas_prepare = 1
        lkas_active = 0
      else:
        lkas_output = torque
        lkas_prepare = 0
        lkas_active = 1
      lkas_mode = 2
    else: # enabled is False
      self.mpc_lkas_prepared = 0

    self.mpc_lkas_last_active = lkas_active
    lkas_counter = self.mpc_lkas_counter = self.generate_mpc_lkas_new_counter()
    values = {
      "SETME_0x1": mpc_lkas_msg["SETME_0x1"],
      "LeftLane": mpc_lkas_msg["LeftLane"],
      "Config": mpc_lkas_msg["Config"],
      "SETME2_0x1": mpc_lkas_msg["SETME2_0x1"],
      "Keep_Hands_On_Wheel": 0,
      "MPCErr": mpc_lkas_msg["MPCErr"],
      "SETME3_0x1": mpc_lkas_msg["SETME3_0x1"],
      "LKAS_Output": lkas_output,
      "LKASPrepare": lkas_prepare,
      "LKAS_ACTIVE": lkas_active,
      "TSRStatus": mpc_lkas_msg["TSRStatus"],
      "SETME4_0x1": mpc_lkas_msg["SETME4_0x1"],
      "RightLane": mpc_lkas_msg["RightLane"],
      "LKAS_Mode": lkas_mode,
      "SETME_0x0": mpc_lkas_msg["SETME_0x0"],
      "TSRResult": mpc_lkas_msg["TSRResult"],
      "Unknow1": mpc_lkas_msg["Unknow1"],
      "COUNTER": lkas_counter,
    }
    data = self.packer.make_can_msg("MPC_LKAS_CMD", CANBUS.main_bus, values)
    values["CHECKSUM"] = byd_checksum(data)
    return self.packer.make_can_msg("MPC_LKAS_CMD", CANBUS.main_bus, values)
