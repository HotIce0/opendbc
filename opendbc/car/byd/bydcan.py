from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.tesla.values import CANBUS, CarControllerParams


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