#include "opendbc/safety/declarations.h"

static bool byd_longitudinal = false;

static bool byd_tx_hook(const CANPacket_t *msg) {
  SAFETY_UNUSED(msg);
  return true;
}

static void byd_rx_hook(const CANPacket_t *msg) {
  // Main Bus
  if (msg->bus == 0) {
    // PEDAL
    if (msg->addr == 834) {
      gas_pressed = msg->data[0] != 0;
      brake_pressed = msg->data[1] != 0;
    }
    // ESC
    if (msg->addr == 289) {
      vehicle_moving = (((msg->data[1] & 0x0FU) << 8) | msg->data[0]) != 0;
    }
  }
  // Cam Bus
  else if (msg->bus == 2) {
    // ACC_HUD_ADAS
    if (msg->addr == 813) {
      // CRUISE_STATE not in (3, 5)
      uint8_t cruise_state = (msg->data[5] >> 4) & 0x0FU;
      if (cruise_state == 3 || cruise_state == 5) {
        controls_allowed = true;
      } else {
        controls_allowed = false;
      }
    }
  }
}

static safety_config byd_init(uint16_t param) {
  SAFETY_UNUSED(param);

  static const CanMsg BYD_TX_MSGS[] = {
    {790, 0, 8, .check_relay = true}, // MPC_LKAS_CMD
    {792, 2, 8, .check_relay = true}, // STEERING_TORQUE
  };

  static const CanMsg BYD_TX_LONG_MSGS[] = {
    {790, 0, 8, .check_relay = true}, // MPC_LKAS_CMD
    {792, 2, 8, .check_relay = true}, // STEERING_TORQUE
    {814, 0, 8, .check_relay = true}, // ACC_CMD
  };

  static RxCheck byd_rx_checks[] = {
    {.msg = {{578, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // DRIVE_STATE
    {.msg = {{544, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // BRAKE_APPLIED
    {.msg = {{834, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // PEDAL
    {.msg = {{287, 0, 5, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // STEER_MODULE
    {.msg = {{792, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // STEERING_TORQUE
    {.msg = {{813, 2, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // ACC_HUD_ADAS
    {.msg = {{307, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // STALKS
    {.msg = {{660, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // METER_CLUSTER
    {.msg = {{790, 2, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // MPC_LKAS_CMD
  };

  const uint16_t BYD_FLAG_LONG_CONTROL = 1;
  byd_longitudinal = GET_FLAG(param, BYD_FLAG_LONG_CONTROL);

  safety_config ret;
  if (byd_longitudinal) {
    ret = BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_LONG_MSGS);
  } else {
    ret = BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
  }
  return ret;
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
};
