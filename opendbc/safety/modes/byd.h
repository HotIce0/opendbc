#include "opendbc/safety/safety_declarations.h"

#define BYD_CANADDR_IPB                 0x123

static uint32_t byd_compute_checksum(const CANPacket_t *msg) {
  UNUSED(msg);
  return 0;
}

static uint32_t byd_get_checksum(const CANPacket_t *msg) {
  UNUSED(msg);
  return 0;
}

static uint8_t byd_get_counter(const CANPacket_t *msg) {
  UNUSED(msg);
  return 0;
}

static bool byd_tx_hook(const CANPacket_t *msg) {
  UNUSED(msg);
  return true;
}

static void byd_rx_hook(const CANPacket_t *msg) {
  UNUSED(msg);
}

static safety_config byd_init(uint16_t param) {
  UNUSED(param);

    static const CanMsg BYD_TX_MSGS[] = {
    {482, 0, 8, .check_relay = true}, // STEERING_MODULE_ADAS
    {790, 0, 8, .check_relay = true}, // LKAS_HUD_ADAS
  };

  // static const CanMsg BYD_TX_LONG_MSGS[] = {
  //   {482, 0, 8, .check_relay = true}, // STEERING_MODULE_ADAS
  //   {790, 0, 8, .check_relay = true}, // LKAS_HUD_ADAS
  //   {814, 0, 8, .check_relay = true}  // ACC_CMD
  // };

  static RxCheck byd_rx_checks[] = {
    {.msg = {{287, 0, 5, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}}, // STEER_MODULE_2
    {.msg = {{496, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},  // WHEEL_SPEED2
    {.msg = {{508, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},  // STEERING_TORQUE
    {.msg = {{834, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},  // PEDAL
    {.msg = {{944, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 20U}, { 0 }, { 0 }}},  // PCM_BUTTONS
    {.msg = {{814, 2, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}},  // ACC_CMD
  };

  return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .get_counter = byd_get_counter,
  .get_checksum = byd_get_checksum,
  .compute_checksum = byd_compute_checksum,
};
