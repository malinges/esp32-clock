#pragma once

#include <stdint.h>

#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/rmt_types.h>
#include <driver/rmt_tx.h>

#define TM1637_MAX_DIGITS (6)

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  gpio_num_t clk_gpio_num;
  gpio_num_t dio_gpio_num;
  uint32_t frequency_hz;
  uint8_t digits_num;
} tm1637_config_t;

typedef struct {
  rmt_symbol_word_t start_symbol;
  rmt_symbol_word_t bit0_symbol;
  rmt_symbol_word_t bit1_symbol;
  rmt_symbol_word_t ack1_symbol;
  rmt_symbol_word_t ack2_symbol;
  rmt_symbol_word_t padding_symbol;
  rmt_symbol_word_t stop_symbol;
} tm1637_encoder_config_t;

typedef struct {
  rmt_channel_handle_t clk_tx_channel;
  rmt_encoder_handle_t clk_encoder;
  tm1637_encoder_config_t clk_encoder_cfg;
  rmt_transmit_config_t clk_transmit_cfg;
  rmt_channel_handle_t dio_tx_channel;
  rmt_encoder_handle_t dio_encoder;
  tm1637_encoder_config_t dio_encoder_cfg;
  rmt_transmit_config_t dio_transmit_cfg;
  rmt_sync_manager_handle_t sync_manager;
  uint8_t digits_num;
  uint8_t cmd1_buf;
  uint8_t cmd2_buf[1 + TM1637_MAX_DIGITS];
  uint8_t cmd3_buf;
} tm1637_state_t;

typedef tm1637_state_t *tm1637_handle_t;

esp_err_t tm1637_init(tm1637_config_t *config, tm1637_handle_t *ret_tm1637);
esp_err_t tm1637_set_digit_raw(tm1637_handle_t tm1637, uint8_t idx, uint8_t value);
esp_err_t tm1637_set_digit_number(tm1637_handle_t tm1637, uint8_t idx, uint8_t value, bool set_dot);
esp_err_t tm1637_set_brightness(tm1637_handle_t tm1637, uint8_t brightness);
esp_err_t tm1637_set_display_enabled(tm1637_handle_t tm1637, bool enabled);
esp_err_t tm1637_update(tm1637_handle_t tm1637);
esp_err_t tm1637_deinit(tm1637_handle_t tm1637);

#ifdef __cplusplus
}
#endif
