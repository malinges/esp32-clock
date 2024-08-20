#pragma once

#include <esp_err.h>
#include <driver/gpio.h>
#include <driver/rmt_types.h>
#include <driver/rmt_tx.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  gpio_num_t clk_gpio_num;
  gpio_num_t dio_gpio_num;
  uint32_t frequency_hz;
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
} tm1637_state_t;

typedef tm1637_state_t *tm1637_handle_t;

esp_err_t tm1637_init(tm1637_config_t *config, tm1637_handle_t *ret_tm1637);
esp_err_t tm1637_transmit_bytes(tm1637_handle_t tm1637, const uint8_t *bytes, size_t bytes_size);
esp_err_t tm1637_deinit(tm1637_handle_t tm1637);

void rmt_test(void);

#ifdef __cplusplus
}
#endif
