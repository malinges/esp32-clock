#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_check.h>
#include <esp_log.h>

#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>

#include "tm1637.h"

// TODO:
// - add to public API:
//   - configurable intr_priority
// - make updating display non-blocking (no rmt_tx_wait_all_done())
// - double buffering?
// - find a way to transmit everything in a single rmt transaction
// - move useful definitions to header file
// - document:
//   - relation between resolution, symbols and frequency
//   - symbol design decisions
//   - rmt buffering issues
// - make this an esp-idf component

static const char *TAG = "tm1637";

#define TM1637_MEM_ALLOC_CAPS                   (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#define TM1637_RMT_CLK_SRC                      RMT_CLK_SRC_DEFAULT
#define TM1637_RMT_RESOLUTION_HZ                (80 * 1000 * 1000) // 80MHz resolution, 1 tick = 12.5ns
#define TM1637_RMT_MEM_BLOCK_SYMBOLS            (48)
#define TM1637_RMT_TRANS_QUEUE_DEPTH            (3)

#define TM1637_BITS_PER_PERIOD                  (4)
#define TM1637_RMT_SYMBOLS_PER_BYTE             (12) // (1 start + 8 * 1 bit + 1 ack1 + 1 ack2 + 1 stop)
#define TM1637_MAX_SYMBOL_TICKS_MULTIPLIER      (3)
#define TM1637_CHANNELS_NUM                     (2)

#define RMT_MAX_TICKS                           ((1U << 15) - 1)
#define CEIL(x, y)                              (((x) + (y) - 1) / (y))
#define TM1637_QUARTER_PERIOD_IN_TICKS(freq)    CEIL(TM1637_RMT_RESOLUTION_HZ, (freq) * TM1637_BITS_PER_PERIOD)
#define TM1637_MIN_FREQUENCY                    CEIL(CEIL(TM1637_RMT_RESOLUTION_HZ, RMT_MAX_TICKS / TM1637_MAX_SYMBOL_TICKS_MULTIPLIER), TM1637_BITS_PER_PERIOD)
#define TM1637_MAX_FREQUENCY                    (TM1637_RMT_RESOLUTION_HZ / TM1637_BITS_PER_PERIOD)

#define TM1637_CMD1_BASE                        (0x40)
#define TM1637_CMD1_OPERATION_WRITE             (0x00)
#define TM1637_CMD1_OPERATION_READ              (0x02)
#define TM1637_CMD1_ADDR_AUTOINC                (0x00)
#define TM1637_CMD1_ADDR_FIXED                  (0x04)

#define TM1637_CMD2_BASE                        (0xc0) // apply bitwise OR with 0-5 to select start address,
                                                       // and follow with segment data bytes

#define TM1637_CMD3_BASE                        (0x80)
#define TM1637_CMD3_BRIGHTNESS_0                (0x00)
#define TM1637_CMD3_BRIGHTNESS_1                (0x01)
#define TM1637_CMD3_BRIGHTNESS_2                (0x02)
#define TM1637_CMD3_BRIGHTNESS_3                (0x03)
#define TM1637_CMD3_BRIGHTNESS_4                (0x04)
#define TM1637_CMD3_BRIGHTNESS_5                (0x05)
#define TM1637_CMD3_BRIGHTNESS_6                (0x06)
#define TM1637_CMD3_BRIGHTNESS_7                (0x07)
#define TM1637_CMD3_DISPLAY_OFF                 (0x00)
#define TM1637_CMD3_DISPLAY_ON                  (0x08)

static const uint8_t tm1637_symbols[] = {
                // XGFEDCBA
        0x3f, // 0b00111111,    // 0
        0x06, // 0b00000110,    // 1
        0x5b, // 0b01011011,    // 2
        0x4f, // 0b01001111,    // 3
        0x66, // 0b01100110,    // 4
        0x6d, // 0b01101101,    // 5
        0x7d, // 0b01111101,    // 6
        0x07, // 0b00000111,    // 7
        0x7f, // 0b01111111,    // 8
        0x6f, // 0b01101111,    // 9
        0x77, // 0b01110111,    // A
        0x7c, // 0b01111100,    // b
        0x39, // 0b00111001,    // C
        0x5e, // 0b01011110,    // d
        0x79, // 0b01111001,    // E
        0x71, // 0b01110001     // F
        0x40, // 0b01000000     // minus sign
};

// CLK symbols

#define TM1637_CLK_START_SYMBOL(freq)   ((rmt_symbol_word_t) {  \
    .level0 = 1,                                                \
    .duration0 = 1 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
    .level1 = 1,                                                \
    .duration1 = 1 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
})

#define TM1637_CLK_BIT_SYMBOL(freq)     ((rmt_symbol_word_t) {  \
    .level0 = 0,                                                \
    .duration0 = 2 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
    .level1 = 1,                                                \
    .duration1 = 2 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
})

#define TM1637_CLK_ACK1_SYMBOL(freq)    ((rmt_symbol_word_t) {  \
    .level0 = 1,                                                \
    .duration0 = 1 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
    .level1 = 0,                                                \
    .duration1 = 2 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
})

#define TM1637_CLK_ACK2_SYMBOL(freq)    ((rmt_symbol_word_t) {  \
    .level0 = 1,                                                \
    .duration0 = 1 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
    .level1 = 1,                                                \
    .duration1 = 1 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
})

#define TM1637_CLK_PADDING_SYMBOL(freq) ((rmt_symbol_word_t) {  \
    .level0 = 1,                                                \
    .duration0 = 1 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
    .level1 = 1,                                                \
    .duration1 = 1 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
})

#define TM1637_CLK_STOP_SYMBOL(freq)    ((rmt_symbol_word_t) {  \
    .level0 = 0,                                                \
    .duration0 = 2 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
    .level1 = 1,                                                \
    .duration1 = 3 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
})

// DIO symbols

#define TM1637_DIO_START_SYMBOL(freq)   ((rmt_symbol_word_t) {  \
    .level0 = 0,                                                \
    .duration0 = 2 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
    .level1 = 0,                                                \
    .duration1 = 1 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
})

#define TM1637_DIO_BIT0_SYMBOL(freq)    ((rmt_symbol_word_t) {  \
    .level0 = 0,                                                \
    .duration0 = 2 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
    .level1 = 0,                                                \
    .duration1 = 2 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
})

#define TM1637_DIO_BIT1_SYMBOL(freq)    ((rmt_symbol_word_t) {  \
    .level0 = 1,                                                \
    .duration0 = 2 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
    .level1 = 1,                                                \
    .duration1 = 2 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
})

#define TM1637_DIO_ACK1_SYMBOL(freq)    ((rmt_symbol_word_t) {  \
    .level0 = 0,                                                \
    .duration0 = 1 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
    .level1 = 0,                                                \
    .duration1 = 2 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
})

#define TM1637_DIO_ACK2_SYMBOL(freq)    ((rmt_symbol_word_t) {  \
    .level0 = 0,                                                \
    .duration0 = 1 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
    .level1 = 0,                                                \
    .duration1 = 1 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
})

#define TM1637_DIO_PADDING_SYMBOL(freq) ((rmt_symbol_word_t) {  \
    .level0 = 0,                                                \
    .duration0 = 1 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
    .level1 = 0,                                                \
    .duration1 = 1 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
})

#define TM1637_DIO_STOP_SYMBOL(freq)    ((rmt_symbol_word_t) {  \
    .level0 = 0,                                                \
    .duration0 = 2 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
    .level1 = 1,                                                \
    .duration1 = 2 * TM1637_QUARTER_PERIOD_IN_TICKS(freq),      \
})

static IRAM_ATTR size_t tm1637_encoder_callback(const void *data, size_t data_size,
                               size_t symbols_written, size_t symbols_free,
                               rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    const size_t data_pos = symbols_written / TM1637_RMT_SYMBOLS_PER_BYTE;
    size_t symbol_pos = 0;
    const uint8_t *data_bytes = (uint8_t *)data;
    const tm1637_encoder_config_t *cfg = (const tm1637_encoder_config_t *)arg;

    if (symbols_free >= TM1637_RMT_SYMBOLS_PER_BYTE && data_pos < data_size) {
        // If this is the first byte, send start symbol, otherwise send padding symbol
        if (data_pos == 0) {
            symbols[symbol_pos++] = cfg->start_symbol;
        } else {
            symbols[symbol_pos++] = cfg->padding_symbol;
        }

        // Send bit symbols
        for (uint8_t bitmask = 0x01; bitmask != 0; bitmask <<= 1) {
            if (data_bytes[data_pos] & bitmask) {
                symbols[symbol_pos++] = cfg->bit1_symbol;
            } else {
                symbols[symbol_pos++] = cfg->bit0_symbol;
            }
        }

        // Send ack symbols
        symbols[symbol_pos++] = cfg->ack1_symbol;
        symbols[symbol_pos++] = cfg->ack2_symbol;

        // If this is the last byte, send stop symbol and signal completion, otherwise send padding symbol
        if (data_pos == data_size - 1) {
            symbols[symbol_pos++] = cfg->stop_symbol;
            *done = 1;
        } else {
            symbols[symbol_pos++] = cfg->padding_symbol;
        }
    }

    // ESP_EARLY_LOGI(TAG, "[dio] data_pos=%u data_size=%u symbols_written=%u symbols_free=%u symbol_pos=%u done=%d",
    //     data_pos, data_size, symbols_written, symbols_free, symbol_pos, (int)*done);
    return symbol_pos;
}

esp_err_t tm1637_init(tm1637_config_t *config, tm1637_handle_t *ret_tm1637)
{
    ESP_RETURN_ON_FALSE(config && ret_tm1637, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(config->frequency_hz >= TM1637_MIN_FREQUENCY && config->frequency_hz <= TM1637_MAX_FREQUENCY,
                        ESP_ERR_INVALID_ARG, TAG, "frequency_hz must be between %d and %d", TM1637_MIN_FREQUENCY, TM1637_MAX_FREQUENCY);
    ESP_RETURN_ON_FALSE(config->digits_num <= TM1637_MAX_DIGITS, ESP_ERR_INVALID_ARG, TAG, "digits_num must be between 0 and %d", TM1637_MAX_DIGITS);

    esp_err_t ret;

    tm1637_state_t *tm1637 = heap_caps_calloc(1, sizeof(tm1637_state_t), TM1637_MEM_ALLOC_CAPS);
    ESP_GOTO_ON_FALSE(tm1637, ESP_ERR_NO_MEM, err, TAG, "no mem for tm1637 state");

    // CLK

    rmt_tx_channel_config_t clk_tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = TM1637_RMT_RESOLUTION_HZ,
        .mem_block_symbols = TM1637_RMT_MEM_BLOCK_SYMBOLS,
        .trans_queue_depth = TM1637_RMT_TRANS_QUEUE_DEPTH,
        .gpio_num = config->clk_gpio_num,
    };
    ESP_GOTO_ON_ERROR(rmt_new_tx_channel(&clk_tx_channel_cfg, &tm1637->clk_tx_channel), err, TAG, "failed to create clk tx channel");

    tm1637->clk_encoder_cfg.start_symbol = TM1637_CLK_START_SYMBOL(config->frequency_hz);
    tm1637->clk_encoder_cfg.bit0_symbol = TM1637_CLK_BIT_SYMBOL(config->frequency_hz);
    tm1637->clk_encoder_cfg.bit1_symbol = TM1637_CLK_BIT_SYMBOL(config->frequency_hz);
    tm1637->clk_encoder_cfg.ack1_symbol = TM1637_CLK_ACK1_SYMBOL(config->frequency_hz);
    tm1637->clk_encoder_cfg.ack2_symbol = TM1637_CLK_ACK2_SYMBOL(config->frequency_hz);
    tm1637->clk_encoder_cfg.padding_symbol = TM1637_CLK_PADDING_SYMBOL(config->frequency_hz);
    tm1637->clk_encoder_cfg.stop_symbol = TM1637_CLK_STOP_SYMBOL(config->frequency_hz);

    const rmt_simple_encoder_config_t clk_simple_encoder_cfg = {
        .callback = tm1637_encoder_callback,
        .arg = &tm1637->clk_encoder_cfg,
        .min_chunk_size = TM1637_RMT_SYMBOLS_PER_BYTE,
    };
    ESP_GOTO_ON_ERROR(rmt_new_simple_encoder(&clk_simple_encoder_cfg, &tm1637->clk_encoder), err, TAG, "failed to create clk encoder");

    ESP_GOTO_ON_ERROR(rmt_enable(tm1637->clk_tx_channel), err, TAG, "failed to enable clk tx channel");

    tm1637->clk_transmit_cfg.flags.eot_level = 1;

    // DIO

    rmt_tx_channel_config_t dio_tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = TM1637_RMT_RESOLUTION_HZ,
        .mem_block_symbols = TM1637_RMT_MEM_BLOCK_SYMBOLS,
        .trans_queue_depth = TM1637_RMT_TRANS_QUEUE_DEPTH,
        .gpio_num = config->dio_gpio_num,
    };
    ESP_GOTO_ON_ERROR(rmt_new_tx_channel(&dio_tx_channel_cfg, &tm1637->dio_tx_channel), err, TAG, "failed to create dio tx channel");

    tm1637->dio_encoder_cfg.start_symbol = TM1637_DIO_START_SYMBOL(config->frequency_hz);
    tm1637->dio_encoder_cfg.bit0_symbol = TM1637_DIO_BIT0_SYMBOL(config->frequency_hz);
    tm1637->dio_encoder_cfg.bit1_symbol = TM1637_DIO_BIT1_SYMBOL(config->frequency_hz);
    tm1637->dio_encoder_cfg.ack1_symbol = TM1637_DIO_ACK1_SYMBOL(config->frequency_hz);
    tm1637->dio_encoder_cfg.ack2_symbol = TM1637_DIO_ACK2_SYMBOL(config->frequency_hz);
    tm1637->dio_encoder_cfg.padding_symbol = TM1637_DIO_PADDING_SYMBOL(config->frequency_hz);
    tm1637->dio_encoder_cfg.stop_symbol = TM1637_DIO_STOP_SYMBOL(config->frequency_hz);

    const rmt_simple_encoder_config_t dio_simple_encoder_cfg = {
        .callback = tm1637_encoder_callback,
        .arg = &tm1637->dio_encoder_cfg,
        .min_chunk_size = TM1637_RMT_SYMBOLS_PER_BYTE,
    };
    ESP_GOTO_ON_ERROR(rmt_new_simple_encoder(&dio_simple_encoder_cfg, &tm1637->dio_encoder), err, TAG, "failed to create dio encoder");

    ESP_GOTO_ON_ERROR(rmt_enable(tm1637->dio_tx_channel), err, TAG, "failed to enable dio tx channel");

    tm1637->dio_transmit_cfg.flags.eot_level = 1;

    // Sync manager

    rmt_channel_handle_t tx_channels[TM1637_CHANNELS_NUM] = { tm1637->clk_tx_channel, tm1637->dio_tx_channel };

    rmt_sync_manager_config_t sync_manager_cfg = {
        .tx_channel_array = tx_channels,
        .array_size = sizeof(tx_channels) / sizeof(tx_channels[0]),
    };
    ESP_GOTO_ON_ERROR(rmt_new_sync_manager(&sync_manager_cfg, &tm1637->sync_manager), err, TAG, "failed to create sync manager");

    // digits_num and buffers

    tm1637->digits_num = config->digits_num == 0 ? TM1637_MAX_DIGITS : config->digits_num; // default to max if 0
    tm1637->cmd1_buf = TM1637_CMD1_BASE | TM1637_CMD1_OPERATION_WRITE | TM1637_CMD1_ADDR_AUTOINC;
    tm1637->cmd2_buf[0] = TM1637_CMD2_BASE;
    tm1637->cmd3_buf = TM1637_CMD3_BASE | TM1637_CMD3_BRIGHTNESS_7 | TM1637_CMD3_DISPLAY_ON;

    *ret_tm1637 = tm1637;

    // ESP_LOGI(TAG, "frequency requested=%"PRIu32" actual=%"PRIu32, config->frequency_hz,
    //     TM1637_RMT_RESOLUTION_HZ / TM1637_QUARTER_PERIOD_IN_TICKS(config->frequency_hz) / 4);

    return ESP_OK;

err:
    if (tm1637) {
        if (tm1637->sync_manager) {
            rmt_del_sync_manager(tm1637->sync_manager);
        }
        if (tm1637->dio_encoder) {
            rmt_del_encoder(tm1637->dio_encoder);
        }
        if (tm1637->dio_tx_channel) {
            rmt_del_channel(tm1637->dio_tx_channel);
        }
        if (tm1637->clk_encoder) {
            rmt_del_encoder(tm1637->clk_encoder);
        }
        if (tm1637->clk_tx_channel) {
            rmt_del_channel(tm1637->clk_tx_channel);
        }
        free(tm1637);
    }
    return ret;
}

esp_err_t tm1637_set_digit_raw(tm1637_handle_t tm1637, uint8_t idx, uint8_t value)
{
    ESP_RETURN_ON_FALSE(tm1637, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(idx < tm1637->digits_num, ESP_ERR_INVALID_ARG, TAG, "invalid argument: idx must be below %" PRIu8, tm1637->digits_num);
    tm1637->cmd2_buf[1 + idx] = value;
    return ESP_OK;
}

esp_err_t tm1637_set_digit_number(tm1637_handle_t tm1637, uint8_t idx, uint8_t value, bool set_dot)
{
    ESP_RETURN_ON_FALSE(tm1637, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(idx < tm1637->digits_num, ESP_ERR_INVALID_ARG, TAG, "invalid argument: idx must be below %" PRIu8, tm1637->digits_num);
    ESP_RETURN_ON_FALSE(value < 16, ESP_ERR_INVALID_ARG, TAG, "invalid argument: value must be below 16");
    tm1637->cmd2_buf[1 + idx] = tm1637_symbols[value] | (set_dot ? 0x80 : 0);
    return ESP_OK;
}

esp_err_t tm1637_set_brightness(tm1637_handle_t tm1637, uint8_t brightness)
{
    ESP_RETURN_ON_FALSE(tm1637, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_FALSE(brightness < 8, ESP_ERR_INVALID_ARG, TAG, "invalid argument: brightness must be below 8");
    tm1637->cmd3_buf = (tm1637->cmd3_buf & ~TM1637_CMD3_BRIGHTNESS_7) | brightness;
    return ESP_OK;
}

esp_err_t tm1637_set_display_enabled(tm1637_handle_t tm1637, bool enabled)
{
    ESP_RETURN_ON_FALSE(tm1637, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    if (enabled) {
        tm1637->cmd3_buf |= TM1637_CMD3_DISPLAY_ON;
    } else {
        tm1637->cmd3_buf &= ~TM1637_CMD3_DISPLAY_ON;
    }
    return ESP_OK;
}

static esp_err_t tm1637_transmit_bytes(tm1637_handle_t tm1637, const uint8_t *bytes, size_t bytes_size)
{
    ESP_RETURN_ON_FALSE(tm1637 && bytes && bytes_size, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(tm1637->clk_tx_channel, portMAX_DELAY), TAG, "failed to wait for clk channel");
    ESP_RETURN_ON_ERROR(rmt_tx_wait_all_done(tm1637->dio_tx_channel, portMAX_DELAY), TAG, "failed to wait for dio channel");
    ESP_RETURN_ON_ERROR(rmt_sync_reset(tm1637->sync_manager), TAG, "failed to reset sync manager");
    ESP_RETURN_ON_ERROR(rmt_transmit(tm1637->clk_tx_channel, tm1637->clk_encoder, (const void *)bytes, bytes_size, &tm1637->clk_transmit_cfg), TAG, "failed to transmit on clk channel");
    ESP_RETURN_ON_ERROR(rmt_transmit(tm1637->dio_tx_channel, tm1637->dio_encoder, (const void *)bytes, bytes_size, &tm1637->dio_transmit_cfg), TAG, "failed to transmit on dio channel");
    return ESP_OK;
}

esp_err_t tm1637_update(tm1637_handle_t tm1637)
{
    ESP_RETURN_ON_FALSE(tm1637, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    ESP_RETURN_ON_ERROR(tm1637_transmit_bytes(tm1637, &tm1637->cmd1_buf, sizeof(tm1637->cmd1_buf)), TAG, "failed to transmit cmd1");
    ESP_RETURN_ON_ERROR(tm1637_transmit_bytes(tm1637, tm1637->cmd2_buf, 1 + tm1637->digits_num), TAG, "failed to transmit cmd2");
    ESP_RETURN_ON_ERROR(tm1637_transmit_bytes(tm1637, &tm1637->cmd3_buf, sizeof(tm1637->cmd3_buf)), TAG, "failed to transmit cmd3");
    return ESP_OK;
}

esp_err_t tm1637_deinit(tm1637_handle_t tm1637)
{
    ESP_RETURN_ON_FALSE(tm1637, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    rmt_del_sync_manager(tm1637->sync_manager);
    rmt_del_encoder(tm1637->dio_encoder);
    rmt_del_channel(tm1637->dio_tx_channel);
    rmt_del_encoder(tm1637->clk_encoder);
    rmt_del_channel(tm1637->clk_tx_channel);
    free(tm1637);
    return ESP_OK;
}
