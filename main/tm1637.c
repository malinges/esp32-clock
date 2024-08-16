#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <esp_check.h>
#include <esp_log.h>

#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>

#include "tm1637.h"

#define TM1637_RESOLUTION_HZ                (2000000) // 2MHz resolution, 1 tick = 0.5us
#define TM1637_FREQUENCY                    (100000)
#define TM1637_CLK_GPIO_NUM                 (0)
#define TM1637_DIO_GPIO_NUM                 (1)

#define TM1637_VALUES_PER_PERIOD            (4)
#define TM1637_BASE_DURATION_IN_TICKS(res)  (1ULL * (res) / (TM1637_VALUES_PER_PERIOD * TM1637_FREQUENCY))
#define TM1637_CHANNELS_NUM                 (2)

#define RMT_MAX_TICKS                       ((1U << 15) - 1)
#define TM1637_MAX_SYMBOL_TICKS_MULTIPLIER  (2)
#define CEIL(x, y)                          (((x) + (y) - 1) / (y))
#define TM1637_MIN_FREQUENCY(res)           CEIL(CEIL((res), RMT_MAX_TICKS / TM1637_MAX_SYMBOL_TICKS_MULTIPLIER), TM1637_VALUES_PER_PERIOD)
#define TM1637_MAX_FREQUENCY(res)           ((res) / TM1637_VALUES_PER_PERIOD)

#define TM1637_RMT_SYMBOLS_PER_BYTE         (12)    // (1 start + 8 * 1 bit + 1 ack1 + 1 ack2 + 1 stop)

static const char *TAG = "tm1637";

/*
 * CLK encoder
 */

#define TM1637_CLK_START_SYMBOL(res)    ((rmt_symbol_word_t) {  \
    .level0 = 0,                                                \
    .duration0 = 1 * TM1637_BASE_DURATION_IN_TICKS(res),        \
    .level1 = 1,                                                \
    .duration1 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
})

#define TM1637_CLK_BIT_SYMBOL(res)      ((rmt_symbol_word_t) {  \
    .level0 = 0,                                                \
    .duration0 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
    .level1 = 1,                                                \
    .duration1 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
})

#define TM1637_CLK_ACK1_SYMBOL(res)     ((rmt_symbol_word_t) {  \
    .level0 = 1,                                                \
    .duration0 = 1 * TM1637_BASE_DURATION_IN_TICKS(res),        \
    .level1 = 0,                                                \
    .duration1 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
})

#define TM1637_CLK_ACK2_SYMBOL(res)     ((rmt_symbol_word_t) {  \
    .level0 = 1,                                                \
    .duration0 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
    .level1 = 0,                                                \
    .duration1 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
})

#define TM1637_CLK_STOP_SYMBOL(res)     ((rmt_symbol_word_t) {  \
    .level0 = 1,                                                \
    .duration0 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
    .level1 = 0,                                                \
    .duration1 = 1 * TM1637_BASE_DURATION_IN_TICKS(res),        \
})

static IRAM_ATTR size_t tm1637_clk_encoder_callback(const void *data, size_t data_size,
                               size_t symbols_written, size_t symbols_free,
                               rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    size_t data_pos = symbols_written / TM1637_RMT_SYMBOLS_PER_BYTE;
    size_t symbol_pos = 0;

    while (symbols_free >= TM1637_RMT_SYMBOLS_PER_BYTE && data_pos < data_size) {
        symbols[symbol_pos++] = TM1637_CLK_START_SYMBOL(TM1637_RESOLUTION_HZ);
        for (uint8_t bitmask = 0x01; bitmask != 0; bitmask <<= 1) {
            symbols[symbol_pos++] = TM1637_CLK_BIT_SYMBOL(TM1637_RESOLUTION_HZ);
        }
        symbols[symbol_pos++] = TM1637_CLK_ACK1_SYMBOL(TM1637_RESOLUTION_HZ);
        symbols[symbol_pos++] = TM1637_CLK_ACK2_SYMBOL(TM1637_RESOLUTION_HZ);
        symbols[symbol_pos++] = TM1637_CLK_STOP_SYMBOL(TM1637_RESOLUTION_HZ);
        data_pos++;
        symbols_free -= TM1637_RMT_SYMBOLS_PER_BYTE;
    }

    if (data_pos >= data_size) {
        *done = 1;
    }

    return symbol_pos;
}

/*
 * DIO encoder
 */

#define TM1637_DIO_START_SYMBOL(res)    ((rmt_symbol_word_t) {  \
    .level0 = 1,                                                \
    .duration0 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
    .level1 = 0,                                                \
    .duration1 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
})

#define TM1637_DIO_BIT0_SYMBOL(res)    ((rmt_symbol_word_t) {   \
    .level0 = 0,                                                \
    .duration0 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
    .level1 = 0,                                                \
    .duration1 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
})

#define TM1637_DIO_BIT1_SYMBOL(res)    ((rmt_symbol_word_t) {   \
    .level0 = 1,                                                \
    .duration0 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
    .level1 = 1,                                                \
    .duration1 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
})

#define TM1637_DIO_ACK1_SYMBOL(res)     ((rmt_symbol_word_t) {  \
    .level0 = 0,                                                \
    .duration0 = 1 * TM1637_BASE_DURATION_IN_TICKS(res),        \
    .level1 = 0,                                                \
    .duration1 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
})

#define TM1637_DIO_ACK2_SYMBOL(res)     ((rmt_symbol_word_t) {  \
    .level0 = 0,                                                \
    .duration0 = 1 * TM1637_BASE_DURATION_IN_TICKS(res),        \
    .level1 = 0,                                                \
    .duration1 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
})

#define TM1637_DIO_STOP_SYMBOL(res)     ((rmt_symbol_word_t) {  \
    .level0 = 0,                                                \
    .duration0 = 1 * TM1637_BASE_DURATION_IN_TICKS(res),        \
    .level1 = 1,                                                \
    .duration1 = 2 * TM1637_BASE_DURATION_IN_TICKS(res),        \
})

static IRAM_ATTR size_t tm1637_dio_encoder_callback(const void *data, size_t data_size,
                               size_t symbols_written, size_t symbols_free,
                               rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    size_t data_pos = symbols_written / TM1637_RMT_SYMBOLS_PER_BYTE;
    size_t symbol_pos = 0;
    uint8_t *data_bytes = (uint8_t*)data;

    while (symbols_free >= TM1637_RMT_SYMBOLS_PER_BYTE && data_pos < data_size) {
        symbols[symbol_pos++] = TM1637_DIO_START_SYMBOL(TM1637_RESOLUTION_HZ);
        for (uint8_t bitmask = 0x01; bitmask != 0; bitmask <<= 1) {
            if (data_bytes[data_pos] & bitmask) {
                symbols[symbol_pos++] = TM1637_DIO_BIT1_SYMBOL(TM1637_RESOLUTION_HZ);
            } else {
                symbols[symbol_pos++] = TM1637_DIO_BIT0_SYMBOL(TM1637_RESOLUTION_HZ);
            }
        }
        symbols[symbol_pos++] = TM1637_DIO_ACK1_SYMBOL(TM1637_RESOLUTION_HZ);
        symbols[symbol_pos++] = TM1637_DIO_ACK2_SYMBOL(TM1637_RESOLUTION_HZ);
        symbols[symbol_pos++] = TM1637_DIO_STOP_SYMBOL(TM1637_RESOLUTION_HZ);
        data_pos++;
        symbols_free -= TM1637_RMT_SYMBOLS_PER_BYTE;
    }

    if (data_pos >= data_size) {
        *done = 1;
    }

    return symbol_pos;
}

void __attribute__((unused)) rmt_test(void)
{
    //
    // CLK TX channel
    //

    ESP_LOGI(TAG, "create tm1637 clk RMT TX channel");
    rmt_tx_channel_config_t clk_tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = TM1637_RESOLUTION_HZ,
        .mem_block_symbols = 48, // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num = TM1637_CLK_GPIO_NUM,
    };
    rmt_channel_handle_t tm1637_clk_tx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&clk_tx_channel_cfg, &tm1637_clk_tx_channel));

    ESP_LOGI(TAG, "install tm1637 clk encoder");
    rmt_encoder_handle_t tm1637_clk_encoder = NULL;
    const rmt_simple_encoder_config_t tm1637_clk_simple_encoder_cfg = {
        .callback = tm1637_clk_encoder_callback,
        .min_chunk_size = TM1637_RMT_SYMBOLS_PER_BYTE,
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&tm1637_clk_simple_encoder_cfg, &tm1637_clk_encoder));

    ESP_LOGI(TAG, "enable tm1637 clk RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(tm1637_clk_tx_channel));

    rmt_transmit_config_t tm1637_clk_transmit_config = {
        .loop_count = 0, // no loop
        .flags = {
            .eot_level = 0,
        },
    };

    // DIO TX channel

    ESP_LOGI(TAG, "create tm1637 dio RMT TX channel");
    rmt_tx_channel_config_t dio_tx_channel_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = TM1637_RESOLUTION_HZ,
        .mem_block_symbols = 48, // amount of RMT symbols that the channel can store at a time
        .trans_queue_depth = 4,  // number of transactions that allowed to pending in the background, this example won't queue multiple transactions, so queue depth > 1 is sufficient
        .gpio_num = TM1637_DIO_GPIO_NUM,
    };
    rmt_channel_handle_t tm1637_dio_tx_channel = NULL;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&dio_tx_channel_cfg, &tm1637_dio_tx_channel));

    ESP_LOGI(TAG, "install tm1637 dio encoder");
    rmt_encoder_handle_t tm1637_dio_encoder = NULL;
    const rmt_simple_encoder_config_t tm1637_dio_simple_encoder_cfg = {
        .callback = tm1637_dio_encoder_callback,
        .min_chunk_size = TM1637_RMT_SYMBOLS_PER_BYTE,
    };
    ESP_ERROR_CHECK(rmt_new_simple_encoder(&tm1637_dio_simple_encoder_cfg, &tm1637_dio_encoder));

    ESP_LOGI(TAG, "enable tm1637 dio RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(tm1637_dio_tx_channel));

    rmt_transmit_config_t tm1637_dio_transmit_config = {
        .loop_count = 0, // no loop
        .flags = {
            .eot_level = 1,
        },
    };

    // Sync manager

    rmt_channel_handle_t tx_channels[TM1637_CHANNELS_NUM] = { tm1637_clk_tx_channel, tm1637_dio_tx_channel };
    rmt_encoder_handle_t tx_encoders[TM1637_CHANNELS_NUM] = { tm1637_clk_encoder, tm1637_dio_encoder };
    rmt_transmit_config_t transmit_configs[TM1637_CHANNELS_NUM] = { tm1637_clk_transmit_config, tm1637_dio_transmit_config };

    rmt_sync_manager_handle_t synchro = NULL;
    rmt_sync_manager_config_t synchro_config = {
        .tx_channel_array = tx_channels,
        .array_size = sizeof(tx_channels) / sizeof(tx_channels[0]),
    };
    ESP_ERROR_CHECK(rmt_new_sync_manager(&synchro_config, &synchro));

    uint8_t test_data[] = {
        0, 1, 2, 3, 4, 5, 6, 7,
        8, 9, 10, 11, 12, 13, 14, 15,
    };

    while (1) {

        ESP_ERROR_CHECK(rmt_sync_reset(synchro));

        ESP_LOGW(TAG, "min frequency with res=%d: %d", TM1637_RESOLUTION_HZ, TM1637_MIN_FREQUENCY(TM1637_RESOLUTION_HZ));
        ESP_LOGW(TAG, "max frequency with res=%d: %d", TM1637_RESOLUTION_HZ, TM1637_MAX_FREQUENCY(TM1637_RESOLUTION_HZ));

        for (int i = 0; i < TM1637_CHANNELS_NUM; i++) {
            ESP_LOGI(TAG, "transmitting %d bytes on channel %d", sizeof(test_data), i);
            ESP_ERROR_CHECK(rmt_transmit(tx_channels[i], tx_encoders[i], test_data, sizeof(test_data), &transmit_configs[i]));
        }

        for (int i = 0; i < TM1637_CHANNELS_NUM; i++) {
            ESP_LOGI(TAG, "awaiting end of transmission on channel %d", i);
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_channels[i], portMAX_DELAY));
        }

        ESP_LOGI(TAG, "done! waiting for another round... :)");
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
