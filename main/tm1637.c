#include <esp_check.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <driver/rmt_tx.h>

#include "tm1637.h"

#define TM1637_RESOLUTION_HZ                (2000000) // 2MHz resolution, 1 tick = 0.5us
#define TM1637_FREQUENCY                    (100000)
#define TM1637_CLK_GPIO_NUM                 (0)
#define TM1637_DIO_GPIO_NUM                 (1)

#define TM1637_TICKS_PER_PERIOD             (4)
#define TM1637_BASE_DURATION_IN_TICKS(res)  (1ULL * (res) / (TM1637_TICKS_PER_PERIOD * TM1637_FREQUENCY))
#define TM1637_CHANNELS_NUM                 (2)

#define RMT_MAX_TICKS                       ((1U << 15) - 1)
#define TM1637_MAX_SYMBOL_TICKS_MULTIPLIER  (3)
#define TM1637_MIN_FREQUENCY(res)           ((res) / (RMT_MAX_TICKS / TM1637_MAX_SYMBOL_TICKS_MULTIPLIER) * TM1637_TICKS_PER_PERIOD)
#define TM1637_MAX_FREQUENCY(res)           ((res) / TM1637_TICKS_PER_PERIOD)

static const char *TAG = "tm1637";

typedef struct {
    uint32_t resolution; /*!< Encoder resolution, in Hz */
} tm1637_encoder_config_t;

/*
 * CLK encoder
 */

#define TM1637_CLK_SYMBOLS_NUM              (12)        // (1 start + 8 * 1 bit + 1 ack1 + 1 ack2 + 1 stop)

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

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *copy_encoder;
    rmt_symbol_word_t tm1637_clk_symbols[TM1637_CLK_SYMBOLS_NUM];
} rmt_tm1637_clk_encoder_t;

static IRAM_ATTR size_t rmt_encode_tm1637_clk(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_tm1637_clk_encoder_t *tm1637_clk_encoder = __containerof(encoder, rmt_tm1637_clk_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    rmt_encoder_handle_t copy_encoder = tm1637_clk_encoder->copy_encoder;
    encoded_symbols += copy_encoder->encode(copy_encoder, channel, &tm1637_clk_encoder->tm1637_clk_symbols,
                                            sizeof(tm1637_clk_encoder->tm1637_clk_symbols), &session_state);
    if (session_state & RMT_ENCODING_COMPLETE) {
        state |= RMT_ENCODING_COMPLETE;
    }
    if (session_state & RMT_ENCODING_MEM_FULL) {
        state |= RMT_ENCODING_MEM_FULL;
    }
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_tm1637_clk_encoder(rmt_encoder_t *encoder)
{
    rmt_tm1637_clk_encoder_t *tm1637_clk_encoder = __containerof(encoder, rmt_tm1637_clk_encoder_t, base);
    rmt_del_encoder(tm1637_clk_encoder->copy_encoder);
    free(tm1637_clk_encoder);
    return ESP_OK;
}

static esp_err_t rmt_tm1637_clk_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_tm1637_clk_encoder_t *tm1637_clk_encoder = __containerof(encoder, rmt_tm1637_clk_encoder_t, base);
    rmt_encoder_reset(tm1637_clk_encoder->copy_encoder);
    return ESP_OK;
}

static esp_err_t rmt_new_tm1637_clk_encoder(const tm1637_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_tm1637_clk_encoder_t *tm1637_clk_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    tm1637_clk_encoder = rmt_alloc_encoder_mem(sizeof(rmt_tm1637_clk_encoder_t));
    ESP_GOTO_ON_FALSE(tm1637_clk_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for tm1637 clk encoder");
    tm1637_clk_encoder->base.encode = rmt_encode_tm1637_clk;
    tm1637_clk_encoder->base.del = rmt_del_tm1637_clk_encoder;
    tm1637_clk_encoder->base.reset = rmt_tm1637_clk_encoder_reset;

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &tm1637_clk_encoder->copy_encoder), err, TAG, "create copy encoder failed");

    tm1637_clk_encoder->tm1637_clk_symbols[0] = TM1637_CLK_START_SYMBOL(config->resolution);
    for (int i = 1; i < 9; i++) {
        tm1637_clk_encoder->tm1637_clk_symbols[i] = TM1637_CLK_BIT_SYMBOL(config->resolution);
    }
    tm1637_clk_encoder->tm1637_clk_symbols[9] = TM1637_CLK_ACK1_SYMBOL(config->resolution);
    tm1637_clk_encoder->tm1637_clk_symbols[10] = TM1637_CLK_ACK2_SYMBOL(config->resolution);
    tm1637_clk_encoder->tm1637_clk_symbols[11] = TM1637_CLK_STOP_SYMBOL(config->resolution);

    *ret_encoder = &tm1637_clk_encoder->base;
    return ESP_OK;
err:
    if (tm1637_clk_encoder) {
        if (tm1637_clk_encoder->copy_encoder) {
            rmt_del_encoder(tm1637_clk_encoder->copy_encoder);
        }
        free(tm1637_clk_encoder);
    }
    return ret;
}

/*
 * DIO encoder
 */

#define TM1637_DIO_SYMBOLS_LEN              (12)    // (1 start + 8 * 1 bit + 1 ack1 + 1 ack2 + 1 stop)

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

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *copy_encoder;
    rmt_encoder_t *bytes_encoder;
    rmt_symbol_word_t tm1637_dio_start_symbol;
    rmt_symbol_word_t tm1637_dio_ack1_symbol;
    rmt_symbol_word_t tm1637_dio_ack2_symbol;
    rmt_symbol_word_t tm1637_dio_stop_symbol;
    int state;
} rmt_tm1637_dio_encoder_t;

static IRAM_ATTR size_t rmt_encode_tm1637_dio(rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    rmt_tm1637_dio_encoder_t *tm1637_dio_encoder = __containerof(encoder, rmt_tm1637_dio_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    uint8_t *byte = (uint8_t*)primary_data;
    rmt_encoder_handle_t copy_encoder = tm1637_dio_encoder->copy_encoder;
    rmt_encoder_handle_t bytes_encoder = tm1637_dio_encoder->bytes_encoder;
    switch (tm1637_dio_encoder->state) {
    case 0: // send start symbol
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &tm1637_dio_encoder->tm1637_dio_start_symbol,
                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            tm1637_dio_encoder->state = 1; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    // fall-through
    case 1: // send byte
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, &byte, sizeof(*byte), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            tm1637_dio_encoder->state = 2; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    // fall-through
    case 2: // send ack1 symbol
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &tm1637_dio_encoder->tm1637_dio_ack1_symbol,
                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            tm1637_dio_encoder->state = 3; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    // fall-through
    case 3: // send ack2 symbol
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &tm1637_dio_encoder->tm1637_dio_ack2_symbol,
                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            tm1637_dio_encoder->state = 4; // we can only switch to next state when current encoder finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    // fall-through
    case 4: // send stop symbol
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &tm1637_dio_encoder->tm1637_dio_stop_symbol,
                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            tm1637_dio_encoder->state = RMT_ENCODING_RESET; // back to the initial encoding session
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space to put other encoding artifacts
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_tm1637_dio_encoder(rmt_encoder_t *encoder)
{
    rmt_tm1637_dio_encoder_t *tm1637_dio_encoder = __containerof(encoder, rmt_tm1637_dio_encoder_t, base);
    rmt_del_encoder(tm1637_dio_encoder->copy_encoder);
    rmt_del_encoder(tm1637_dio_encoder->bytes_encoder);
    free(tm1637_dio_encoder);
    return ESP_OK;
}

static esp_err_t rmt_tm1637_dio_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_tm1637_dio_encoder_t *tm1637_dio_encoder = __containerof(encoder, rmt_tm1637_dio_encoder_t, base);
    rmt_encoder_reset(tm1637_dio_encoder->copy_encoder);
    rmt_encoder_reset(tm1637_dio_encoder->bytes_encoder);
    tm1637_dio_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t rmt_new_tm1637_dio_encoder(const tm1637_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_tm1637_dio_encoder_t *tm1637_dio_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    tm1637_dio_encoder = rmt_alloc_encoder_mem(sizeof(rmt_tm1637_dio_encoder_t));
    ESP_GOTO_ON_FALSE(tm1637_dio_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for tm1637 dio encoder");
    tm1637_dio_encoder->base.encode = rmt_encode_tm1637_dio;
    tm1637_dio_encoder->base.del = rmt_del_tm1637_dio_encoder;
    tm1637_dio_encoder->base.reset = rmt_tm1637_dio_encoder_reset;

    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &tm1637_dio_encoder->copy_encoder), err, TAG, "create copy encoder failed");

    tm1637_dio_encoder->tm1637_dio_start_symbol = TM1637_DIO_START_SYMBOL(config->resolution);
    tm1637_dio_encoder->tm1637_dio_ack1_symbol = TM1637_DIO_ACK1_SYMBOL(config->resolution);
    tm1637_dio_encoder->tm1637_dio_ack2_symbol = TM1637_DIO_ACK2_SYMBOL(config->resolution);
    tm1637_dio_encoder->tm1637_dio_stop_symbol = TM1637_DIO_STOP_SYMBOL(config->resolution);

    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = TM1637_DIO_BIT0_SYMBOL(config->resolution),
        .bit1 = TM1637_DIO_BIT1_SYMBOL(config->resolution),
    };
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &tm1637_dio_encoder->bytes_encoder), err, TAG, "create bytes encoder failed");

    *ret_encoder = &tm1637_dio_encoder->base;
    return ESP_OK;
err:
    if (tm1637_dio_encoder) {
        if (tm1637_dio_encoder->copy_encoder) {
            rmt_del_encoder(tm1637_dio_encoder->copy_encoder);
        }
        if (tm1637_dio_encoder->bytes_encoder) {
            rmt_del_encoder(tm1637_dio_encoder->bytes_encoder);
        }
        free(tm1637_dio_encoder);
    }
    return ret;
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
    tm1637_encoder_config_t tm1637_clk_encoder_cfg = {
        .resolution = TM1637_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t tm1637_clk_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_tm1637_clk_encoder(&tm1637_clk_encoder_cfg, &tm1637_clk_encoder));

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
    tm1637_encoder_config_t tm1637_dio_encoder_cfg = {
        .resolution = TM1637_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t tm1637_dio_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_tm1637_dio_encoder(&tm1637_dio_encoder_cfg, &tm1637_dio_encoder));

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

    while (1) {
        uint8_t data = 0xff;

        ESP_ERROR_CHECK(rmt_sync_reset(synchro));

        ESP_LOGW(TAG, "min frequency with res=%d: %d", TM1637_RESOLUTION_HZ, TM1637_MIN_FREQUENCY(TM1637_RESOLUTION_HZ));
        ESP_LOGW(TAG, "max frequency with res=%d: %d", TM1637_RESOLUTION_HZ, TM1637_MAX_FREQUENCY(TM1637_RESOLUTION_HZ));

        for (int i = 0; i < TM1637_CHANNELS_NUM; i++) {
            ESP_LOGI(TAG, "transmitting %d bytes on channel %d", sizeof(data), i);
            ESP_ERROR_CHECK(rmt_transmit(tx_channels[i], tx_encoders[i], &data, sizeof(data), &transmit_configs[i]));
        }

        for (int i = 0; i < TM1637_CHANNELS_NUM; i++) {
            ESP_LOGI(TAG, "awaiting end of transmission on channel %d", i);
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(tx_channels[i], portMAX_DELAY));
        }

        ESP_LOGI(TAG, "done! waiting for another round... :)");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
