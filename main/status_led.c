#include <stdint.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include <esp_check.h>
#include <esp_err.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_clk_tree.h>
#include <driver/ledc.h>

#include <wifi_provisioning/manager.h>

#include "status_led.h"
#include "sntp.h"
#include "tz.h"

#define STATUS_LED_TIMER                        LEDC_TIMER_0
#define STATUS_LED_SPEED_MODE                   LEDC_LOW_SPEED_MODE
#define STATUS_LED_OUTPUT_GPIO                  (8)
#define STATUS_LED_CHANNEL                      LEDC_CHANNEL_0
#define STATUS_LED_INTR_TYPE                    LEDC_INTR_DISABLE
#define STATUS_LED_DUTY_RES                     LEDC_TIMER_14_BIT
#define STATUS_LED_CLK_CFG                      LEDC_AUTO_CLK
#define STATUS_LED_IDLE_LEVEL                   (1)
#define STATUS_LED_FREQUENCY_SLOW               (2UL)
#define STATUS_LED_FREQUENCY_FAST               (8UL)

#define STATUS_LED_DUTY_SYM                     (0x8UL << (STATUS_LED_DUTY_RES - 4))
#if STATUS_LED_IDLE_LEVEL
#define STATUS_LED_DUTY_ASYM                    (0xfUL << (STATUS_LED_DUTY_RES - 4))
#else
#define STATUS_LED_DUTY_ASYM                    (0x1UL << (STATUS_LED_DUTY_RES - 4))
#endif

#define STATUS_LED_CHECK(a, str, ret_val)       ESP_RETURN_ON_FALSE(a, ret_val, STATUS_LED_TAG, "%s", str)
#define STATUS_LED_ARG_CHECK(a, param)          ESP_RETURN_ON_FALSE(a, ESP_ERR_INVALID_ARG, STATUS_LED_TAG, param " argument is invalid")

#define STATUS_LED_IS_FLAG_SET(flags, flag)     (((flags) & (1U << flag)) != 0)

#define STATUS_LED_QUEUE_SIZE                   (4)

static __attribute__((unused)) const char *STATUS_LED_TAG = "status_led";
static __attribute__((unused)) const char *STATUS_LED_NOT_INIT = "status_led is not initialized";
static __attribute__((unused)) const char *STATUS_LED_INIT = "status_led is already initialized";

typedef enum {
    STATUS_LED_MODE_STOP = 0,
    STATUS_LED_MODE_FAST,
    STATUS_LED_MODE_SLOW_SYM,
    STATUS_LED_MODE_SLOW_ASYM,
    STATUS_LED_MODE_MAX,
} status_led_mode_t;

typedef struct {
    uint32_t freq;
    uint32_t duty;
} status_led_mode_params_t;

static const status_led_mode_params_t status_led_mode_params[STATUS_LED_MODE_MAX] = {
    [STATUS_LED_MODE_STOP]      = { .freq = 0,                          .duty = 0 },
    [STATUS_LED_MODE_FAST]      = { .freq = STATUS_LED_FREQUENCY_FAST,  .duty = STATUS_LED_DUTY_SYM },
    [STATUS_LED_MODE_SLOW_SYM]  = { .freq = STATUS_LED_FREQUENCY_SLOW,  .duty = STATUS_LED_DUTY_SYM },
    [STATUS_LED_MODE_SLOW_ASYM] = { .freq = STATUS_LED_FREQUENCY_SLOW,  .duty = STATUS_LED_DUTY_ASYM },
};

typedef uint8_t status_led_flags_t; // Increase size if the number of flags ever goes over 8

typedef struct {
    status_led_flags_t flags;
    status_led_mode_t mode;
} status_led_status_t;

static status_led_status_t status_led_status;

typedef enum {
    STATUS_LED_FLAG_IS_PROVISIONING_WIFI = 0,
    STATUS_LED_FLAG_IS_WIFI_CONNECTED,
    STATUS_LED_FLAG_IS_CLOCK_SYNCED,
    STATUS_LED_FLAG_IS_TIMEZONE_SET,
    STATUS_LED_FLAG_MAX,
} status_led_flag_t;

typedef struct {
    status_led_flag_t flag;
    bool set;
} status_led_msg_t;

static QueueHandle_t status_led_queue;

static status_led_mode_t status_led_flags_to_mode(status_led_flags_t flags)
{
    if (STATUS_LED_IS_FLAG_SET(flags, STATUS_LED_FLAG_IS_PROVISIONING_WIFI)) {
        return STATUS_LED_MODE_FAST;
    } else if (!STATUS_LED_IS_FLAG_SET(flags, STATUS_LED_FLAG_IS_WIFI_CONNECTED)) {
        return STATUS_LED_MODE_SLOW_SYM;
    } else if (!STATUS_LED_IS_FLAG_SET(flags, STATUS_LED_FLAG_IS_CLOCK_SYNCED) || !STATUS_LED_IS_FLAG_SET(flags, STATUS_LED_FLAG_IS_TIMEZONE_SET)) {
        return STATUS_LED_MODE_SLOW_ASYM;
    } else {
        return STATUS_LED_MODE_STOP;
    }
}

static esp_err_t status_led_update(void)
{
    status_led_mode_t current_mode = status_led_status.mode;
    status_led_mode_t mode = status_led_flags_to_mode(status_led_status.flags);

    if (current_mode == STATUS_LED_MODE_STOP && mode != STATUS_LED_MODE_STOP) {
        // Prepare and then apply the LEDC PWM timer configuration
        ledc_timer_config_t ledc_timer = {
            .speed_mode         = STATUS_LED_SPEED_MODE,
            .duty_resolution    = STATUS_LED_DUTY_RES,
            .timer_num          = STATUS_LED_TIMER,
            .freq_hz            = status_led_mode_params[mode].freq,
            .clk_cfg            = STATUS_LED_CLK_CFG,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        // Prepare and then apply the LEDC PWM channel configuration
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = STATUS_LED_SPEED_MODE,
            .channel        = STATUS_LED_CHANNEL,
            .timer_sel      = STATUS_LED_TIMER,
            .intr_type      = STATUS_LED_INTR_TYPE,
            .gpio_num       = STATUS_LED_OUTPUT_GPIO,
            .duty           = status_led_mode_params[mode].duty,
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    } else if (current_mode != STATUS_LED_MODE_STOP && mode == STATUS_LED_MODE_STOP) {
        ESP_ERROR_CHECK(ledc_stop(STATUS_LED_SPEED_MODE, STATUS_LED_CHANNEL, STATUS_LED_IDLE_LEVEL));

        ESP_ERROR_CHECK(ledc_timer_pause(STATUS_LED_SPEED_MODE, STATUS_LED_TIMER));

        ledc_timer_config_t ledc_timer = {
            .speed_mode     = STATUS_LED_SPEED_MODE,
            .timer_num      = STATUS_LED_TIMER,
            .deconfigure    = true
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    } else if (current_mode != mode) {
        uint32_t current_freq = status_led_mode_params[current_mode].freq;
        uint32_t freq = status_led_mode_params[mode].freq;
        if (current_freq != freq) {
            ESP_ERROR_CHECK(ledc_set_freq(STATUS_LED_SPEED_MODE, STATUS_LED_TIMER, freq));
        }

        uint32_t current_duty = status_led_mode_params[current_mode].duty;
        uint32_t duty = status_led_mode_params[mode].duty;
        if (current_duty != duty) {
            ESP_ERROR_CHECK(ledc_set_duty(STATUS_LED_SPEED_MODE, STATUS_LED_CHANNEL, duty));
            ESP_ERROR_CHECK(ledc_update_duty(STATUS_LED_SPEED_MODE, STATUS_LED_CHANNEL));
        }
    }

    status_led_status.mode = mode;

    return ESP_OK;
}

static esp_err_t status_led_set_flag(status_led_flag_t flag, bool set)
{
    STATUS_LED_CHECK(status_led_queue != NULL, STATUS_LED_NOT_INIT, ESP_ERR_INVALID_STATE);
    STATUS_LED_ARG_CHECK(flag < STATUS_LED_FLAG_MAX, "flag");

    status_led_msg_t msg = {
        .flag = flag,
        .set = set,
    };

    if (xQueueSend(status_led_queue, &msg, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

static void status_led_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START || event_id == WIFI_EVENT_STA_DISCONNECTED) {
            status_led_set_flag(STATUS_LED_FLAG_IS_WIFI_CONNECTED, false);
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            status_led_set_flag(STATUS_LED_FLAG_IS_WIFI_CONNECTED, true);
        }
    } else if (event_base == WIFI_PROV_EVENT) {
        if (event_id == WIFI_PROV_START) {
            status_led_set_flag(STATUS_LED_FLAG_IS_PROVISIONING_WIFI, true);
        } else if (event_id == WIFI_PROV_CRED_SUCCESS) {
            status_led_set_flag(STATUS_LED_FLAG_IS_PROVISIONING_WIFI, false);
        }
    } else if (event_base == SNTP_EVENT) {
        if (event_id == SNTP_EVENT_STARTED) {
            status_led_set_flag(STATUS_LED_FLAG_IS_CLOCK_SYNCED, false);
        } else if (event_id == SNTP_EVENT_SYNCED) {
            status_led_set_flag(STATUS_LED_FLAG_IS_CLOCK_SYNCED, true);
        }
    } else if (event_base == TZ_EVENT) {
        if (event_id == TZ_EVENT_TIMEZONE_SET) {
            status_led_set_flag(STATUS_LED_FLAG_IS_TIMEZONE_SET, true);
        }
    }
}

static esp_err_t status_led_event_handler_register(void)
{
    esp_err_t err;

    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &status_led_event_handler, NULL);
    if (err != ESP_OK) return err;

    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &status_led_event_handler, NULL);
    if (err != ESP_OK) return err;

    err = esp_event_handler_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &status_led_event_handler, NULL);
    if (err != ESP_OK) return err;

    err = esp_event_handler_register(SNTP_EVENT, ESP_EVENT_ANY_ID, &status_led_event_handler, NULL);
    if (err != ESP_OK) return err;

    err = esp_event_handler_register(TZ_EVENT, TZ_EVENT_TIMEZONE_SET, &status_led_event_handler, NULL);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

static void status_led_task(void *pvParameters)
{
    ESP_ERROR_CHECK(status_led_update());

    status_led_msg_t msg;

    while (1) {
        if (xQueueReceive(status_led_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.set) {
                status_led_status.flags |= ((status_led_flags_t)1 << msg.flag);
            } else {
                status_led_status.flags &= ~((status_led_flags_t)1 << msg.flag);
            }
            ESP_ERROR_CHECK(status_led_update());
        }
    }
}

esp_err_t status_led_init(void)
{
    STATUS_LED_CHECK(status_led_queue == NULL, STATUS_LED_INIT, ESP_ERR_INVALID_STATE);

    status_led_queue = xQueueCreate(STATUS_LED_QUEUE_SIZE, sizeof(status_led_msg_t));
    if (status_led_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = status_led_event_handler_register();
    if (err != ESP_OK) {
        vQueueDelete(status_led_queue);
        status_led_queue = NULL;
        return err;
    }

    if (xTaskCreate(status_led_task, "status_led_task", 1024, NULL, 10, NULL) != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}
