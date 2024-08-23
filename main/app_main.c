#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_log.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <nvs_flash.h>

#include "status_led.h"
#include "wifi_prov.h"
#include "sntp.h"
#include "tz.h"
#include "tm1637.h"

static const char *TAG = "app";

typedef enum {
    APP_EVENT_GROUP_WIFI_CONNECTED_BIT  = BIT0,
    APP_EVENT_GROUP_SNTP_SYNCED_BIT     = BIT1,
    APP_EVENT_GROUP_TIMEZONE_SET_BIT    = BIT2,
} app_event_group_bit_t;

static EventGroupHandle_t app_event_group;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            ESP_LOGI(TAG, "WiFi disconnected, reconnecting...");
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT) {
        if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "WiFi connected:");
            ESP_LOGI(TAG, " - IP address: " IPSTR, IP2STR(&event->ip_info.ip));
            ESP_LOGI(TAG, " - netmask:    " IPSTR, IP2STR(&event->ip_info.netmask));
            ESP_LOGI(TAG, " - gateway:    " IPSTR, IP2STR(&event->ip_info.gw));
            xEventGroupSetBits(app_event_group, APP_EVENT_GROUP_WIFI_CONNECTED_BIT);
        }
    } else if (event_base == SNTP_EVENT) {
        if (event_id == SNTP_EVENT_SYNCED) {
            xEventGroupSetBits(app_event_group, APP_EVENT_GROUP_SNTP_SYNCED_BIT);
        }
    } else if (event_base == TZ_EVENT) {
        if (event_id == TZ_EVENT_TIMEZONE_SET) {
            xEventGroupSetBits(app_event_group, APP_EVENT_GROUP_TIMEZONE_SET_BIT);
        }
    }
}

void app_main(void)
{
    /* Initialize NVS partition */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS partition was truncated
         * and needs to be erased */
        ESP_ERROR_CHECK(nvs_flash_erase());

        /* Retry nvs_flash_init */
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* Initialize TCP/IP */
    ESP_ERROR_CHECK(esp_netif_init());
    
    app_event_group = xEventGroupCreate(); // TODO error check

    /* Initialize the event loop */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(status_led_init());
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(SNTP_EVENT, SNTP_EVENT_SYNCED, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(TZ_EVENT, TZ_EVENT_TIMEZONE_SET, &event_handler, NULL));
    ESP_ERROR_CHECK(wifi_prov_event_handler_register());

    ESP_ERROR_CHECK(sntp_init2());
    ESP_ERROR_CHECK(sntp_event_handler_register());

    /* Initialize Wi-Fi including netif with default config */
    esp_netif_create_default_wifi_sta();

#ifdef CONFIG_EXAMPLE_PROV_TRANSPORT_SOFTAP
    esp_netif_create_default_wifi_ap();
#endif

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(wifi_prov_init());

    tz_init("America/Montreal"); // TODO lookup dynamically

    xEventGroupWaitBits(app_event_group,
                        APP_EVENT_GROUP_WIFI_CONNECTED_BIT | APP_EVENT_GROUP_SNTP_SYNCED_BIT | APP_EVENT_GROUP_TIMEZONE_SET_BIT,
                        true,
                        true,
                        portMAX_DELAY);

    tm1637_config_t tm1637_cfg = {
        .clk_gpio_num = 0,
        .dio_gpio_num = 2,
        .frequency_hz = 100000,
        .digits_num = 4,
    };
    tm1637_handle_t tm1637 = NULL;
    ESP_ERROR_CHECK(tm1637_init(&tm1637_cfg, &tm1637));
    ESP_ERROR_CHECK(tm1637_set_brightness(tm1637, 0));

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        time_t t = time(NULL);
        if (t == (time_t)-1) {
            ESP_LOGE(TAG, "time() failed!");
        } else {
            struct tm tm;
            if (localtime_r(&t, &tm) == NULL) {
                ESP_LOGE(TAG, "localtime_r() failed!");
            } else {
                tm1637_set_digit_number(tm1637, 0, tm.tm_hour / 10, false);
                tm1637_set_digit_number(tm1637, 1, tm.tm_hour % 10, tm.tm_sec % 2 == 0);
                tm1637_set_digit_number(tm1637, 2, tm.tm_min / 10, false);
                tm1637_set_digit_number(tm1637, 3, tm.tm_min % 10, false);
                tm1637_update(tm1637);
            }

            char buf[26];
            if (ctime_r(&t, buf) == NULL) {
                ESP_LOGE(TAG, "ctime_r() failed!");
            } else {
                char *newline = strchr(buf, '\n');
                if (newline != NULL) {
                    *newline = '\0';
                }
                ESP_LOGI(TAG, "%s", buf);
            }
        }

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}
