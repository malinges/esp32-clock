#include <esp_err.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_netif_sntp.h>

#include "sntp.h"

static const char *TAG = "sntp";

ESP_EVENT_DEFINE_BASE(SNTP_EVENT);

static void sntp_sync_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Clock synchronized"); // TODO log time
    if (esp_event_post(SNTP_EVENT, SNTP_EVENT_SYNCED, NULL, 0, portMAX_DELAY) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to post event SNTP_EVENT_SYNCED");
    }
}

esp_err_t sntp_init2(void)
{
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    config.start = false;                       // start the SNTP service explicitly (after connecting)
    config.sync_cb = sntp_sync_cb;
#ifdef CONFIG_LWIP_DHCP_GET_NTP_SRV
    config.server_from_dhcp = true;             // accept the NTP offers from DHCP server
    config.renew_servers_after_new_IP = true;   // let esp-netif update the configured SNTP server(s) after receiving the DHCP lease
    config.index_of_first_server = 1;           // updates from server num 1, leaving server 0 (from DHCP) intact
#endif

    return esp_netif_sntp_init(&config);
}

static void sntp_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "WiFi connected, starting SNTP...");
        ESP_ERROR_CHECK(esp_netif_sntp_start());
        if (esp_event_post(SNTP_EVENT, SNTP_EVENT_STARTED, NULL, 0, portMAX_DELAY) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to post event SNTP_EVENT_STARTED");
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected, stopping SNTP...");
        esp_netif_sntp_deinit();
        if (esp_event_post(SNTP_EVENT, SNTP_EVENT_STOPPED, NULL, 0, portMAX_DELAY) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to post event SNTP_EVENT_STOPPED");
        }
        ESP_ERROR_CHECK(sntp_init2());
    }
}

esp_err_t sntp_event_handler_register(void)
{
    esp_err_t err;

    err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, sntp_event_handler, NULL);
    if (err != ESP_OK) return err;
    
    err = esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, sntp_event_handler, NULL);
    if (err != ESP_OK) return err;

    return ESP_OK;
}
