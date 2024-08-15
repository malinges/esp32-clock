#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <esp_log.h>
#include <esp_err.h>
#include <esp_event.h>

#include "tz.h"
#include "tzdata.h"

static const char *TAG = "tz";

ESP_EVENT_DEFINE_BASE(TZ_EVENT);

esp_err_t tz_init(const char *tzname) {
    for (size_t i = 0; i < timezones_len; i++) {
        timezone_t timezone = timezones[i];
        if (strcmp(timezone.name, tzname) == 0) {
            if (setenv("TZ", timezone.tz, 1) == 0) {
                tzset();
                ESP_LOGI(TAG, "Timezone set to %s", tzname);
                if (esp_event_post(TZ_EVENT, TZ_EVENT_TIMEZONE_SET, NULL, 0, portMAX_DELAY) != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to post event TZ_EVENT_TIMEZONE_SET");
                }
                return ESP_OK;
            }
        }
    }

    ESP_LOGE(TAG, "Failed to set timezone to %s", tzname);
    return ESP_FAIL;
}
