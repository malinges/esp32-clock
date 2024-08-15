#pragma once

#include <esp_err.h>
#include <esp_event.h>

ESP_EVENT_DECLARE_BASE(TZ_EVENT);

typedef enum {
  TZ_EVENT_TIMEZONE_SET,
} tz_event_t;

esp_err_t tz_init(const char *tzname);
