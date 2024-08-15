#pragma once

#include <esp_err.h>

ESP_EVENT_DECLARE_BASE(SNTP_EVENT);

typedef enum {
  SNTP_EVENT_STARTED,
  SNTP_EVENT_SYNCED,
  SNTP_EVENT_STOPPED,
} sntp_event_t;

esp_err_t sntp_init2(void);
esp_err_t sntp_event_handler_register(void);
