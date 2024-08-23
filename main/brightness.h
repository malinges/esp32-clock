#include <esp_err.h>
#include <esp_adc/adc_oneshot.h>

esp_err_t brightness_init(adc_oneshot_unit_handle_t *ret_adc_unit);
esp_err_t brightness_read(adc_oneshot_unit_handle_t adc_unit, int *ret_brightness, int *ret_brightness_raw);
