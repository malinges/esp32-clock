#include <esp_adc/adc_oneshot.h>
#include <esp_check.h>
#include <esp_err.h>

#include "brightness.h"

static const char *TAG = "brightness";

#define BRIGHTNESS_ADC_UNIT         ADC_UNIT_1
#define BRIGHTNESS_ADC_ULP_MODE     ADC_ULP_MODE_DISABLE
#define BRIGHTNESS_ADC_BITWIDTH     ADC_BITWIDTH_12
#define BRIGHTNESS_ADC_ATTEN        ADC_ATTEN_DB_12
#define BRIGHTNESS_ADC_CHANNEL      ADC_CHANNEL_0

#define BRIGHTNESS_VALUE_BITWIDTH   (3)

esp_err_t brightness_init(adc_oneshot_unit_handle_t *ret_adc_unit)
{
    esp_err_t ret;

    adc_oneshot_unit_handle_t adc = NULL;
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BRIGHTNESS_ADC_UNIT,
        .ulp_mode = BRIGHTNESS_ADC_ULP_MODE,
    };
    ESP_GOTO_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &adc), err, TAG, "failed to init ADC unit");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = BRIGHTNESS_ADC_BITWIDTH,
        .atten = BRIGHTNESS_ADC_ATTEN,
    };
    ESP_GOTO_ON_ERROR(adc_oneshot_config_channel(adc, BRIGHTNESS_ADC_CHANNEL, &chan_cfg), err, TAG, "failed to init ADC channel");

    *ret_adc_unit = adc;

    return ESP_OK;

err:
    if (adc) {
        adc_oneshot_del_unit(adc);
    }
    return ret;
}

esp_err_t brightness_read(adc_oneshot_unit_handle_t adc_unit, int *ret_brightness, int *ret_brightness_raw)
{
    const int divisor = 1 << (BRIGHTNESS_ADC_BITWIDTH - BRIGHTNESS_VALUE_BITWIDTH);
    const int hysteresis_offset = divisor / 4;
    int previous_brightness_raw = *ret_brightness_raw;

    int brightness_raw;
    ESP_RETURN_ON_ERROR(adc_oneshot_read(adc_unit, BRIGHTNESS_ADC_CHANNEL, &brightness_raw), TAG, "failed to read from ADC");

    if (brightness_raw > previous_brightness_raw) {
        brightness_raw -= hysteresis_offset;
    } else if (brightness_raw < previous_brightness_raw) {
        brightness_raw += hysteresis_offset;
    }

    *ret_brightness = brightness_raw / divisor;
    *ret_brightness_raw = brightness_raw;

    return ESP_OK;
}
