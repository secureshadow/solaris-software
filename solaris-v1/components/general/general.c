#include "general.h"
#include "bmp390.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "General";

esp_err_t init_common_sensors(data_t *baro)
{
    esp_err_t ret;

    ret = bmp390_init(baro);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando BMP390: %d", ret);
        return ret;
    }

    ret = bmp390_soft_reset(baro);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en soft reset BMP390: %d", ret);
        return ret;
    }

    ret = bmp390_enable_spi_mode(baro);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error activando SPI BMP390: %d", ret);
        return ret;
    }

    ret = bmp390_configure_oversampling(baro);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando oversampling BMP390: %d", ret);
        return ret;
    }

    ret = bmp390_start_normal_mode(baro);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error activando modo normal BMP390: %d", ret);
        return ret;
    }

    uint8_t if_conf = 0;
    ret = bmp390_read_if_conf(baro, &if_conf);
    if (ret != ESP_OK || if_conf != BMP390_IF_CONF_SPI) {
        ESP_LOGE(TAG, "IF_CONF inválido: 0x%02X", if_conf);
        return ESP_FAIL;
    }

    uint8_t chip_id = 0;
    ret = bmp390_read_chip_id(baro, &chip_id);
    if (ret != ESP_OK || chip_id != BMP390_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "CHIP ID inválido: 0x%02X", chip_id);
        return ESP_FAIL;
    }

    return ESP_OK;
}

void read_common_sensors(data_t *baro)
{
    int32_t pressure = 0, temperature = 0;
    float altitude = 0.0f;

    esp_err_t ret = bmp390_get_measurements(baro, &pressure, &temperature, &altitude);
    if (ret == ESP_OK) {
        ESP_LOGI("SENSORES", "Presión: %ld Pa | Temp (raw): %ld | Altitud: %.2f m",
                 pressure, temperature, altitude);
    } else {
        ESP_LOGE("SENSORES", "Error leyendo mediciones del BMP390");
    }

    // Lectura de otros sensores (IMU, etc.) puede agregarse aquí
}
