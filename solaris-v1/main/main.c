#include <stdio.h>
#include <inttypes.h>
#include "bmp390.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    spi_device_handle_t spi;
    esp_err_t ret;
    uint8_t id, ifc;
    bmp390_temp_calib_t raw_calib;
    bmp390_temp_params_t temp_params;
    uint32_t raw_temp;

    // Tiempo para estabilizar alimentación
    vTaskDelay(pdMS_TO_TICKS(2000));

    //---------Inicializar----------
    ret = bmp390_init(&spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error init BMP390: %d", ret);
        return;
    }

    ret = bmp390_soft_reset(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error soft reset: %d", ret);
        return;
    }

    ret = bmp390_enable_spi_mode(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error enable SPI mode: %d", ret);
        return;
    }
    
    // Dejar tiempo al primer ciclo de configuración
    vTaskDelay(pdMS_TO_TICKS(50));

    //-------Comprobación----------
    ret = bmp390_read_chip_id(spi, &id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer CHIP ID: %d", ret);
        return;
    }
    if (id != BMP390_CHIP_ID_VALUE) {
        ESP_LOGE(TAG, "CHIP ID inesperado: 0x%02X", id);
        return;
    }
    ESP_LOGI(TAG, "CHIP ID confirmado: 0x%02X", id);

    ret = bmp390_read_if_conf(spi, &ifc);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer IF_CONF: %d", ret);
        return;
    }

    //-------Configuración sensor-------
    ret = bmp390_set_mode_normal(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error modo normal: %d", ret);
        return;
    }
    ret = bmp390_set_osr_temp(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error OSR: %d", ret);
        return;
    }
    ret = bmp390_set_odr(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error ODR: %d", ret);
        return;
    }
    ret = bmp390_set_iir(spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error IIR: %d", ret);
        return;
    }

    // Dejar tiempo al primer dato en Normal mode
    vTaskDelay(pdMS_TO_TICKS(50));

    // ---Leer coeficientes de temperatura raw---
    ret = bmp390_read_raw_coeffs(spi, &raw_calib);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Coef raw T1=%u, T2=%d, T3=%d",
                 raw_calib.par_t1,
                 raw_calib.par_t2,
                 raw_calib.par_t3);
    } else {
        ESP_LOGE(TAG, "Error al leer coef. calib.: %d", ret);
        return;
    }

    // --- Calibrar parámetros de temperatura ---
    ret = bmp390_calibrate_params(spi, &temp_params);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "PAR_T1 calibrado: %.4f",  temp_params.PAR_T1);
        ESP_LOGI(TAG, "PAR_T2 calibrado: %.6e",  temp_params.PAR_T2);
        ESP_LOGI(TAG, "PAR_T3 calibrado: %.6e",  temp_params.PAR_T3);
    } else {
        ESP_LOGE(TAG, "Error al calibrar params: %d", ret);
        return;
    }

    //--- Bucle de lectura de temperatura cruda ---
       while (1) {
        ret = bmp390_wait_temp_ready(spi);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Error wait temp ready: %d", ret);
            break;
        }

        ret = bmp390_read_raw_temp(spi, &raw_temp);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Raw temp: %u", raw_temp);

            // ← aquí añadimos la compensación
            float comp_temp = bmp390_compensate_temperature(raw_temp, &temp_params);
            ESP_LOGI(TAG, "Temp compensada: %.2f °C", comp_temp);

        } else {
            ESP_LOGE(TAG, "Error read raw temp: %d", ret);
            break;
        }

        // Delay para pruebas en banco (5 s)
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}