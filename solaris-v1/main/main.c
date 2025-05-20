#include "general.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MainApp";

void app_main(void)
{
    ESP_LOGI(TAG, "Esperando 3 segundos para abrir puerto serie...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    data_t baro = {0};

    esp_err_t ret = init_common_sensors(&baro);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Falló la inicialización de sensores.");
        return;
    }

    while (1) {
        read_common_sensors(&baro);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
