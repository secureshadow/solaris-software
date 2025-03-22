#include "mpu9250.h"
#include "bmp390.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MainApp";

// Instancias para ambos dispositivos
data_t dev; 
data_t baro_dev; 

int app_main(void)
{
    // Espera 5 segundos (tal como estaba en tu código original)
    vTaskDelay(pdMS_TO_TICKS(5000));

    // 1) Inicializar la IMU MPU9250 (código existente)
    esp_err_t ret = mpu9250_init(&dev);
    if (ret != ESP_OK) {
        ESP_LOGE (TAG, "Error inicializando MPU9250: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inicialización de la IMU exitosa");

    // 2) Inicializar el barómetro BMP390 (nuevo)
    ret = bmp390_init(&baro_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando BMP390: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inicialización del barómetro exitosa");

    // Aquí puedes continuar con el loop principal o más lógica si lo deseas.
    // Por ejemplo, podrías hacer lecturas periódicas de ambos sensores.

    return ESP_OK;
}
