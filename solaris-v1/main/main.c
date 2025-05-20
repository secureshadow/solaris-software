<<<<<<< HEAD
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
=======
#include "mpu9250.h"
#include "bmp390.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "macros.h"

static const char *TAG = "MainApp";

// Instancias para ambos dispositivos
data_t dev; 
data_t baro_dev; 

static esp_err_t init_sensors(void) {
    esp_err_t ret;

    // Inicializar IMU
    ret = mpu9250_init(&dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando MPU9250: %d", ret);
        return ret;
>>>>>>> 7792b0e3f73e4f158d1c4c61917c9acf61f68c9f
    }
    ESP_LOGI(TAG, "Inicialización de la IMU exitosa");

<<<<<<< HEAD
    while (1) {
        read_common_sensors(&baro);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
=======
    // Inicializar Barómetro
    ret = bmp390_init(&baro_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error inicializando BMP390: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Inicialización del barómetro exitosa");

    return ESP_OK;
}

static void read_bmp390(void) {
    int32_t pressure, temperature;
    float altitude;
    esp_err_t ret = bmp390_get_measurements(&baro_dev, &pressure, &temperature, &altitude);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Presión: %ld Pa | Temp (raw): %ld | Altitud: %.2f m", pressure, temperature, altitude);
    } else {
        ESP_LOGE(TAG, "Error al obtener mediciones: %d", ret);
    }
}

int app_main(void) {
    // Espera 5 segundos antes de iniciar
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Inicializar sensores
    if (init_sensors() != ESP_OK) {
        ESP_LOGE(TAG, "Error en la inicialización de los sensores.");
        return ESP_FAIL;
    }

    // Bucle principal de lectura de datos
    while (1) {
        read_bmp390();
        vTaskDelay(pdMS_TO_TICKS(2000)); // Espera 2 segundos entre lecturas
    }

    return ESP_OK;
}
>>>>>>> 7792b0e3f73e4f158d1c4c61917c9acf61f68c9f
