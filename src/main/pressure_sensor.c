#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "pressure_sensor.h"

#define TAG "PRESSURE_TASK"

// SPI handle global para que lo usemos entre tareas
spi_device_handle_t spi_handle;

// TODO - Missing function input parameters: spi_device_handle_t *spi_handle
void pressure_task(void *pvParameter) {

    // TODO - Here the variables passed to the function are not being used: 
    // spi_device_handle_t *spi_handle
    while (1) {
        // Leer datos del sensor de presión
        uint16_t pressure = pressure_sensor_read(spi_handle);

        // Imprimir los datos leídos
        ESP_LOGI(TAG, "Presión: %u", pressure);

        // Esperar 1 segundo
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    // Inicializar el sensor de presión
    pressure_sensor_init(&spi_handle);

    // Crear la tarea para leer el sensor
    xTaskCreate(pressure_task, "pressure_task", 2048, NULL, 5, NULL);
}
