#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pressure_sensor.h" 
#include "imu_driver.h"

void imu_task(void *pvParameters) {
    printf("Starting IMU initialization...\n");
    imu_init();
    printf("IMU initialized successfully.\n");
    vTaskDelete(NULL);
}

void pressure_sensor_task(void *pvParameters) {
    printf("Starting Pressure Sensor initialization...\n");
    pressure_sensor_init();
    printf("Pressure Sensor initialized successfully.\n");
    vTaskDelete(NULL);
}

void app_main() {
    printf("Creating tasks...\n");
    xTaskCreate(imu_task, "IMU Task", 2048, NULL, 5, NULL);
    xTaskCreate(pressure_sensor_task, "Pressure Sensor Task", 2048, NULL, 5, NULL);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}
