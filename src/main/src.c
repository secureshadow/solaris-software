#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
// #include "pressure_sensor.h" 

#define LED_PIN GPIO_NUM_2  

/*  
    TODO - Create FreeRTOS task that uses the pressure_sensor.h declared functions
    OR - you can decalre the function in pressure_sensor.c and use it directly in main()

    void get_pressure_data(){

        #Declare variables here

        while(1){
            pressure_sensor_init();
            pressure_sensor_read();
        }

    }
*/

void blink_task(void *pvParameter) {

    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(LED_PIN, 1);  
        vTaskDelay(1000 / portTICK_PERIOD_MS);  
        gpio_set_level(LED_PIN, 0); 
        vTaskDelay(1000 / portTICK_PERIOD_MS);  
    }
}

void app_main(void) {

    xTaskCreate(blink_task, "blink_task", 1024, NULL, 5, NULL);

    // Call the pressure sensor task here:
    // xTaskCreate(get_pressure_data, ...)

}