#include "gpio_int.h"
#include "esp_log.h"

static const char *TAG = "GPIO_INT";

//Configuración del pin INT ()
esp_err_t int_gpio_init(void)
{
    esp_err_t ret = ESP_OK;

    //Limpiar Config
    ret = gpio_reset_pin(INT_GPIO);

    if (ret != ESP_OK){
        ESP_LOGE (TAG, "Error reseteando el INT");
        return ret;
    }

    //Config
    const gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << INT_GPIO),        // GPIO5
        .mode          = GPIO_MODE_INPUT,           // Entrada
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_POSEDGE          //Flanco ascendente
    };//Hai que ver como funciona o INT da IMU para saber que habilitar e que tipo de flanco buscar

    ret = gpio_config(&io_conf);

    if (ret != ESP_OK){
        ESP_LOGE (TAG, "Error configurando el INT");
        return ret;
    }

    return ret;
}

//Siguiente paso: ISR (desde o IDF, sin RTOS de momento)
