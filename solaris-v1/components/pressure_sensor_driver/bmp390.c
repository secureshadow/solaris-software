#include "bmp390.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "freertos/task.h"  // Para vTaskDelay

static const char* TAG = "BMP390";

esp_err_t bmp390_init(bmp390_t *p_dev)
{
    // 1. Configurar la estructura del bus SPI
    p_dev->buscfg.miso_io_num = PIN_NUM_CIPO;
    p_dev->buscfg.mosi_io_num = PIN_NUM_COPI;
    p_dev->buscfg.sclk_io_num = PIN_NUM_CLK;
    p_dev->buscfg.quadwp_io_num = -1;
    p_dev->buscfg.quadhd_io_num = -1;
    p_dev->buscfg.max_transfer_sz = 4096;

    // Inicializa el bus SPI
    esp_err_t ret = spi_bus_initialize(SPI_HOST_USED, &p_dev->buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en spi_bus_initialize: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Bus SPI para BMP390 inicializado.");

    // 2. Configurar la interfaz del dispositivo
    p_dev->devcfg.clock_speed_hz = 100000; // Ajusta según requiera el sensor
    p_dev->devcfg.mode = 3;               // Prueba con 0 o 3 según datasheet
    p_dev->devcfg.spics_io_num = PIN_NUM_CS;
    p_dev->devcfg.queue_size = 1;
    p_dev->devcfg.address_bits = 0;
    p_dev->devcfg.command_bits = 0;
    p_dev->devcfg.dummy_bits = 0;
    p_dev->devcfg.flags = 0;
    p_dev->devcfg.duty_cycle_pos = 128;
    p_dev->devcfg.pre_cb = NULL;
    p_dev->devcfg.post_cb = NULL;

    vTaskDelay(pdMS_TO_TICKS(100)); // Pequeña espera para estabilizar el bus

    // Añade el dispositivo al bus SPI
    ret = spi_bus_add_device(SPI_HOST_USED, &p_dev->devcfg, &p_dev->handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en spi_bus_add_device: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Dispositivo BMP390 añadido al bus SPI.");

    // 3. Configurar transacción base
    p_dev->trans_desc.length = 16;    // 16 bits (2 bytes)
    p_dev->trans_desc.rxlength = 0;
    p_dev->trans_desc.flags = 0;

    // 4. Soft reset del sensor (si lo requiere el datasheet)
    // Guardamos en reg/data para mantener la estructura
    p_dev->reg  = BMP390_SOFTRESET_REG;
    p_dev->data = BMP390_SOFTRESET_CMD;

    // Preparar buffers TX/RX
    uint8_t tx_reset[2] = { p_dev->reg, p_dev->data };
    uint8_t rx_reset[2] = { 0, 0 };
    p_dev->trans_desc.tx_buffer = tx_reset;
    p_dev->trans_desc.rx_buffer = rx_reset;

    ret = bmp390_send_message(p_dev);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al enviar soft reset: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Soft reset enviado al BMP390.");
    vTaskDelay(pdMS_TO_TICKS(10)); // Espera breve para que se reinicie

    // 5. Leer Chip ID (para comprobar comunicación)
    //    Típicamente, para lectura se pone el bit 7 en 1 (0x80 | reg)
    p_dev->reg = BMP390_CHIP_ID_REG;
    // data puede ser 0, no se usa en la lectura
    uint8_t tx_chipid[2] = { (uint8_t)(0x80 | p_dev->reg), 0x00 };
    uint8_t rx_chipid[2] = { 0, 0 };
    p_dev->trans_desc.tx_buffer = tx_chipid;
    p_dev->trans_desc.rx_buffer = rx_chipid;

    ret = bmp390_send_message(p_dev);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer Chip ID: %d", ret);
        return ret;
    }

    // El segundo byte es el dato real
    p_dev->chip_id = rx_chipid[1];
    ESP_LOGI(TAG, "Chip ID leído: 0x%02X", p_dev->chip_id);

    // Comprueba si coincide con el esperado
    if (p_dev->chip_id != BMP390_CHIP_ID_VAL) {
        ESP_LOGE(TAG, "Chip ID incorrecto (esperado 0x%02X, leído 0x%02X)",
                 BMP390_CHIP_ID_VAL, p_dev->chip_id);
        return ESP_FAIL;
    }

    // 6. (Opcional) Configurar otros registros: oversampling, modos, etc.

    ESP_LOGI(TAG, "Inicialización de BMP390 completada con éxito.");
    return ESP_OK;
}

esp_err_t bmp390_send_message(bmp390_t *p_dev)
{
    esp_err_t ret = ESP_OK;
    // Ejecuta la transacción SPI usando la configuración en p_dev->trans_desc
    ret = spi_device_transmit(p_dev->handle, &p_dev->trans_desc);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en la transacción SPI: %d", ret);
        return ret;
    } else {
        ESP_LOGI(TAG, "Sensor despertado (PWR_MGMT_1 = 0x00).");
        vTaskDelay(pdMS_TO_TICKS(10));  // Breve espera para estabilizar el sensor
        return ret;
    }

}
