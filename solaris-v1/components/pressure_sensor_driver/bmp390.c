#include "bmp390.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "freertos/task.h"  // Para vTaskDelay
#include <string.h>
#include <math.h>
#include "macros.h"

static const char* TAG = "BMP390";

esp_err_t bmp390_init(data_t *p_dev)
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

esp_err_t bmp390_send_message(data_t *p_dev)
{
    esp_err_t ret = ESP_OK;
    // Ejecuta la transacción SPI usando la configuración en p_dev->trans_desc
    ret = spi_device_transmit(p_dev->handle, &p_dev->trans_desc);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en la transacción SPI: %d", ret);
        return ret;
    } else {
        ESP_LOGI(TAG, "Transacción SPI completada para BMP390. (PWR_MGMT_1 = 0x00).");
        vTaskDelay(pdMS_TO_TICKS(10));  // Breve espera para estabilizar el sensor
        return ret;
    }

}

esp_err_t bmp390_read_data(bmp390_t *p_dev, uint8_t reg, uint8_t *data, size_t len)
{
    // La transacción incluirá el byte de comando y los bytes de datos a leer.
    size_t total_len = len + 1;
    uint8_t tx_buffer[total_len];
    uint8_t rx_buffer[total_len];
    memset(tx_buffer, 0, total_len);
    memset(rx_buffer, 0, total_len);

    tx_buffer[0] = 0x80 | reg; // Bit de lectura activado
    // Los bytes siguientes son dummy (0) para generar los pulsos del reloj

    spi_transaction_t trans = {0};
    trans.length = total_len * 8; // Total de bits a transferir
    trans.tx_buffer = tx_buffer;
    trans.rx_buffer = rx_buffer;

    esp_err_t ret = spi_device_transmit(p_dev->handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo registros 0x%02X: %d", reg, ret);
        return ret;
    }
    // Se omite el primer byte (envío) y se copian los datos recibidos
    memcpy(data, &rx_buffer[1], len);
    return ESP_OK;
}

esp_err_t bmp390_get_measurements(bmp390_t *p_dev, int32_t *pressure, int32_t *temperature, float *altitude)
{
    esp_err_t ret;
    uint8_t press_bytes[3] = {0};
    uint8_t temp_bytes[3] = {0};

    // Leer 3 bytes de presión
    ret = bmp390_read_data(p_dev, BMP390_PRESSURE_REG, press_bytes, 3);
    if (ret != ESP_OK) return ret;
    // Leer 3 bytes de temperatura
    ret = bmp390_read_data(p_dev, BMP390_TEMPERATURE_REG, temp_bytes, 3);
    if (ret != ESP_OK) return ret;

    // Combinar los bytes (asumiendo que el dato es de 24 bits en formato little-endian)
    int32_t raw_pressure = ((int32_t)press_bytes[2] << 16) | ((int32_t)press_bytes[1] << 8) | press_bytes[0];
    int32_t raw_temperature = ((int32_t)temp_bytes[2] << 16) | ((int32_t)temp_bytes[1] << 8) | temp_bytes[0];

    // Extender signo si es necesario (24 bits)
    if(raw_pressure & 0x800000) {
        raw_pressure |= 0xFF000000;
    }
    if(raw_temperature & 0x800000) {
        raw_temperature |= 0xFF000000;
    }

    // Para efectos de este ejemplo, asumimos que raw_pressure está en Pascales.
    *pressure = raw_pressure;
    *temperature = raw_temperature;

    // Calcular la altitud usando la fórmula barométrica:
    // altitude = 44330 * (1 - (pressure_hPa / seaLevel_hPa)^(0.1903))
    // Se convierte la presión a hectopascales (hPa) dividiendo por 100.
    float pressure_hPa = raw_pressure / 100.0f;
    float seaLevel_hPa = 1013.25f; // Valor típico a nivel del mar
    *altitude = 44330.0f * (1.0f - powf(pressure_hPa / seaLevel_hPa, 0.1903f));

    return ESP_OK;
}