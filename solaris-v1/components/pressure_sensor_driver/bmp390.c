#include "bmp390.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

#define TAG "BMP390"

esp_err_t bmp390_init(data_t *dev)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_CIPO,
        .mosi_io_num = PIN_NUM_COPI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    esp_err_t ret = spi_bus_initialize(SPI_HOST_USED, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Error al inicializar el bus SPI: %d", ret);
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 500000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
        .command_bits = 8,
        .dummy_bits = 8,
    };

    ret = spi_bus_add_device(SPI_HOST_USED, &devcfg, &dev->handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al añadir el dispositivo SPI: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "SPI inicializado correctamente");
    return ESP_OK;
}

esp_err_t bmp390_soft_reset(data_t *dev)
{
    uint8_t tx_data[2] = { BMP390_SOFT_RESET_REG & 0x7F, BMP390_SOFT_RESET_CMD };
    spi_transaction_t t = {0};
    t.length = 16;
    t.tx_buffer = tx_data;

    esp_err_t ret = spi_device_transmit(dev->handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al enviar soft reset: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Soft reset enviado, esperando 100 ms");
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

esp_err_t bmp390_enable_spi_mode(data_t *dev)
{
    uint8_t tx_data[2] = { BMP390_IF_CONF_REG & 0x7F, BMP390_IF_CONF_SPI };
    spi_transaction_t t = {0};
    t.length = 16;
    t.tx_buffer = tx_data;

    esp_err_t ret = spi_device_transmit(dev->handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al habilitar modo SPI: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Modo SPI activado correctamente (IF_CONF=0x%02X)", BMP390_IF_CONF_SPI);
    return ESP_OK;
}

esp_err_t bmp390_configure_oversampling(data_t *dev)
{
    uint8_t tx_data[2] = { BMP390_OSR_REG & 0x7F, BMP390_OSR_STANDARD };
    spi_transaction_t t = {0};
    t.length = 16;
    t.tx_buffer = tx_data;

    esp_err_t ret = spi_device_transmit(dev->handle, &t);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Oversampling configurado (OSR=0x%02X)", BMP390_OSR_STANDARD);
    } else {
        ESP_LOGE(TAG, "Error al configurar oversampling: %d", ret);
    }

    return ret;
}

esp_err_t bmp390_start_normal_mode(data_t *dev)
{
    uint8_t tx_data[2] = { BMP390_PWR_CTRL_REG & 0x7F, BMP390_PWR_CTRL_NORMAL };
    spi_transaction_t t = {0};
    t.length = 16;
    t.tx_buffer = tx_data;

    esp_err_t ret = spi_device_transmit(dev->handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar modo normal: %d", ret);
    } else {
        ESP_LOGI(TAG, "Modo normal activado (PWR_CTRL=0x%02X)", BMP390_PWR_CTRL_NORMAL);
    }
    return ret;
}

static esp_err_t bmp390_read_reg(data_t *dev, uint8_t reg, uint8_t *val)
{
    spi_transaction_t t = {0};
    t.cmd = reg | 0x80;
    t.length = 8;
    t.rxlength = 8;
    t.rx_buffer = val;

    return spi_device_polling_transmit(dev->handle, &t);
}

static esp_err_t bmp390_read_bytes(data_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t tx[1] = { reg | 0x80 };
    uint8_t temp_rx[len + 1];
    memset(temp_rx, 0, sizeof(temp_rx));

    spi_transaction_t t = {0};
    t.length = (1 + len) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = temp_rx;
    t.rxlength = len * 8;

    esp_err_t ret = spi_device_transmit(dev->handle, &t);
    if (ret == ESP_OK) {
        memcpy(buf, &temp_rx[1], len); // Ignorar primer byte
    }
    return ret;
}

esp_err_t bmp390_read_if_conf(data_t *dev, uint8_t *if_conf)
{
    esp_err_t ret = bmp390_read_reg(dev, BMP390_IF_CONF_REG, if_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer IF_CONF: %d", ret);
    } else {
        ESP_LOGI(TAG, "IF_CONF leído: 0x%02X", *if_conf);
    }
    return ret;
}

esp_err_t bmp390_read_chip_id(data_t *dev, uint8_t *chip_id)
{
    esp_err_t ret = bmp390_read_reg(dev, BMP390_CHIP_ID_REG, chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer CHIP ID: %d", ret);
    } else {
        ESP_LOGI(TAG, "CHIP ID leído: 0x%02X", *chip_id);
    }
    return ret;
}

esp_err_t bmp390_get_measurements(data_t *dev, int32_t *pressure, int32_t *temperature, float *altitude)
{
    uint8_t press_bytes[3] = {0}, temp_bytes[3] = {0};

    esp_err_t ret = bmp390_read_bytes(dev, 0x04, press_bytes, 3);
    if (ret != ESP_OK) return ret;

    ret = bmp390_read_bytes(dev, 0x07, temp_bytes, 3);
    if (ret != ESP_OK) return ret;

    int32_t raw_pressure = ((int32_t)press_bytes[2] << 16) |
                           ((int32_t)press_bytes[1] << 8)  |
                            press_bytes[0];
    int32_t raw_temp = ((int32_t)temp_bytes[2] << 16) |
                       ((int32_t)temp_bytes[1] << 8)  |
                        temp_bytes[0];

    if (raw_pressure & 0x800000) raw_pressure |= 0xFF000000;
    if (raw_temp & 0x800000) raw_temp |= 0xFF000000;

    *pressure = raw_pressure;
    *temperature = raw_temp;

    float pressure_hPa = raw_pressure / 100.0f;
    *altitude = 44330.0f * (1.0f - powf(pressure_hPa / 1013.25f, 0.1903f));

    return ESP_OK;
}
