#include "bmp390.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "BMP390"

// Pines según la conexión física
#define PIN_NUM_MISO 47  // CIPO
#define PIN_NUM_MOSI 38  // COPI
#define PIN_NUM_CLK  48  // SCK
#define PIN_NUM_CS   18  // CS

esp_err_t bmp390_init(spi_device_handle_t *handle)
{
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al inicializar el bus SPI: %d", ret);
        return ret;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 500 * 1000,  // 500 kHz
        .mode = 0,                   // SPI modo 0: CPOL=0, CPHA=0
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .command_bits = 8,           // 8 bits para la instrucción (registro con bit de lectura)
        .dummy_bits = 8,             // 8 bits dummy para leer la respuesta
    };

    ret = spi_bus_add_device(SPI3_HOST, &devcfg, handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al añadir el dispositivo SPI: %d", ret);
        return ret;
    }

    ESP_LOGI(TAG, "SPI inicializado correctamente");
    return ESP_OK;
}

esp_err_t bmp390_soft_reset(spi_device_handle_t handle)
{
    uint8_t tx_data[2] = { BMP390_SOFT_RESET_REG & 0x7F, BMP390_SOFT_RESET_CMD };
    // Aquí usamos la transacción tradicional (sin command_bits) para el soft reset
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;  // 16 bits
    t.tx_buffer = tx_data;

    esp_err_t ret = spi_device_transmit(handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al enviar soft reset: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Soft reset enviado, esperando 100 ms");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    return ESP_OK;
}

esp_err_t bmp390_enable_spi_mode(spi_device_handle_t handle)
{
    // Para escribir usamos la transacción tradicional (sin utilizar command_bits)
    uint8_t tx_data[2] = { BMP390_IF_CONF_REG & 0x7F, BMP390_IF_CONF_SPI };
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.tx_buffer = tx_data;

    esp_err_t ret = spi_device_transmit(handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al habilitar modo SPI: %d", ret);
        return ret;
    }
    ESP_LOGI(TAG, "Modo SPI activado correctamente (IF_CONF=0x%02X)", BMP390_IF_CONF_SPI);
    return ESP_OK;
}


esp_err_t bmp390_read_reg(spi_device_handle_t handle, uint8_t reg, uint8_t *val)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    // Se usa el campo cmd para enviar el registro a leer con el bit de lectura activado:
    t.cmd = reg | 0x80;
    t.length = 8;      // Se esperan 8 bits (dummy) para la lectura
    t.rxlength = 8;    // Se esperan 8 bits de respuesta
    t.rx_buffer = val;
    return spi_device_polling_transmit(handle, &t);
}

esp_err_t bmp390_read_if_conf(spi_device_handle_t handle, uint8_t *if_conf)
{
    esp_err_t ret = bmp390_read_reg(handle, BMP390_IF_CONF_REG, if_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer IF_CONF: %d", ret);
    } else {
        ESP_LOGI(TAG, "IF_CONF leído: 0x%02X", *if_conf);
    }
    return ret;
}

esp_err_t bmp390_read_chip_id(spi_device_handle_t handle, uint8_t *chip_id)
{
    esp_err_t ret = bmp390_read_reg(handle, BMP390_CHIP_ID_REG, chip_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer CHIP ID: %d", ret);
    } else {
        ESP_LOGI(TAG, "CHIP ID leído: 0x%02X", *chip_id);
    }
    return ret;
}

// —— A PARTIR DE AQUÍ, LO AÑADIDO SIN REDEFINIR NADA MÁS ——

// Bajo nivel: escribir registro
esp_err_t bmp390_write_reg(spi_device_handle_t spi, uint8_t reg_addr, uint8_t data)
{
    spi_transaction_t t = {
        .flags   = SPI_TRANS_USE_TXDATA,
        .tx_data = { reg_addr & 0x7F, data },
        .length  = 16,
    };
    return spi_device_transmit(spi, &t);
}

esp_err_t bmp390_read_regs(spi_device_handle_t spi, uint8_t reg_start, uint8_t *data, size_t len)
{
    spi_transaction_t t = {0};
    t.cmd = reg_start | 0x80; // registro inicial con bit de lectura
    t.length = len * 8;
    t.rxlength = len * 8;
    t.rx_buffer = data;
    return spi_device_polling_transmit(spi, &t);
}

// Variables internas para calibración de temperatura
static float _par_t1, _par_t2, _par_t3;

// Burst-read de coeficientes de temperatura (0x31…0x36)
esp_err_t bmp390_read_calibration(spi_device_handle_t spi)
{
    uint8_t buf[5];
    bmp390_read_regs(spi, 0x31, buf, 5);

    // Ahora buf[0]=0x31, buf[1]=0x32, buf[2]=0x33, buf[3]=0x34, buf[4]=0x35
    uint16_t raw_t1 = ((uint16_t)buf[0] << 8) | buf[1];  // 0x31(MSB)/0x32(LSB)
    uint16_t raw_t2 = ((uint16_t)buf[2] << 8) | buf[3];  // 0x33(MSB)/0x34(LSB)
    int8_t  raw_t3 = (int8_t)buf[4];                    // 0x35 (T3)

    // NUEVO: Imprime los valores crudos para depuración
    ESP_LOGI(TAG, "Buf calib: %02X %02X %02X %02X %02X", buf[0], buf[1], buf[2], buf[3], buf[4]);

    _par_t1 = raw_t1 / 256.0f;               // 2^-8
    _par_t2 = raw_t2 / 1073741824.0f;        // 2^30 :contentReference[oaicite:1]{index=1}
    _par_t3 = raw_t3 / 281474976710656.0f;
    ESP_LOGI(TAG, "Calib T: t1=%.6f t2=%.6f t3=%.12f", _par_t1, _par_t2, _par_t3);
    return ESP_OK;
}


// Lectura cruda de temperatura (3 bytes DATA_3…DATA_5)
esp_err_t bmp390_read_raw_temperature(spi_device_handle_t spi, uint32_t *uncomp_temp)
{
    uint8_t raw[3];
    esp_err_t r = bmp390_read_regs(spi, BMP390_REG_DATA_3, raw, 3);
    if (r != ESP_OK) return r;

    ESP_LOGI(TAG, "Temp raw: %02X %02X %02X", raw[0], raw[1], raw[2]);


    // Convert 24-bit register data to 20-bit ADC value by discarding 4 LSBs
     uint32_t u24 = ((uint32_t)raw[2] << 16)
                  | ((uint32_t)raw[1] <<  8)
                  |  (uint32_t)raw[0];
     *uncomp_temp = u24 >> 4;  // now a true 20-bit uncomp_temp
    return ESP_OK;
}

// Compensación de temperatura en °C (datasheet §3.10)
float BMP390_compensate_temperature(uint32_t uncomp_temp)
{
    float pd1 = (float)uncomp_temp - _par_t1;
    float pd2 = pd1 * _par_t2;
    return pd2 + (pd1 * pd1) * _par_t3;
}

float bmp390_get_par_t1(void) { return _par_t1; }
float bmp390_get_par_t2(void) { return _par_t2; }
float bmp390_get_par_t3(void) { return _par_t3; }
