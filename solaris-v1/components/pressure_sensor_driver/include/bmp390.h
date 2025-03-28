#ifndef BMP390_H
#define BMP390_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "macros.h"

// --- Pines y configuración del bus SPI (ajusta según tu hardware) ---
#define PIN_NUM_CS        22

#define BMP390_CHIP_ID_REG       0x00  
#define BMP390_CHIP_ID_VAL       0x60  
#define BMP390_SOFTRESET_REG     0x7E
#define BMP390_SOFTRESET_CMD     0xB6

// Registros para las mediciones
#define BMP390_PRESSURE_REG     0x04    // Lectura de 3 bytes (LSB, medio, MSB)
#define BMP390_TEMPERATURE_REG  0x07    // Lectura de 3 bytes (LSB, medio, MSB)

esp_err_t bmp390_init(data_t *p_dev);

esp_err_t bmp390_send_message(data_t *p_dev);

esp_err_t bmp390_read_data(bmp390_t *p_dev, uint8_t reg, uint8_t *data, size_t len);
esp_err_t bmp390_get_measurements(bmp390_t *p_dev, int32_t *pressure, int32_t *temperature, float *altitude);

#endif // BMP390_H
