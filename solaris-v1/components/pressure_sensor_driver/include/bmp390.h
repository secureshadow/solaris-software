#ifndef BMP390_H
#define BMP390_H

#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_err.h"
#include "macros.h"

// Pines SPI del sensor
#define PIN_NUM_CS 18

// Registros y valores esperados
#define BMP390_CHIP_ID_REG        0x00
#define BMP390_CHIP_ID_VALUE      0x60

#define BMP390_SOFT_RESET_REG     0x7E
#define BMP390_SOFT_RESET_CMD     0xB6

#define BMP390_IF_CONF_REG        0x1A
#define BMP390_IF_CONF_SPI        0x00

#define BMP390_PWR_CTRL_REG       0x1B
#define BMP390_PWR_CTRL_NORMAL    0x30

#define BMP390_OSR_REG            0x1C
#define BMP390_OSR_STANDARD       0x0A

#define BMP390_PRESSURE_REG       0x04
#define BMP390_TEMPERATURE_REG    0x07

// Funciones públicas
esp_err_t bmp390_init(data_t *dev);
esp_err_t bmp390_soft_reset(data_t *dev);
esp_err_t bmp390_enable_spi_mode(data_t *dev);
esp_err_t bmp390_configure_oversampling(data_t *dev);
esp_err_t bmp390_start_normal_mode(data_t *dev);
esp_err_t bmp390_read_if_conf(data_t *dev, uint8_t *if_conf);
esp_err_t bmp390_read_chip_id(data_t *dev, uint8_t *chip_id);
esp_err_t bmp390_get_measurements(data_t *dev, int32_t *pressure, int32_t *temperature, float *altitude);

#endif // BMP390_H
