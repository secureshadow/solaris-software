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

esp_err_t bmp390_init(data_t *p_dev);

esp_err_t bmp390_send_message(data_t *p_dev);

#endif // BMP390_H
