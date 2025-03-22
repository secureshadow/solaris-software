#ifndef BMP390_H
#define BMP390_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

// --- Pines y configuración del bus SPI (ajusta según tu hardware) ---
#define SPI_HOST_USED     SPI2_HOST
#define PIN_NUM_CS        22
#define PIN_NUM_CIPO      47
#define PIN_NUM_COPI      38
#define PIN_NUM_CLK       48

#define BMP390_CHIP_ID_REG       0x00  
#define BMP390_CHIP_ID_VAL       0x60  
#define BMP390_SOFTRESET_REG     0x7E
#define BMP390_SOFTRESET_CMD     0xB6

// Estructura principal para manejar el dispositivo BMP390
typedef struct {
    spi_device_handle_t handle;
    spi_bus_config_t buscfg;
    spi_device_interface_config_t devcfg;
    spi_transaction_t trans_desc;
    uint8_t chip_id;
    uint8_t reg;
    uint8_t data;
} bmp390_t;

esp_err_t bmp390_init(bmp390_t *p_dev);

esp_err_t bmp390_send_message(bmp390_t *p_dev);

#endif // BMP390_H
