#ifndef ICM20948_H
#define ICM20948_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

// Macros
#define SPI_HOST_USED SPI2_HOST 
#define PIN_NUM_CS         21 
#define PIN_NUM_CIPO       47
#define PIN_NUM_COPI       38
#define PIN_NUM_CLK        48

#define READ_OP            0x80
#define WRITE_OP           0x00

#define REG_PWR_MGMT_1     0x06
#define REG_WHO_AM_I       0x00

#define START_CONECTION    0x00 //Para iniciar la conexión, se pone el CS a 0x00
#define BIT_H_RESET        0x80
#define EMPTY_MESSAGE      0x00

typedef struct {
    spi_device_handle_t handle;
    spi_bus_config_t buscfg;
    spi_device_interface_config_t devcfg;
    spi_transaction_t trans_desc;
    uint8_t who_am_i;
    uint8_t reg;
    uint8_t data;
} icm20948_t;

esp_err_t icm20948_init(icm20948_t *p_dev);
esp_err_t icm20948_send_message(icm20948_t *p_dev); 

#endif
