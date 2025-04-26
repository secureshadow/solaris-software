#ifndef MPU9250_H
#define MPU9250_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "macros.h"

#define PIN_NUM_CS         21 

#define REG_PWR_MGMT_1     0x6B
#define REG_WHO_AM_I       0x75
#define BIT_H_RESET        0x80

esp_err_t mpu9250_init(data_t *p_dev);
esp_err_t mpu9250_send_message(data_t *p_dev); 

#endif 
