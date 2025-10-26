#ifndef BMP390_H
#define BMP390_H

#include <stdint.h>
#include "driver/spi_master.h" 
#include "general.h"


//---------------------INIT------------------------------
#define PIN_NUM_CS   18

void bmp390_init(void *p_dev);

//---------------------AUX------------------------------
esp_err_t bmp390_write_reg(data_t *p_dev, uint8_t reg, uint8_t value);
esp_err_t bmp390_read(data_t *p_dev, uint8_t reg, uint8_t *dst, size_t len);

//--------------------CONFIG and CHECK---------------------------

#define BMP390_CHIP_ID_REG    0x00
#define BMP390_CHIP_ID_VALUE  0x60

#define BMP390_SOFT_RESET_REG 0x7E
#define BMP390_SOFT_RESET_CMD 0xB6

#define BMP390_IF_CONF_REG    0x1A
#define BMP390_IF_CONF_SPI    0x00

esp_err_t bmp390_soft_reset(data_t *p_dev);
esp_err_t bmp390_enable_spi_mode(data_t *p_dev);

esp_err_t bmp390_read_if_conf(data_t *p_dev, uint8_t *if_conf);
esp_err_t bmp390_read_chip_id(data_t *p_dev, uint8_t *chip_id);

//-----------------------PREPARE READ-----------------------

//Modo
#define BMP390_REG_PWRCTRL     0x1B
#define BMP390_VALUE_PWRCTRL   0x33   //(0x30 | 0x01 | 0x02) = 0x33 (normal|press_en|temp_en)

esp_err_t bmp390_set_mode_normal(data_t *p_dev);

//Oversampling
#define BMP390_REG_OSR           0x1C
#define BMP390_VALUE_OSR         0x00 //adaptar según que busquemos (precisión-tiempo-energía) con este +-0.2m

esp_err_t bmp390_set_osr_temp(data_t *p_dev);

//Output Data Rate
#define BMP390_REG_ODR         0x1D
#define BMP390_VALUE_ODR       0x02 //50Hz

esp_err_t bmp390_set_odr(data_t *p_dev);

//Filtro
#define BMP390_REG_IIR   0x1F
#define BMP390_VALUE_IIR    0x02 //coeficiente 1 (adaptar)

esp_err_t bmp390_set_iir(data_t *p_dev);

//Status
#define BMP390_REG_STATUS         0x03

#define BMP390_STATUS_DRDY_TEMP   0x40
#define BMP390_STATUS_DRDY_PRES  0x20

esp_err_t bmp390_read_status(data_t *p_dev, uint8_t *status);
esp_err_t bmp390_wait_temp_ready(data_t *p_dev);
esp_err_t bmp390_wait_press_ready(data_t *p_dev);


//------------------------READ TEMP--------------------------

// Dirección inicial de los coeficientes de temperatura
#define BMP390_TEMP_CALIB_REG_START  0x31
typedef struct {
    uint16_t par_t1;   // 0x31 (LSB), 0x32 (MSB)  → u16
    int16_t  par_t2;   // 0x33 (LSB), 0x34 (MSB)  → s16
    int8_t   par_t3;   // 0x35                  → s8

    float    t_lin;
} bmp390_temp_calib_t;

esp_err_t bmp390_read_raw_temp_coeffs(data_t *p_dev, bmp390_temp_calib_t *tcalib);

typedef struct {
    float PAR_T1;   // Ej. ≈ 28159 / 2^8  ≈ 109.996
    float PAR_T2;   // Ej. ≈ 19960 / 2^30 ≈ 1.859e-5
    float PAR_T3;   // Ej. ≈ −7    / 2^48 ≈ −2.487e-14
} bmp390_temp_params_t;

esp_err_t bmp390_calibrate_temp_params(data_t *p_dev, bmp390_temp_params_t *out);

#define BMP390_TEMP_RAW_REG    0x07
esp_err_t bmp390_read_raw_temp(data_t *p_dev, uint32_t *raw_temp);

float bmp390_compensate_temperature(uint32_t raw_temp, bmp390_temp_params_t *params);

//------------------------READ PRESS--------------------------
// Dirección inicial de los coeficientes de presión
#define BMP390_PRESS_CALIB_REG_START  0x36
typedef struct {
    uint16_t par_p1;   // 0x36 (LSB), 0x37 (MSB)
    uint16_t par_p2;   // 0x38 (LSB), 0x39 (MSB)
    int8_t   par_p3;   // 0x3A
    int8_t   par_p4;   // 0x3B
    uint16_t  par_p5;   // 0x3C (LSB), 0x3D (MSB)
    uint16_t   par_p6;   // 0x3E
    int8_t   par_p7;   // 0x40
    int8_t   par_p8;   // 0x41
    int16_t  par_p9;   // 0x42 (LSB), 0x43 (MSB)
    int8_t   par_p10;  // 0x44
    int8_t   par_p11;  // 0x45
} bmp390_press_calib_t;

esp_err_t bmp390_read_raw_press_coeffs(data_t *p_dev, bmp390_press_calib_t *pcalib);

typedef struct {
    float PAR_P1;   // = (raw.par_p1 - 2^14) / 2^20
    float PAR_P2;   // = (raw.par_p2 - 2^14) / 2^29
    float PAR_P3;   // = raw.par_p3 / 2^32
    float PAR_P4;   // = raw.par_p4 / 2^37
    float PAR_P5;   // = raw.par_p5 / 2^-3
    float PAR_P6;   // = raw.par_p6 / 2^6
    float PAR_P7;   // = raw.par_p7 / 2^8
    float PAR_P8;   // = raw.par_p8 / 2^15
    float PAR_P9;   // = raw.par_p9 / 2^48
    float PAR_P10;  // = raw.par_p10 / 2^48
    float PAR_P11;  // = raw.par_p11 / 2^65
} bmp390_press_params_t;

esp_err_t bmp390_calibrate_press_params(data_t *p_dev, bmp390_press_params_t *out);

#define BMP390_PRESS_RAW_REG    0x04
esp_err_t bmp390_read_raw_press(data_t *p_dev, uint32_t *raw_press);

float bmp390_compensate_pressure(uint32_t raw_press, float t_lin, bmp390_press_params_t *params);

//-----------Aux Functions-----------
esp_err_t bmp390_config(data_t *p_dev);

void bmp390_prepare_mode(data_t *p_dev);
void bmp390_prepare_temp(data_t *p_dev);
void bmp390_prepare_press(data_t *p_dev);
esp_err_t bmp390_prepare_read(data_t *p_dev);

esp_err_t bmp390_read_temp(data_t *p_dev);
esp_err_t bmp390_calc_altitude(data_t *p_dev);
esp_err_t bmp390_read_measurements(data_t *p_dev);


#endif  // BMP390_H