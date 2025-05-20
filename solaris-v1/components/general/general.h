#ifndef GENERAL_H
#define GENERAL_H

#include "esp_err.h"
#include "macros.h"

esp_err_t init_common_sensors(data_t *baro);
void read_common_sensors(data_t *baro);

#endif // GENERAL_H
