#ifndef PRESSURE_SENSOR_H
#define PRESSURE_SENSOR_H

#include <stdint.h>
#include "driver/spi_master.h"

// Configuración del SPI para el sensor
#define SPI_CLOCK_SPEED_HZ 1000000  // Velocidad de reloj de SPI (ajusta según el sensor)
#define SPI_MODE 0                  // Modo SPI (ajusta según el sensor)
#define SPI_CS_PIN 5                // Pin para Chip Select (ajusta según tu diseño)

// Funciones para inicializar y leer del sensor
void pressure_sensor_init(spi_device_handle_t *spi_handle);
uint16_t pressure_sensor_read(spi_device_handle_t spi_handle);

#endif // PRESSURE_SENSOR_H
