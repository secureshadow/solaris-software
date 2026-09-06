# SPP API — a short tour {#spp-api}

This page is a quick orientation to the C API. The full reference — every file,
struct, function and macro — is under <a href="files.html">Files</a> and
<a href="annotated.html">Data Structures</a>, generated from the source
comments in `solaris-v2/`.

## Entry point

The application is deliberately tiny: it wires the ESP32-S3 HAL port into the
finite state machine and then ticks it forever.

```c
void app_main(void)
{
    const SPP_HalPort_t *p_halPorts = SPP_PORTS_ESP32S3_getHalPorts();
    (void)FSM_init((void *)p_halPorts);
    while (1) {
        FSM_tick();
    }
}
```

Everything else — sensor sampling, packet routing, logging, downlink — is driven
from the FSM (@ref fsm.h) through the publish/subscribe core (@ref pubsub.h).

## The layers

| Layer | Headers | What it does |
|-------|---------|--------------|
| Core     | @ref packet.h, @ref pubsub.h, @ref core.h, @ref commonbit.h | packet format, pub/sub, core init |
| Common   | @ref types.h, @ref returnTypes.h, @ref version.h | fixed-width types, return codes |
| HAL      | @ref hal.h, @ref spi.h, @ref uart.h, @ref gpio.h, @ref storage.h, @ref time.h | the peripheral contract (function pointers only) |
| Port     | @ref halEsp32.h, @ref macrosEsp32.h | the ESP32-S3 implementation of that contract |
| Services | @ref fsm.h, @ref databank.h, @ref datalogger.h, @ref log.h, @ref bmp390.h, @ref icm20948.h, @ref sx1262.h | orchestrator, packet pool, sensors, downlink |
| External | @ref encryption.h, @ref cipher.h | AES-128-GCM, compiled in only when `SPP_ENCRYPTION` is set |

## Reference hardware

| Component  | Device    | Bus       | Notes                    |
|------------|-----------|-----------|--------------------------|
| MCU        | ESP32-S3  | —         | ESP-IDF, ESP32-S3 target |
| IMU        | ICM-20948 | SPI2_HOST | DMP FIFO, GPIO data-ready |
| Barometer  | BMP390    | SPI2_HOST | interrupt-driven         |
| GNSS       | MAX-M10M  | UART      | NMEA / UBX               |
| Downlink   | SX1262    | SPI + DIO | LoRa, work in progress   |

The authoritative pin map is defined in @ref macrosEsp32.h.

## Building

```sh
cd solaris-v2
idf.py set-target esp32s3
idf.py build
```

Service inclusion is controlled with CMake options, e.g.
`idf.py build -DSPP_SERVICE_ICM20948=OFF`. See the @ref build-system page and
`compiler/spp/CMakeLists.txt`.

## License

Copyright &copy; 2025 Team Solaris — UVigo Aerotech. Licensed under
**CC BY-NC-SA 4.0**: share and adapt with attribution, non-commercial,
derivatives under the same terms. See `LICENSE.md`.
