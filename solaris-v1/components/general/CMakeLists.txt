idf_component_register(
    SRCS "general.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver pressure_sensor_driver
)
