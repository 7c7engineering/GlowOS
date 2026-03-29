set(priv_requires "esp_driver_tsens")
idf_component_register(SRCS "glow_sensors.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES ${priv_requires})
