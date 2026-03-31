set(priv_requires "glow_context" "esp_driver_tsens" "esp_adc")
idf_component_register(SRCS "glow_sensors.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES ${priv_requires})
