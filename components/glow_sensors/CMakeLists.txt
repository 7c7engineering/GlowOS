set(priv_requires "glow_context" "esp_driver_tsens" "esp_adc" "esp_driver_gpio" "esp_driver_rmt" "esp_timer")
idf_component_register(SRCS "glow_sensors.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES ${priv_requires})
