set(priv_requires "esp_driver_spi" "esp_driver_gpio" "esp_adc")
idf_component_register(SRCS "glow_power.c"
                    INCLUDE_DIRS "include"
                    REQUIRES ${priv_requires})
