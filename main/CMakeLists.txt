idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES glow_led glow_power glow_web glow_context glow_storage glow_sensors nvs_flash esp_event)
