set(priv_requires "glow_context" "glow_storage" "glow_power" "glow_led" "glow_sensors" "esp_timer")
idf_component_register(SRCS "glow_core.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES ${priv_requires})
