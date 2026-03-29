set(priv_requires "driver" "led_strip")
idf_component_register(SRCS "glow_led.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES ${priv_requires})
