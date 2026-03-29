set(priv_requires "fatfs" "wear_levelling" "esp_partition")

idf_component_register(SRCS "glow_storage.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES ${priv_requires})
