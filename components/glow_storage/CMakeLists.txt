set(priv_requires "fatfs" "wear_levelling" "esp_partition" "nvs_flash")

idf_component_register(SRCS "glow_storage.c"
                    INCLUDE_DIRS "include"
                    EMBED_TXTFILES "default.json"
                    PRIV_REQUIRES ${priv_requires})
