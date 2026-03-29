set(priv_requires "glow_context" "glow_storage" "esp_wifi" "esp_event" "esp_netif" "esp_http_server" "nvs_flash" "lwip")

idf_component_register(SRCS "glow_web.c"
                    INCLUDE_DIRS "include"
                    EMBED_TXTFILES "web/index.html"
                    PRIV_REQUIRES ${priv_requires})
