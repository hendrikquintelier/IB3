idf_component_register(SRCS "station_example_main.c" "ping.c" "http_server.c" "mpu.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_netif esp_wifi nvs_flash esp_http_server lwip driver esp_timer)
