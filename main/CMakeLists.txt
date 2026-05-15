idf_component_register(SRCS "main.c"
                            "wifi/wifi_station.c"
                            "web_api/web_api.c"
                            "mpu6050/mpu6050.c"
                    INCLUDE_DIRS "." "wifi" "web_api" "mpu6050"
                    PRIV_REQUIRES esp_wifi
                                  esp_netif
                                  nvs_flash
                                  esp_event
                                  esp_http_server
                                  esp_timer
                                  cjson
                                  driver)
