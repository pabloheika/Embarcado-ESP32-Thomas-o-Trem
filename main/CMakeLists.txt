idf_component_register(SRCS "main.c"
                            "wifi/wifi_station.c"
                            "web_api/web_api.c"
                            "mpu6050/mpu6050.c"
                            "qtr8rc/qtr8rc.c"
                            "motor_driver/motor_driver.c"
                            "pid/pid_controller.c"
                            "line_follower/line_follower.c"
                            "mode_manager/mode_manager.c"
                            "sys_logger/sys_logger.c"
                            "test_modules.c"
                    INCLUDE_DIRS "." "wifi" "web_api" "mpu6050" "qtr8rc"
                                 "motor_driver" "pid" "line_follower"
                                 "mode_manager" "sys_logger"
                    PRIV_REQUIRES esp_wifi
                                  esp_netif
                                  nvs_flash
                                  esp_event
                                  esp_http_server
                                  esp_timer
                                  cjson
                                  esp_driver_gpio
                                  esp_adc
                                  esp_driver_ledc
                                  esp_driver_i2c
                                  unity)
