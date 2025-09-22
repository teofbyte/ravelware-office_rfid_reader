idf_component_register(SRCS
                        "main.cpp"
                        "drivers/led-driver/led_driver.cpp"
                        "drivers/wifi-driver/wifi_driver.cpp"
                        "drivers/mqtt-driver/mqtt_driver.cpp"
                        "drivers/reader-driver/reader_driver.cpp"
                        "drivers/buzzer-driver/buzzer_driver.cpp"
                        "services/wifi-service/wifi_service.cpp"
                        "services/mqtt-service/mqtt_service.cpp"
                        "services/reader-service/reader_service.cpp"
                    INCLUDE_DIRS ".")