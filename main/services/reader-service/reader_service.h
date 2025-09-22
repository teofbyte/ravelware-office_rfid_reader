#ifndef READER_SERVICE_H
#define READER_SERVICE_H

#include "drivers/reader-driver/reader_driver.h"
#include "drivers/mqtt-driver/mqtt_driver.h"
#include "drivers/led-driver/led_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string>
#include "cJSON.h"

class ReaderService
{
public:
    ReaderService(ReaderDriver &uid, LedDriver &led, MqttDriver &mqtt, const std::string &topic);
    void start(); // untuk membuat task

private:
    ReaderDriver &_uid_driver;
    LedDriver &_led_driver;
    MqttDriver &_mqtt_driver;

    TaskHandle_t _reader_task_handle;
    // bool _publishStatus;
    std::string _uid = "";
    std::string _topic; 

    static void _readerTask(void *pvParameters);
};

#endif