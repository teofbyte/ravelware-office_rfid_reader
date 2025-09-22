#ifndef WIFI_SERVICE_H
#define WIFI_SERVICE_H

#include "drivers/led-driver/led_driver.h"
#include "drivers/wifi-driver/wifi_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"

class WifiService
{
public:
    WifiService(WifiDriver& wifi, LedDriver& led);
    void start();
    void stop();
    void wifiIndicatorStatus();

private:
    WifiDriver& _wifi_driver;
    LedDriver& _led_driver;
    TaskHandle_t _wifi_task_handle;

    static void _wifiTask(void* pvParameters);
};

#endif