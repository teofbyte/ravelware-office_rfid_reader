#include "wifi_service.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"

static const char *TAG = "wifiService";

WifiService::WifiService(WifiDriver &wifi, LedDriver &led)
    : _wifi_driver(wifi), _led_driver(led), _wifi_task_handle(nullptr) {}

void WifiService::wifiIndicatorStatus()
{
    if (_wifi_driver.isconnectedWifi())
    {
        _led_driver.onLed();
        _led_driver.stopBlink();
    }
    else
    {
        _led_driver.startBlink(500); // LED blinks slowly when not connected
    }
}

void WifiService::_wifiTask(void* pvParameters)
{
    WifiService* wifi_service = static_cast<WifiService*>(pvParameters);
    
    while (true)
    {
        wifi_service->wifiIndicatorStatus();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every 1 second
    }
}

void WifiService::start()
{
    if (_wifi_task_handle == nullptr)
    {
        xTaskCreatePinnedToCore(
            _wifiTask,               // Task function
            "WifiServiceTask",      // Task name
            2048,                   // Stack size
            this,                   // Task parameter (pointer to this instance)
            1,                      // Task priority
            &_wifi_task_handle,      // Task handle
            0                       // Core ID (Core 1)
        );
        ESP_LOGI(TAG, "WiFi service task started on Core 0");
    }
}

void WifiService::stop()
{
    if (_wifi_task_handle != nullptr)
    {
        vTaskDelete(_wifi_task_handle);
        _wifi_task_handle = nullptr;
        ESP_LOGI(TAG, "WiFi service task stopped");
    }
}