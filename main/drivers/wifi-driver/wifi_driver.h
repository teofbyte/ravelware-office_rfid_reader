#ifndef WIFI_DRIVER_H
#define WIFI_DRIVER_H

#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/event_groups.h"

class WifiDriver
{
public:
    WifiDriver();
    esp_err_t initWifiDriver(const char *ssid, const char *password);
    bool isconnectedWifi();

    ~WifiDriver();

private:
    bool _connected = false; // status koneksi
    esp_event_handler_instance_t _instance_any_id;
    esp_event_handler_instance_t _instance_got_ip;
    static void _wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
};

#endif