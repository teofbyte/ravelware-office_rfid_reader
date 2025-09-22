#ifndef MQTT_SERVICE_H
#define MQTT_SERVICE_H

#include "drivers/led-driver/led_driver.h"
#include "drivers/mqtt-driver/mqtt_driver.h"
#include "drivers/buzzer-driver/buzzer_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include <string>

class MqttService
{
public:
    MqttService(MqttDriver &driver, LedDriver &ledc, LedDriver &ledr, BuzzerDriver &buzzer, const std::string &topic);
    ~MqttService();

    void start();
    void stop();

private:
    MqttDriver &_driver;
    LedDriver &_ledc;
    LedDriver &_ledr;
    BuzzerDriver &_buzzer;
    std::string _topic;
    bool running = false;

    // Event Handlers
    static void _handleConnected(void *handler_args,
                                esp_event_base_t base,
                                int32_t event_id,
                                void *event_data);

    static void _handleDisconnected(void *handler_args,
                                   esp_event_base_t base,
                                   int32_t event_id,
                                   void *event_data);

    static void _handleData(void *handler_args,
                           esp_event_base_t base,
                           int32_t event_id,
                           void *event_data);
};

#endif // MQTT_SERVICE_H
