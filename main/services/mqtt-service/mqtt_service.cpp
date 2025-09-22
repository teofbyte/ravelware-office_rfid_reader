#include "mqtt_service.h"
#include "esp_log.h"
#include <cstring>  // untuk strcmp, strlen

static const char *TAG = "mqttService";

MqttService::MqttService(MqttDriver &driver, LedDriver &ledc, LedDriver &ledr, BuzzerDriver &buzzer, const std::string &topic)
    : _driver(driver), _ledc(ledc), _ledr(ledr), _buzzer(buzzer), _topic(topic) {}

MqttService::~MqttService()
{
    stop();
}

void MqttService::start()
{
    if (running)
        return;
    running = true;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        MQTT_APP_EVENTS, MQTT_APP_CONNECTED,
        &MqttService::_handleConnected, this, NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        MQTT_APP_EVENTS, MQTT_APP_DISCONNECTED,
        &MqttService::_handleDisconnected, this, NULL));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        MQTT_APP_EVENTS, MQTT_APP_DATA,
        &MqttService::_handleData, this, NULL));

    ESP_LOGI(TAG, "🚀 MQTT Service started, waiting for events...");
}

void MqttService::stop()
{
    if (!running)
        return;
    running = false;

    esp_event_handler_unregister(MQTT_APP_EVENTS,
                                 MQTT_APP_CONNECTED,
                                 &MqttService::_handleConnected);
    esp_event_handler_unregister(MQTT_APP_EVENTS,
                                 MQTT_APP_DISCONNECTED,
                                 &MqttService::_handleDisconnected);
    esp_event_handler_unregister(MQTT_APP_EVENTS,
                                 MQTT_APP_DATA,
                                 &MqttService::_handleData);

    ESP_LOGI(TAG, "🛑 MQTT Service stopped");
}

// ✅ CONNECTED
void MqttService::_handleConnected(void *handler_args,
                                  esp_event_base_t base,
                                  int32_t event_id,
                                  void *event_data)
{
    auto *service = static_cast<MqttService *>(handler_args);

    service->_ledc.stopBlink();
    service->_ledc.onLed();

    ESP_LOGI(TAG, "✅ MQTT connected (service)");

    if (!service->_topic.empty())
    {
        service->_driver.subscribe(service->_topic, 1);
        ESP_LOGI(TAG, "📡 Subscribed to topic: %s", service->_topic.c_str());
    }
}

// ✅ DISCONNECTED
void MqttService::_handleDisconnected(void *handler_args,
                                     esp_event_base_t base,
                                     int32_t event_id,
                                     void *event_data)
{
    auto *service = static_cast<MqttService *>(handler_args);
    ESP_LOGW(TAG, "⚠️ MQTT disconnected (service)");

    service->_ledc.startBlink(500);
}

// ✅ DATA
void MqttService::_handleData(void *handler_args,
                             esp_event_base_t base,
                             int32_t event_id,
                             void *event_data)
{
    auto *service = static_cast<MqttService *>(handler_args);
    auto *evt = static_cast<mqtt_app_data_event_t *>(event_data);

    if (strlen(evt->topic) == 0 || strlen(evt->data) == 0)
    {
        ESP_LOGD(TAG, "📭 Data kosong diabaikan");
        return;
    }

    ESP_LOGI(TAG, "📥 Data received | topic=%s | data=%s",
             evt->topic, evt->data);

    if (strcmp(evt->topic, service->_topic.c_str()) == 0)
    {
        if (strcmp(evt->data, "buka") == 0)
        {
            service->_ledr.stopBlink();
            service->_ledr.startBlink(500);
            ESP_LOGI(TAG, "🚪 Door Open");
        }
        else if (strcmp(evt->data, "invalid") == 0)
        {
            service->_ledr.stopBlink();
            service->_ledr.startBlink(1000);
            ESP_LOGI(TAG, "⛔ Invalid UID");
        }
        else if (strcmp(evt->data, "daftar") == 0)
        {
            // service->_buzzer.stopBuzzer();
            service->_buzzer.startBuzzer(1000);
            // service->_buzzer.stopBuzzer();
            ESP_LOGI(TAG, "📝 Enrollment Mode");
        }
        else if (strcmp(evt->data, "selesai") == 0)
        {
            // service->_buzzer.stopBuzzer();
            service->_buzzer.startBuzzer(1000);
            ESP_LOGI(TAG, "✅ Finish Enrollment");
        }
    }
}