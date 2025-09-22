#ifndef MQTT_DRIVER_H
#define MQTT_DRIVER_H

#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_event.h"
#include <string>
#include <map>
#include <vector>
#include <mutex>

enum {
    MQTT_APP_CONNECTED,
    MQTT_APP_DISCONNECTED,
    MQTT_APP_DATA
};

typedef struct {
    char topic[128];
    char data[256];
} mqtt_app_data_event_t;

ESP_EVENT_DECLARE_BASE(MQTT_APP_EVENTS);

class MqttDriver {
public:
    MqttDriver(const std::string &brokerUri,
               const std::string &username = "",
               const std::string &password = "");
    ~MqttDriver();

    void start(const std::string &lwtTopic = "");
    void stop();

    void subscribe(const std::string &topic, int qos = 0);
    void publish(const std::string &topic, const std::string &message,
                 int qos = 0, bool retain = false);

private:
    static void _mqttEventHandler(void *handler_args,
                                 esp_event_base_t base,
                                 int32_t event_id,
                                 void *event_data);

    esp_mqtt_client_handle_t _client;
    std::string _brokerUri, _username, _password, _lwtTopic;
    mutable std::mutex _connMutex;

    bool _isConnected = false;
    std::map<std::string, std::string> _topicMessages;
    std::vector<std::pair<std::string,int>> _subscribedTopics;

    void _reSubscribeAll();  // helper untuk re-subscribe
};

#endif // MQTT_DRIVER_H