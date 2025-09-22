#include "mqtt_driver.h"

static const char *TAG = "MqttDriver";

ESP_EVENT_DEFINE_BASE(MQTT_APP_EVENTS);

MqttDriver::MqttDriver(const std::string &brokerUri,
                       const std::string &username,
                       const std::string &password)
    : _client(nullptr), _brokerUri(brokerUri),
      _username(username), _password(password) {}

MqttDriver::~MqttDriver()
{
    stop();
}

void MqttDriver::start(const std::string &lwtTopic)
{
    this->_lwtTopic = lwtTopic;

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = _brokerUri.c_str();

    if (!_username.empty() && !_password.empty())
    {
        mqtt_cfg.credentials.username = _username.c_str();
        mqtt_cfg.credentials.authentication.password = _password.c_str();
    }

    if (!lwtTopic.empty())
    {
        mqtt_cfg.session.last_will.topic = lwtTopic.c_str();
        mqtt_cfg.session.last_will.msg = "offline";
        mqtt_cfg.session.last_will.qos = 1;
        mqtt_cfg.session.last_will.retain = true;
    }

    mqtt_cfg.session.keepalive = 60;
    mqtt_cfg.network.reconnect_timeout_ms = 2000;

    _client = esp_mqtt_client_init(&mqtt_cfg);
    if (!_client)
    {
        ESP_LOGE(TAG, "❌ Failed to init MQTT client");
        return;
    }

    esp_mqtt_client_register_event(_client,
                                   (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID,
                                   _mqttEventHandler,
                                   this);

    esp_err_t err = esp_mqtt_client_start(_client);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "❌ Failed to start MQTT client: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "🚀 MQTT client started. Broker: %s", _brokerUri.c_str());
    }
}

void MqttDriver::stop()
{
    if (_client)
    {
        if (!_lwtTopic.empty())
        {
            publish(_lwtTopic, "offline", 1, true);
        }
        esp_mqtt_client_stop(_client);
        esp_mqtt_client_destroy(_client);
        _client = nullptr;
        {
            std::lock_guard<std::mutex> lock(_connMutex);
            _isConnected = false;
        }
        ESP_LOGI(TAG, "🛑 MQTT client stopped.");
    }
}

void MqttDriver::subscribe(const std::string &topic, int qos)
{
    if (_client)
    {
        int msg_id = esp_mqtt_client_subscribe(_client, topic.c_str(), qos);
        if (msg_id >= 0)
        {
            _subscribedTopics.push_back({topic, qos});
            ESP_LOGI(TAG, "✅ Subscribed to %s, msg_id=%d", topic.c_str(), msg_id);
        }
        else
        {
            ESP_LOGE(TAG, "❌ Failed to subscribe to %s", topic.c_str());
        }
    }
}

void MqttDriver::publish(const std::string &topic,
                         const std::string &message,
                         int qos, bool retain)
{
    std::lock_guard<std::mutex> lock(_connMutex);
    if (_client && _isConnected)
    {
        int msg_id = esp_mqtt_client_publish(_client, topic.c_str(),
                                             message.c_str(), 0,
                                             qos, retain);
        if (msg_id >= 0)
        {
            ESP_LOGI(TAG, "📤 Published to %s, msg_id=%d, message=%s",
                     topic.c_str(), msg_id, message.c_str());
        }
        else
        {
            ESP_LOGE(TAG, "❌ Failed to publish to %s", topic.c_str());
        }
    }
    else
    {
        ESP_LOGW(TAG, "⚠️ Cannot publish, MQTT not connected");
    }
}

void MqttDriver::_reSubscribeAll()
{
    for (const auto &t : _subscribedTopics)
    {
        int msg_id = esp_mqtt_client_subscribe(_client, t.first.c_str(), t.second);
        if (msg_id >= 0)
        {
            ESP_LOGI(TAG, "🔁 Re-subscribed to %s", t.first.c_str());
        }
    }
}

void MqttDriver::_mqttEventHandler(void *handler_args,
                                  esp_event_base_t base,
                                  int32_t event_id,
                                  void *event_data)
{
    MqttDriver *driver = static_cast<MqttDriver *>(handler_args);
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
    {
        {
            std::lock_guard<std::mutex> lock(driver->_connMutex);
            driver->_isConnected = true;
        }
        ESP_LOGI(TAG, "✅ MQTT Connected");

        if (!driver->_lwtTopic.empty())
        {
            driver->publish(driver->_lwtTopic, "online", 1, true);
        }

        driver->_reSubscribeAll();

        esp_event_post(MQTT_APP_EVENTS, MQTT_APP_CONNECTED, NULL, 0, portMAX_DELAY);
        break;
    }

    case MQTT_EVENT_DISCONNECTED:
    {
        std::lock_guard<std::mutex> lock(driver->_connMutex);
        driver->_isConnected = false;
    }
        ESP_LOGW(TAG, "⚠️ MQTT Disconnected");
        esp_event_post(MQTT_APP_EVENTS, MQTT_APP_DISCONNECTED, NULL, 0, portMAX_DELAY);
        break;

    case MQTT_EVENT_DATA:
    {
        if (event->topic_len > 0 && event->data_len > 0)
        {
            mqtt_app_data_event_t evt_data = {};
            snprintf(evt_data.topic, sizeof(evt_data.topic), "%.*s", event->topic_len, event->topic);
            snprintf(evt_data.data, sizeof(evt_data.data), "%.*s", event->data_len, event->data);

            driver->_topicMessages[evt_data.topic] = evt_data.data;

            ESP_LOGI(TAG, "📩 DATA RECEIVED: topic=%s, data=%s", evt_data.topic, evt_data.data);

            esp_event_post(MQTT_APP_EVENTS, MQTT_APP_DATA, &evt_data, sizeof(evt_data), portMAX_DELAY);
            break;
        }
        break;
    }

    default:
        ESP_LOGD(TAG, "Unhandled MQTT event: %d", event->event_id);
        break;
    }
}
