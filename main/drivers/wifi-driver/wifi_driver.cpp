#include "wifi_driver.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "string.h"

static const char *TAG = "WifiDriver";

WifiDriver::WifiDriver() : _instance_any_id(nullptr), _instance_got_ip(nullptr) {}

esp_err_t WifiDriver::initWifiDriver(const char* ssid, const char* password) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi
    ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, 
                                                      ESP_EVENT_ANY_ID, 
                                                      &_wifi_event_handler, 
                                                      this, 
                                                      &_instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, 
                                                      IP_EVENT_STA_GOT_IP, 
                                                      &_wifi_event_handler, 
                                                      this, 
                                                      &_instance_got_ip));

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    return ESP_OK;
}

void WifiDriver::_wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    WifiDriver* wifi = static_cast<WifiDriver*>(arg);
    
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi->_connected = false;
        // _connected = false;
        esp_wifi_connect();
        ESP_LOGI(TAG, "retry to connect to the AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        wifi->_connected = true;
        // _connected = true;
        ESP_LOGI(TAG, "WiFi Connected");
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    }
}

bool WifiDriver::isconnectedWifi()
{
    return _connected;
}

WifiDriver::~WifiDriver() {
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, _instance_any_id);
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, _instance_got_ip);
    esp_wifi_stop();
    esp_wifi_deinit();
}