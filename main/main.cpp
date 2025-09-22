#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "drivers/wifi-driver/wifi_driver.h"
#include "drivers/mqtt-driver/mqtt_driver.h"
#include "drivers/led-driver/led_driver.h"
#include "drivers/buzzer-driver/buzzer_driver.h"
#include "drivers/reader-driver/reader_driver.h"
#include "services/wifi-service/wifi_service.h"
#include "services/mqtt-service/mqtt_service.h"
#include "services/reader-service/reader_service.h"

#define WIFI_SSID "TAF"
#define WIFI_PASS "semangatsemangatokok"
#define PIN_ETH GPIO_NUM_26
#define PIN_READER GPIO_NUM_33
#define PIN_RUN GPIO_NUM_32
#define PIN_BUZZER GPIO_NUM_13
#define BROKER "mqtt://192.168.137.1"
#define USERNAME ""
#define PASSWORD ""
#define TOPIC_PUB "RFID_tapping/scan"
#define TOPIC_SUB "RFID_tapping/feedback"
#define TOPIC_STATUS "RFID_tapping/status"
#define SDA_PIN GPIO_NUM_16
#define SCL_PIN GPIO_NUM_17
#define RESET_PIN GPIO_NUM_NC
#define IRQ_PIN GPIO_NUM_NC

static const char *TAG = "main";

// In main.cpp, outside app_main()
static LedDriver ledDriverWifi(PIN_ETH);
static LedDriver ledDriverMqtt(PIN_READER);
static LedDriver ledDriverReader(PIN_RUN);
static WifiDriver wifiDriver;
static MqttDriver mqtt(BROKER, USERNAME, PASSWORD);
static ReaderDriver uidDriver(SDA_PIN, SCL_PIN, RESET_PIN, IRQ_PIN);
static WifiService wifiService(wifiDriver, ledDriverWifi);
static ReaderService readerService(uidDriver, ledDriverReader, mqtt, TOPIC_PUB);
static BuzzerDriver buzzerDriver(PIN_BUZZER);
static MqttService mqttService(mqtt, ledDriverMqtt, ledDriverReader, buzzerDriver, TOPIC_SUB);

extern "C" void app_main(void)
{
    // In app_main(), initialize and start them
    esp_event_loop_create_default();
    wifiDriver.initWifiDriver(WIFI_SSID, WIFI_PASS);
    wifiService.start();
    mqtt.start(TOPIC_STATUS);
    mqttService.start();
    mqtt.subscribe(TOPIC_SUB, 0);
    readerService.start();

    ESP_LOGI(TAG, "🚀 System started");

     // vTaskDelete(NULL);
    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}