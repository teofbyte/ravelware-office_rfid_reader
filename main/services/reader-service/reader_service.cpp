#include "reader_service.h"

static const char *TAG = "readerService";

// Konstruktor, inisialisasi driver
ReaderService::ReaderService(ReaderDriver &uid, LedDriver &led, MqttDriver &mqtt, const std::string &topic)
    : _uid_driver(uid), _led_driver(led), _mqtt_driver(mqtt), _reader_task_handle(nullptr), _topic(topic) // _publishStatus(false),
{
}

// Task FreeRTOS
void ReaderService::_readerTask(void *pvParameters)
{
    ReaderService *reader_service = static_cast<ReaderService *>(pvParameters);

     if (reader_service->_uid_driver.init() == ESP_OK)
    {
        while (true)
        {
            std::string uid = reader_service->_uid_driver.readUid();

            if (!uid.empty())
            {
                // Jika UID baru dan belum pernah dipublish
                if (reader_service->_uid != uid)
                {
                    // ESP_LOGI(TAG, "UID Baru: %s", uid.c_str());

                    cJSON *root = cJSON_CreateObject();
                    cJSON_AddStringToObject(root, "uid", uid.c_str());

                    char *jsonStr = cJSON_PrintUnformatted(root);
                    if (jsonStr != nullptr)
                    {
                        reader_service->_mqtt_driver.publish(reader_service->_topic, jsonStr);

                        reader_service->_led_driver.startBlink(1000);
                        ESP_LOGI(TAG, "Blink LED Reader ON");
                        reader_service->_uid = uid;   // simpan UID terakhir
                        free(jsonStr);
                    }
                    cJSON_Delete(root);
                }
                else
                {
                    // UID sama, jangan publish lagi
                    reader_service->_led_driver.stopBlink();
                }
            }
            else
            {
                // Tidak ada kartu
                reader_service->_uid.clear();
                reader_service->_led_driver.stopBlink();
            }

            vTaskDelay(pdMS_TO_TICKS(100)); // polling interval
        }
    }

    vTaskDelete(NULL); // kalau init gagal
}

void ReaderService::start()
{
    if (_reader_task_handle == nullptr)
    {
        xTaskCreatePinnedToCore(
            _readerTask,
            "ReaderTask",
            4096,
            this,
            5,
            &_reader_task_handle,
            1);
    }
}