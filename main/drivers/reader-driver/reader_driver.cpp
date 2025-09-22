#include "reader_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdio>
#include <cstring>
#include "esp_log.h"

static const char *TAG = "readerDriver";

ReaderDriver::ReaderDriver(gpio_num_t sda, gpio_num_t scl, gpio_num_t reset, gpio_num_t irq)
    : _sda(sda), _scl(scl), _reset(reset), _irq(irq), _lastUid("")
{
}

ReaderDriver::~ReaderDriver()
{
    pn532_release(&_pn532_io);
}

esp_err_t ReaderDriver::init()
{
    esp_err_t err;

    ESP_LOGI(TAG, "Init PN532 in I2C mode");
    ESP_ERROR_CHECK(pn532_new_driver_i2c(_sda, _scl, _reset, _irq, 0, &_pn532_io));

    // Initialize PN532
    do
    {
        err = pn532_init(&_pn532_io);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to initialize PN532, retrying...");
            pn532_release(&_pn532_io);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    } while (err != ESP_OK);

    // Get firmware version
    uint32_t version_data = 0;
    do
    {
        err = pn532_get_firmware_version(&_pn532_io, &version_data);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "PN53x not found, resetting...");
            pn532_reset(&_pn532_io);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    } while (err != ESP_OK);

    ESP_LOGI(TAG, "Found chip PN5%x", (unsigned int)(version_data >> 24) & 0xFF);
    ESP_LOGI(TAG, "Firmware ver. %d.%d", (int)(version_data >> 16) & 0xFF, (int)(version_data >> 8) & 0xFF);
    ESP_LOGI(TAG, "Waiting for an Card ...");//ISO14443A 

    return ESP_OK;
}

std::string ReaderDriver::_uidToString(const uint8_t *puid, uint8_t len)
{
    char uid_str[20] = {0};
    char *ptr = uid_str;
    for (uint8_t i = 0; i < len; i++)
    {
        snprintf(ptr, 3, "%02X", puid[i]);
        ptr += 2;
    }
    return std::string(uid_str);
}

std::string ReaderDriver::readUid()
{
    uint8_t uid[7] = {0};
    uint8_t uid_length = 0;
    esp_err_t err;

    err = pn532_read_passive_target_id(&_pn532_io, PN532_BRTY_ISO14443A_106KBPS, uid, &uid_length, 100);
    if (err == ESP_OK)
    {
        std::string uid_str = _uidToString(uid, uid_length);

        if (uid_str != _lastUid)
        {
            ESP_LOGI(TAG, "Found new card UID: %s", uid_str.c_str());
            _lastUid = uid_str;
            _statusSend = true;
        }

        // InList Passive Target
        err = pn532_in_list_passive_target(&_pn532_io);
        if (err != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to inList passive target");
        }

        return uid_str;
    }
    else
    {
        // Jika tidak ada kartu
        _lastUid.clear();
        _statusSend = false;
        return "";
    }
}

bool ReaderDriver::hasNewCard()
{
    std::string uid = readUid();
    ESP_LOGI(TAG, "hasNewCard: current UID=%s, last UID=%s", uid.c_str(), _lastUid.c_str());
    if (!uid.empty() && uid != _lastUid)
    {
        _lastUid = uid; // update UID terakhir
        return true;    // artinya ada kartu baru
        _statusSend = true;
    }

    return false;
}

bool ReaderDriver::getStatusUID()
{
    return _statusSend;
}