#ifndef READER_DRIVER_H
#define READER_DRIVER_H

#include "pn532_driver_i2c.h"
#include "pn532.h"
#include "esp_err.h"
#include "esp_log.h"
#include <string>

class ReaderDriver
{
public:
    ReaderDriver(gpio_num_t sda, gpio_num_t scl, gpio_num_t reset = GPIO_NUM_NC, gpio_num_t irq = GPIO_NUM_NC);
    ~ReaderDriver();

    esp_err_t init();
    std::string readUid();
    bool hasNewCard();
    bool getStatusUID();

private:
    pn532_io_t _pn532_io;
    gpio_num_t _sda, _scl, _reset, _irq;
    std::string _lastUid;
    bool _statusSend = false;
    std::string _uidToString(const uint8_t *puid, uint8_t len);
};

#endif // UID_DRIVER_H
