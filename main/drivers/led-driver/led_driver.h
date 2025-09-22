#ifndef LED_DRIVER_H
#define LED_DRIVER_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class LedDriver
{
public:
    LedDriver(gpio_num_t gpio);
    ~LedDriver();
    void onLed();
    void offLed();
    void startBlink(int delay_ms); // mulai blink (buat task)
    void stopBlink();              // stop blink (hapus task)

private:
    gpio_num_t _pin;
    bool _ledState;
    TaskHandle_t _blinkTaskHandle;

    static void _blinkTask(void *param); // static agar bisa dipanggil dari FreeRTOS
};

#endif
