#include "led_driver.h"

LedDriver::LedDriver(gpio_num_t gpio) : _pin(gpio), _ledState(false), _blinkTaskHandle(nullptr)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << _pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    offLed();
}

LedDriver::~LedDriver(){
    stopBlink();
}

void LedDriver::onLed()
{
    gpio_set_level(_pin, 0);   // aktif low
    _ledState = true;
}

void LedDriver::offLed()
{
    gpio_set_level(_pin, 1);   // aktif low
    _ledState = false;
}

void LedDriver::startBlink(int delay_ms)
{
    if (_blinkTaskHandle == nullptr) {
        // buat task blink
        xTaskCreate(
            LedDriver::_blinkTask,   // fungsi task
            "BlinkTask",            // nama task
            2048,                   // stack
            this,                   // argumen → this (objek LED)
            1,                      // prioritas
            &_blinkTaskHandle        // handle
        );
    }
}

void LedDriver::stopBlink()
{
    if (_blinkTaskHandle != nullptr) {
        vTaskDelete(_blinkTaskHandle);
        _blinkTaskHandle = nullptr;
        offLed(); // pastikan mati setelah berhenti
    }
}

void LedDriver::_blinkTask(void *param)
{
    LedDriver *led = static_cast<LedDriver *>(param);
    const TickType_t delayTicks = pdMS_TO_TICKS(500); // default 500ms

    while (true) {
        if (led->_ledState)
            led->offLed();
        else
            led->onLed();

        vTaskDelay(delayTicks);
    }
}
