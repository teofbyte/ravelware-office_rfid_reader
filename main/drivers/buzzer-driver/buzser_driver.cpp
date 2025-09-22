#include "buzzer_driver.h"

BuzzerDriver::BuzzerDriver(gpio_num_t gpio) 
    : _pin(gpio), _buzzerState(false), _buzzerTaskHandle(nullptr), _buzzerDurationMs(0)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << _pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    offBuzzer();
}

BuzzerDriver::~BuzzerDriver()
{
    // Pastikan task dihentikan sebelum objek dihancurkan
    stopBuzzer();
}

void BuzzerDriver::onBuzzer()
{
    gpio_set_level(_pin, 1); 
    _buzzerState.store(true, std::memory_order_relaxed);
}

void BuzzerDriver::offBuzzer()
{
    gpio_set_level(_pin, 0);
    _buzzerState.store(false, std::memory_order_relaxed);
}

void BuzzerDriver::startBuzzer(int delay_ms)
{
    if (_buzzerTaskHandle == nullptr)
    {
        _buzzerDurationMs = delay_ms; // simpan durasi untuk task
        xTaskCreatePinnedToCore(
            BuzzerDriver::_buzzerTask,
            "BuzzerTask",
            2048,
            this,                     // kirim pointer objek
            1,
            &_buzzerTaskHandle,
            1);                       // jalankan di core 1
    }
}

void BuzzerDriver::stopBuzzer()
{
    if (_buzzerTaskHandle != nullptr)
    {
        vTaskDelete(_buzzerTaskHandle);
        _buzzerTaskHandle = nullptr;
        offBuzzer();
    }
}

void BuzzerDriver::_buzzerTask(void *param)
{
    BuzzerDriver *buzzer = static_cast<BuzzerDriver *>(param);

    buzzer->onBuzzer();
    vTaskDelay(pdMS_TO_TICKS(buzzer->_buzzerDurationMs)); // gunakan delay_ms yang dikirim
    buzzer->offBuzzer();

    buzzer->_buzzerTaskHandle = nullptr; 
    vTaskDelete(NULL); // hapus task dirinya sendiri
}
