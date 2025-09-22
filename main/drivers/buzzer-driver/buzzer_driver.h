#ifndef BUZZER_DRIVER_H
#define BUZZER_DRIVER_H

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>

class BuzzerDriver
{
public:
    explicit BuzzerDriver(gpio_num_t gpio);
    ~BuzzerDriver();                 // destructor untuk cleanup

    void onBuzzer();                 // hidupkan buzzer langsung
    void offBuzzer();                // matikan buzzer langsung
    void startBuzzer(int delay_ms);  // nyalakan buzzer dengan durasi tertentu (ms)
    void stopBuzzer();               // hentikan task buzzer jika masih ada

private:
    gpio_num_t _pin;
    std::atomic_bool _buzzerState;    // aman untuk akses dari beberapa task
    TaskHandle_t _buzzerTaskHandle;
    int _buzzerDurationMs;            // simpan durasi bunyi

    static void _buzzerTask(void *param);
};

#endif
