#include <stdio.h>
#include <stdlib.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdkconfig.h"
#include "pn532_driver_i2c.h"
#include "pn532_driver_hsu.h"
#include "pn532_driver_spi.h"
#include "pn532.h"


// select ONLY ONE interface for the PN532
#define PN532_MODE_I2C 0
#define PN532_MODE_HSU 0
#define PN532_MODE_SPI 1

#if PN532_MODE_I2C

// I2C mode needs only SDA, SCL and IRQ pins. RESET pin will be used if valid.
// IRQ pin can be used in polling mode or in interrupt mode. Use menuconfig to select mode.
#define SCL_PIN    (0)
#define SDA_PIN    (1)
#define RESET_PIN  (-1)
#define IRQ_PIN    (3)

#elif PN532_MODE_HSU

// HSU mode needs only RX/TX pins. RESET pin will be used if valid.
#define RESET_PIN      (-1)
#define IRQ_PIN        (-1)
#define HSU_HOST_RX    (4)
#define HSU_HOST_TX    (5)
#define HSU_UART_PORT  UART_NUM_1
#define HSU_BAUD_RATE  (921600)

#elif PN532_MODE_SPI

// SPI mode needs only CS, SCK, MISO and MOSI pins.
// If at least one of CS, SCK, MISO and MOSI is set to -1 the SPI host must be configured outside the library.
// IRQ pin can be used in polling mode or in interrupt mode. Use menuconfig to select mode.
#define RESET_PIN      (-1)
#define IRQ_PIN        (6)
//#define IRQ_PIN        (-1)
#define SPI_CS         (5)
#define SPI_SCK        (2)
#define SPI_MISO       (3)
#define SPI_MOSI       (4)
#define SPI_HOST_NFC   (SPI3_HOST)
#define SPI_CLOCKRATE  (1000000)

#endif

static const char *TAG = "ntag_read";

void app_main()
{
    pn532_io_t pn532_io;
    esp_err_t err;

    printf("APP MAIN\n");

#if 0
    // Enable DEBUG logging
    esp_log_level_set("PN532", ESP_LOG_DEBUG);
    esp_log_level_set("pn532_driver", ESP_LOG_DEBUG);
    esp_log_level_set("pn532_driver_i2c", ESP_LOG_DEBUG);
    esp_log_level_set("i2c.master", ESP_LOG_DEBUG);
    esp_log_level_set("pn532_driver_hsu", ESP_LOG_DEBUG);
    esp_log_level_set("pn532_driver_spi", ESP_LOG_DEBUG);
    esp_log_level_set("spi", ESP_LOG_DEBUG);
#endif

    vTaskDelay(1000 / portTICK_PERIOD_MS);

#if PN532_MODE_I2C

    ESP_LOGI(TAG, "init PN532 in I2C mode");
    ESP_ERROR_CHECK(pn532_new_driver_i2c(SDA_PIN, SCL_PIN, RESET_PIN, IRQ_PIN, 0, &pn532_io));

#elif PN532_MODE_HSU

    ESP_LOGI(TAG, "init PN532 in HSU mode");
    ESP_ERROR_CHECK(pn532_new_driver_hsu(HSU_HOST_RX,
                                         HSU_HOST_TX,
                                         RESET_PIN,
                                         IRQ_PIN,
                                         HSU_UART_PORT,
                                         HSU_BAUD_RATE,
                                         &pn532_io));

#elif PN532_MODE_SPI

    ESP_LOGI(TAG, "init PN532 in SPI mode");
    ESP_ERROR_CHECK(pn532_new_driver_spi(
        SPI_MISO,
        SPI_MOSI,
        SPI_SCK,
        SPI_CS,
        -1,
        IRQ_PIN,
        SPI_HOST_NFC,
        SPI_CLOCKRATE,
        &pn532_io));

#endif

    do {
        err = pn532_init(&pn532_io);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "failed to initialize PN532");
            pn532_release(&pn532_io);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    } while(err != ESP_OK);

    ESP_LOGI(TAG, "get firmware version");
    uint32_t version_data = 0;
    do {
        err = pn532_get_firmware_version(&pn532_io, &version_data);
        if (ESP_OK != err) {
            ESP_LOGI(TAG, "Didn't find PN53x board");
            pn532_reset(&pn532_io);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    } while (ESP_OK != err);

    // Log firmware infos
    ESP_LOGI(TAG, "Found chip PN5%x", (unsigned int)(version_data >> 24) & 0xFF);
    ESP_LOGI(TAG, "Firmware ver. %d.%d", (int)(version_data >> 16) & 0xFF, (int)(version_data >> 8) & 0xFF);

    ESP_LOGI(TAG, "Waiting for an ISO14443A Card ...");
    while (1)
    {
        uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0}; // Buffer to store the returned UID
        uint8_t uid_length;                     // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

        // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
        // 'uid' will be populated with the UID, and uid_length will indicate
        // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
        err = pn532_read_passive_target_id(&pn532_io, PN532_BRTY_ISO14443A_106KBPS, uid, &uid_length, 0);

        if (ESP_OK == err)
        {
            // Display some basic information about the card
            ESP_LOGI(TAG, "\nFound an ISO14443A card");
            ESP_LOGI(TAG, "UID Length: %d bytes", uid_length);
            ESP_LOGI(TAG, "UID Value:");
            ESP_LOG_BUFFER_HEX_LEVEL(TAG, uid, uid_length, ESP_LOG_INFO);

            err = pn532_in_list_passive_target(&pn532_io);
            if (err != ESP_OK) {
                ESP_LOGI(TAG, "Failed to inList passive target");
                continue;
            }

            NTAG2XX_MODEL ntag_model = NTAG2XX_UNKNOWN;
            err = ntag2xx_get_model(&pn532_io, &ntag_model);
            if (err != ESP_OK)
                continue;

            int page_max;
            switch (ntag_model) {
                case NTAG2XX_NTAG213:
                    page_max = 45;
                    ESP_LOGI(TAG, "found NTAG213 target (or maybe NTAG203)");
                    break;

                case NTAG2XX_NTAG215:
                    page_max = 135;
                    ESP_LOGI(TAG, "found NTAG215 target");
                    break;

                case NTAG2XX_NTAG216:
                    page_max = 231;
                    ESP_LOGI(TAG, "found NTAG216 target");
                    break;

                default:
                    ESP_LOGI(TAG, "Found unknown NTAG target!");
                    continue;
            }

            for(int page=0; page < page_max; page+=4) {
                uint8_t buf[16];
                err = ntag2xx_read_page(&pn532_io, page, buf, 16);
                if (err == ESP_OK) {
                    ESP_LOG_BUFFER_HEXDUMP(TAG, buf, 16, ESP_LOG_INFO);
                }
                else {
                    ESP_LOGI(TAG, "Failed to read page %d", page);
                    break;
                }
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}
