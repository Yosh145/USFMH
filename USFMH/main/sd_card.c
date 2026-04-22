#include "sd_card.h"
#include <string.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"

static const char *TAG = "sd_card";

// SPI pin assignment — plain GPIOs via GPIO matrix
#define SD_PIN_MOSI  GPIO_NUM_6
#define SD_PIN_MISO  GPIO_NUM_8
#define SD_PIN_CLK   GPIO_NUM_7
#define SD_PIN_CS    GPIO_NUM_15

static sdmmc_card_t *s_card = NULL;

esp_err_t sd_card_init(void)
{
    ESP_LOGI(TAG, "Initializing SD card (SPI mode)");
    ESP_LOGI(TAG, "  MOSI=%d  MISO=%d  CLK=%d  CS=%d",
             SD_PIN_MOSI, SD_PIN_MISO, SD_PIN_CLK, SD_PIN_CS);

    // Let the SD card module power up fully
    vTaskDelay(pdMS_TO_TICKS(200));

    // Enable internal pull-ups on all SPI lines
    gpio_set_pull_mode(SD_PIN_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SD_PIN_MISO, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SD_PIN_CLK,  GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(SD_PIN_CS,   GPIO_PULLUP_ONLY);

    // Host config — use SPI2_HOST, slow clock for reliable init
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = 400;

    // SPI bus config
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_PIN_MOSI,
        .miso_io_num = SD_PIN_MISO,
        .sclk_io_num = SD_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };

    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Slot config with CS pin
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_PIN_CS;
    slot_config.host_id = host.slot;

    // Mount FAT filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024,
    };

    // Retry mount
    for (int attempt = 0; attempt < 5; attempt++) {
        if (attempt > 0) {
            ESP_LOGW(TAG, "Retry %d/5...", attempt + 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config,
                                      &mount_config, &s_card);
        if (ret == ESP_OK) break;
        ESP_LOGW(TAG, "Mount attempt %d failed: %s", attempt + 1, esp_err_to_name(ret));
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed after retries: %s", esp_err_to_name(ret));
        spi_bus_free(host.slot);
        return ret;
    }

    sdmmc_card_print_info(stdout, s_card);

    uint64_t total_bytes = (uint64_t)s_card->csd.capacity * s_card->csd.sector_size;
    ESP_LOGI(TAG, "SD card mounted at %s (%.1f GB)",
             SD_MOUNT_POINT, (double)total_bytes / (1024.0 * 1024.0 * 1024.0));

    return ESP_OK;
}

void sd_card_deinit(void)
{
    if (s_card) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
        spi_bus_free(SPI2_HOST);
        s_card = NULL;
        ESP_LOGI(TAG, "SD card unmounted");
    }
}
