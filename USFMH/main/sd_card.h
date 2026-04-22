#pragma once

#include "esp_err.h"

#define SD_MOUNT_POINT "/sdcard"

/**
 * Initialize and mount the SD card over SPI.
 * Files are accessible under SD_MOUNT_POINT (e.g. "/sdcard/input.wav").
 */
esp_err_t sd_card_init(void);

/**
 * Unmount and free the SD card SPI bus.
 */
void sd_card_deinit(void);
