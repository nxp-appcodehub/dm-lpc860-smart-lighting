/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include <string.h>

/* SDK drivers */
#include "fsl_crc.h"
#include "fsl_iap.h"

/* Internal header */
#include "util_flash_lpc86x_iap.h"

#define UTIL_FLASH_SECTOR_SIZE  1024 /* Sector size, in bytes */
#define UTIL_FLASH_SECTOR_COUNT 1    /* Sector to be used */
#define UTIL_FLASH_SECTOR_START 62   /* Sector start */
#define UTIL_FLASH_SECTOR_END   (UTIL_FLASH_SECTOR_START + UTIL_FLASH_SECTOR_COUNT - 1)

#define UTIL_FLASH_PAGE_SIZE  64 /* Page size, will be used as slots */
#define UTIL_FLASH_PAGE_START (UTIL_FLASH_SECTOR_START * UTIL_FLASH_SECTOR_SIZE / UTIL_FLASH_PAGE_SIZE)

#define UTIL_FLASH_PARTITION_SIZE (UTIL_FLASH_SECTOR_SIZE * UTIL_FLASH_SECTOR_COUNT)
#define UTIL_FLASH_LOCATION       (UTIL_FLASH_SECTOR_START * UTIL_FLASH_SECTOR_SIZE) /* 64kB - 4kB */

/**
 * @brief Initialize the flash slot.
 *
 * @param slot Pointer to the slot needs to be initialized
 * @return int 0 for success, others for error.
 */
int util_flash_init(void) {
    /* No initialization is required. */

    /**
     * TODO: Multiple sector management
     * Find the last sector with valid magic
     * Erase the next sector, If none of the sectors are available, erase the first sector , "next_sector=0".
     * During BOR, write the cache content to "next_sector"
     * During manual flush, write the cache content into "next_sector" then increment and erase the "next_sector"
     */

    return 0;
}

/**
 * @brief Read data slot.
 *
 * @param slot Pointer to the slot needs to be loaded
 * @param data Pointer to data
 * @param size size of the data to be loaded.
 * @return int 0 for success, others for corrupted or error.
 */
int util_flash_slot_read(util_flash_t *slot, uint8_t *data) {
    uint32_t slot_size = slot->page_count * UTIL_FLASH_PAGE_SIZE;
    uint32_t data_ptr  = UTIL_FLASH_LOCATION + (slot->start_page * UTIL_FLASH_PAGE_SIZE);

    /* Copy from cache */
    memcpy(data, (void *)data_ptr, slot_size);

    return 0;
}

/**
 * @brief Store the data to internal cache, do not flush back to FLASH now.
 *
 * @param slot Pointer to the slot needs to be stored
 * @param data Pointer to data
 * @param size Size of the data, in bytes.
 * @return int 0 for success, others for error.
 */
int util_flash_slot_program(util_flash_t *slot, uint8_t *data) {
    uint32_t slot_size    = slot->page_count * UTIL_FLASH_PAGE_SIZE;
    uint32_t data_ptr     = UTIL_FLASH_LOCATION + (slot->start_page * UTIL_FLASH_PAGE_SIZE);
    uint32_t sys_clock_hz = CLOCK_GetCoreSysClkFreq();

    status_t ret;

    /* Unlock sectors */
    ret = IAP_PrepareSectorForWrite(UTIL_FLASH_SECTOR_START, UTIL_FLASH_SECTOR_END);
    if (ret != kStatus_Success) {
        return -1;
    }

    /* IRQ will be disabled at iap_entry() */
    ret = IAP_CopyRamToFlash(data_ptr, (uint32_t *)data, slot_size, sys_clock_hz);
    if (ret != kStatus_Success) {
        return -2;
    }

    return 0;
}

int util_flash_slot_erase(util_flash_t *slot) {
    int      ret;
    uint32_t page_start   = UTIL_FLASH_PAGE_START + slot->start_page;
    uint32_t page_end     = page_start + slot->page_count - 1;
    uint32_t sys_clock_hz = CLOCK_GetCoreSysClkFreq();

    /* Unlock sectors */
    ret = IAP_PrepareSectorForWrite(UTIL_FLASH_SECTOR_START, UTIL_FLASH_SECTOR_END);
    if (ret != kStatus_Success) {
        return -1;
    }
    /* IRQ will be disabled at iap_entry() */
    ret = IAP_ErasePage(page_start, page_end, sys_clock_hz);
    if (ret != kStatus_Success) {
        return -2;
    }

    return 0;
}

int util_flash_erase_all(void) {
    int      ret;
    uint32_t sys_clock_hz = CLOCK_GetCoreSysClkFreq();

    ret = IAP_BlankCheckSector(UTIL_FLASH_SECTOR_START, UTIL_FLASH_SECTOR_END);
    if (ret != kStatus_Success) {
        if (ret != kStatus_IAP_SectorNotblank) {
            /* Other error. */
            return -1;
        }
    } else {
        /* Flash sector is already blank, skip erasing */
        return 0;
    }

    /* Unlock sectors */
    ret = IAP_PrepareSectorForWrite(UTIL_FLASH_SECTOR_START, UTIL_FLASH_SECTOR_END);
    if (ret != kStatus_Success) {
        return -2;
    }

    ret = IAP_EraseSector(UTIL_FLASH_SECTOR_START, UTIL_FLASH_SECTOR_END, sys_clock_hz);
    if (ret != kStatus_Success) {
        return -3;
    }

    return 0;
}
