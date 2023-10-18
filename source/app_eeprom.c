/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include "fsl_i2c.h"
#include "fsl_power.h"

#define EEPROM_SIZE      2048
#define EEPROM_PAGE_SIZE 16

#define EEPROM_I2C_INSTANCE I2C0
#define EEPROM_I2C_FREQ     1000000 /* AT24C16 can take up to 1MHz above 1.7V */

#define EEPROM_ADDR_TO_I2C(a) (0x50 | ((a >> 8U) & 0x07U)) /* 0x50 - 0x57, 8 * 256 bytes */
#define EEPROM_ADDR_TO_SUB(a) (a & 0xFFU)

int app_eeprom_init(void) {
    i2c_master_config_t i2c_cfg;
    uint32_t            i2c_root_freq;

    CLOCK_Select(kI2C0_Clk_From_MainClk);
    CLOCK_EnableClock(kCLOCK_I2c0);

    i2c_root_freq = CLOCK_GetFreq(kCLOCK_MainClk);

    I2C_MasterGetDefaultConfig(&i2c_cfg);
    i2c_cfg.baudRate_Bps = EEPROM_I2C_FREQ;

    I2C_MasterInit(EEPROM_I2C_INSTANCE, &i2c_cfg, i2c_root_freq);

    return 0;
}

int app_eeprom_read(uint16_t addr, uint8_t *data, uint16_t len) {
    i2c_master_transfer_t xfer = {
        .slaveAddress   = EEPROM_ADDR_TO_I2C(addr),
        .subaddress     = EEPROM_ADDR_TO_SUB(addr),
        .subaddressSize = 1U,
        .data           = data,
        .dataSize       = len,
        .direction      = kI2C_Read,
        .flags          = kI2C_TransferDefaultFlag,
    };

    status_t ret = I2C_MasterTransferBlocking(EEPROM_I2C_INSTANCE, &xfer);
    if (ret != kStatus_Success) {
        return -1;
    }

    return 0;
}

int app_eeprom_write(uint16_t addr, uint8_t *data, uint16_t len) {
    /*
     * A EEPROM write can start with arbitary address,
     * however the address will wrap if the internal pointer hits the page boundary.
     * 1: If the write can be done within a single page, do it.
     * 2: If the write can't be done within a single page,
     * transfer rest of the first page first.
     *
     * The EEPROM will not respond during its internal write procedure.
     */

    status_t ret = kStatus_Success;

    i2c_master_transfer_t xfer = {
        .slaveAddress   = EEPROM_ADDR_TO_I2C(addr),
        .subaddress     = EEPROM_ADDR_TO_SUB(addr),
        .subaddressSize = 1U,
        .direction      = kI2C_Write,
        .flags          = kI2C_TransferDefaultFlag,
    };

    uint8_t  page_remains = EEPROM_PAGE_SIZE - (addr % EEPROM_PAGE_SIZE);
    uint8_t *write_ptr    = data;

    if (len < page_remains) {
        /* The write can be completed in the current page */

        xfer.data     = data;
        xfer.dataSize = len;

        ret = I2C_MasterTransferBlocking(EEPROM_I2C_INSTANCE, &xfer);
        if (ret != kStatus_Success) {
            return -1;
        }

        return 0;
    } else {
        /* Send the current page remains */
        xfer.data     = data;
        xfer.dataSize = page_remains;

        ret = I2C_MasterTransferBlocking(EEPROM_I2C_INSTANCE, &xfer);
        if (ret != kStatus_Success) {
            return -2;
        }

        write_ptr += page_remains;
    }

    while (write_ptr < (data + len)) {
    }

    return 0;
}
