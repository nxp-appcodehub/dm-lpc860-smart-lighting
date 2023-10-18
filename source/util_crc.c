/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/* Board */
#include "board.h"
#include "pin_mux.h"

/**
 * @brief CRC checksum
 * CRC checksum shall be placed at the first or last 4 bytes (considering alignments)
 * It should be calculated on slot-basis, since all writes are performed this way.
 */
#include "fsl_crc.h"

#include "util_crc.h"

void util_crc_init(void) {
    crc_config_t cfg = {
        .polynomial    = kCRC_Polynomial_CRC_32,
        .complementIn  = false,
        .complementOut = true,
        .reverseIn     = true,
        .reverseOut    = true,
        .seed          = 0xFFFFFFFF,
    };
    CRC_Init(CRC0, &cfg);
}

uint32_t util_crc_calculate(uint8_t *data, uint16_t size) {
    uint32_t crc = 0xFFFFFFFF;

    /* LPC84x hardware CRC module */
    CRC_WriteSeed(CRC0, crc);
    CRC_WriteData(CRC0, data, size);
    crc = CRC_Get32bitResult(CRC0);
    return crc;
}