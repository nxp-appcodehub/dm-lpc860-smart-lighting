/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/* Device drivers */
#include "fsl_common.h"

/* Private headers */
#include "util_systick.h"

#define UTIL_SYSTICK_PRIO ((1UL << __NVIC_PRIO_BITS) - 1UL)
#define UTIL_SYSTICK_RATE 1000

static volatile uint32_t s_tickcount;

/**
 * @brief Enable SysTick counter.
 *
 */
void util_systick_init(void) {
    /* Initialize SysTick timer and other sys components(if any) */
    uint32_t core_clock = CLOCK_GetCoreSysClkFreq();

    s_tickcount         = 0U;

    NVIC_SetPriority(SysTick_IRQn, UTIL_SYSTICK_PRIO);

    /* Reset SysTick peripheral */
    SysTick->CTRL = 0UL;
    SysTick->VAL  = 0UL;

    /* Configure counter clock source and interrupt */
    SysTick->CTRL = SysTick_CTRL_TICKINT_Msk;
    SysTick->LOAD = (core_clock / (2 * UTIL_SYSTICK_RATE)) - 1;

    /* Enable counter */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

/**
 * @brief Get current counter value, may overflow.
 *
 * @return uint32_t current tick count
 */
uint32_t util_systick_get(void) {
    /* No additional operation required. */
    return s_tickcount;
}

/**
 * @brief Delay for specific msec.
 *
 * @param msec milliseconds to delay
 */
void util_systick_delay(uint32_t msec) {
    uint32_t tick_end = s_tickcount + msec;
    while (s_tickcount < tick_end) {
        __WFI();
    }
}

/**
 * @brief SysTick interrupt handler
 *
 */
void SysTick_Handler(void) {
    /* Increment local counter */
    s_tickcount++;
}