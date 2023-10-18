/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/* Board */
#include "clock_config.h"
#include "pin_mux.h"

/* SDK drivers */
#include "fsl_adc.h"
#include "fsl_dma.h"
#include "fsl_gpio.h"
#include "fsl_inputmux.h"
#include "fsl_power.h"

/* Private */
#include "app_adc_shared.h"

#define APP_ADC_SHARED_INSTANCE      ADC0
#define APP_ADC_SHARED_PD            kPDRUNCFG_PD_ADC0
#define APP_ADC_SHARED_CHANNEL_COUNT 12  /* Total ADC channels to be sampled */
#define APP_ADC_SHARED_SAMPLE_TIME   24  /* ADC sample time is minimal 6.5 ADC clock cycles */
#define APP_ADC_SHARED_CLK_DIV       121 /* ADC clock divider */
#define APP_ADC_SHARED_AVG_COUNT     6

#define APP_ADC_SHARED_DMA_INSTANCE DMA0
#define APP_ADC_SHARED_DMA_IRQ_NUMBER DMA0_IRQn
#define APP_ADC_SHARED_DMA_IRQ_PRIO 1
#define APP_ADC_SHARED_DMA_CHANNEL  0

/* Convert channel number to data register */
#define APP_ADC_CHANNEL_TO_ADC_MASK(ch) (1U << ch)
#define APP_ADC_DAT_TO_VALUE(dat)       ((dat & ADC_DAT_RESULT_MASK) >> ADC_DAT_RESULT_SHIFT)

static inline uint32_t app_adc_get_avg_data(app_adc_shared_channel_t ch);
static void app_adc_data_ready_callback(dma_handle_t *handle, void *userData, bool transferDone, uint32_t intmode);

static DMA_ALLOCATE_LINK_DESCRIPTORS(s_app_adc_dma_desc, APP_ADC_SHARED_AVG_COUNT);
static dma_handle_t s_app_adc_dma_handle;
static uint32_t     s_app_adc_dma_results[APP_ADC_SHARED_AVG_COUNT][APP_ADC_SHARED_CHANNEL_COUNT];
static app_adc_cb_t s_app_adc_cb = NULL;

/**
 * @brief Initialize shared ADC interface
 * 
 * @return int 0 for success, negative values for error
 */
int app_adc_shared_init(void) {
    adc_config_t          adc_cfg;
    adc_conv_seq_config_t seq_cfg;  /* Sequence A */
    dma_channel_trigger_t dma_trig; /* DMA chanel trigger for ADC sequence A */
    uint32_t              adc_root_clock;

    /* Use 30/30MHz to calibrate ADC */
    BOARD_BootClockFRO30M();

    CLOCK_Select(kADC_Clk_From_Fro);
    CLOCK_SetClkDivider(kCLOCK_DivAdcClk, 1U);
    CLOCK_EnableClock(kCLOCK_Adc);

    adc_root_clock = CLOCK_GetFreq(kCLOCK_Fro) / CLOCK_GetClkDivider(kCLOCK_DivAdcClk);

    POWER_DisablePD(APP_ADC_SHARED_PD);

    if (ADC_DoSelfCalibration(APP_ADC_SHARED_INSTANCE, adc_root_clock) != true) {
        return -1;
    }

    /* Restore original clocking */
    BOARD_InitBootClocks();

    CLOCK_SetClkDivider(kCLOCK_DivAdcClk, 2U);
    CLOCK_Select(kADC_Clk_From_SysPll_DIV);

    /* ADC uses PLLDIV clock and has its own divider. */
    adc_root_clock = CLOCK_GetFreq(kCLOCK_PllOut) / CLOCK_GetClkDivider(kCLOCK_DivPllClk);
    adc_root_clock = adc_root_clock / CLOCK_GetClkDivider(kCLOCK_DivAdcClk);

    ADC_GetDefaultConfig(&adc_cfg);

    adc_cfg.clockDividerNumber = APP_ADC_SHARED_CLK_DIV;

    ADC_Init(APP_ADC_SHARED_INSTANCE, &adc_cfg);

    BOARD_InitADCPins();

    /* Channel A: ALS, NTC and POT */
    seq_cfg.channelMask =
        APP_ADC_CHANNEL_TO_ADC_MASK(APP_ADC_SHARED_CH_ALS) | APP_ADC_CHANNEL_TO_ADC_MASK(APP_ADC_SHARED_CH_NTC) |
        APP_ADC_CHANNEL_TO_ADC_MASK(APP_ADC_SHARED_CH_POT) | APP_ADC_CHANNEL_TO_ADC_MASK(APP_ADC_SHARED_CH_LED_R) |
        APP_ADC_CHANNEL_TO_ADC_MASK(APP_ADC_SHARED_CH_LED_G) | APP_ADC_CHANNEL_TO_ADC_MASK(APP_ADC_SHARED_CH_LED_B);

    seq_cfg.triggerMask      = 0U;
    seq_cfg.triggerPolarity  = kADC_TriggerPolarityPositiveEdge;
    seq_cfg.enableSingleStep = false;
    seq_cfg.enableSyncBypass = false;
    seq_cfg.interruptMode    = kADC_InterruptForEachSequence;

    ADC_SetConvSeqAConfig(APP_ADC_SHARED_INSTANCE, &seq_cfg);

    ADC_EnableInterrupts(APP_ADC_SHARED_INSTANCE, kADC_ConvSeqAInterruptEnable);
    ADC_EnableInterrupts(APP_ADC_SHARED_INSTANCE, kADC_ConvSeqBInterruptEnable);

    DMA_Init(APP_ADC_SHARED_DMA_INSTANCE);

    INPUTMUX_Init(INPUTMUX);
    INPUTMUX_AttachSignal(INPUTMUX, APP_ADC_SHARED_DMA_CHANNEL, kINPUTMUX_Adc0SeqaIrqToDma);

    DMA_CreateHandle(&s_app_adc_dma_handle, APP_ADC_SHARED_DMA_INSTANCE, APP_ADC_SHARED_DMA_CHANNEL);

    DMA_EnableChannel(APP_ADC_SHARED_DMA_INSTANCE, APP_ADC_SHARED_DMA_CHANNEL);

    dma_trig.burst = kDMA_SingleTransfer;
    dma_trig.type  = kDMA_RisingEdgeTrigger;
    dma_trig.wrap  = kDMA_NoWrap;

    /* Sequence A */

    for (uint8_t i = 0; i < APP_ADC_SHARED_AVG_COUNT; i++) {
        /* Determine which the next descriptor is. */
        uint8_t idx_next   = (i == (APP_ADC_SHARED_AVG_COUNT - 1)) ? 0 : (i + 1);
        bool    enable_irq = (i == (APP_ADC_SHARED_AVG_COUNT - 1)) ? true : false;

        DMA_SetupDescriptor(&s_app_adc_dma_desc[i],
                            DMA_CHANNEL_XFER(true, true, enable_irq, false, 4U, kDMA_AddressInterleave1xWidth,
                                             kDMA_AddressInterleave1xWidth, 4U * APP_ADC_SHARED_CHANNEL_COUNT),
                            (void *)(APP_ADC_SHARED_INSTANCE->DAT), s_app_adc_dma_results[i],
                            &s_app_adc_dma_desc[idx_next]);
    }

    DMA_SetChannelConfig(APP_ADC_SHARED_DMA_INSTANCE, APP_ADC_SHARED_DMA_CHANNEL, &dma_trig, false);

    DMA_SubmitChannelDescriptor(&s_app_adc_dma_handle, &s_app_adc_dma_desc[0]); /* Channel for sequence A */

    DMA_SetCallback(&s_app_adc_dma_handle, app_adc_data_ready_callback, NULL);

    EnableIRQ(APP_ADC_SHARED_DMA_IRQ_NUMBER);
    NVIC_SetPriority(APP_ADC_SHARED_DMA_IRQ_NUMBER, APP_ADC_SHARED_DMA_IRQ_PRIO);

    return 0;
}

int app_adc_shared_start(void) {
    ADC_EnableConvSeqA(APP_ADC_SHARED_INSTANCE, true);
    ADC_EnableConvSeqABurstMode(APP_ADC_SHARED_INSTANCE, true);

    return 0;
}

int app_adc_shared_stop(void) {
    ADC_EnableConvSeqABurstMode(APP_ADC_SHARED_INSTANCE, false);
    ADC_EnableConvSeqA(APP_ADC_SHARED_INSTANCE, false);

    return 0;
}

int app_adc_shared_deinit(void) {
    ADC_Deinit(APP_ADC_SHARED_INSTANCE);
    POWER_EnablePD(kPDRUNCFG_PD_ADC0);
    CLOCK_DisableClock(kCLOCK_Adc);

    BOARD_InitADCPins_deinit();

    return 0;
}

int app_adc_shared_get_result(app_adc_shared_channel_t ch) {
    return app_adc_get_avg_data(ch);
}

int app_adc_shared_get_voltage(app_adc_shared_channel_t ch) {
    return app_adc_get_avg_data(ch) * 3300 / 4096;
}

void app_adc_shared_set_callback(app_adc_cb_t cb) {
    s_app_adc_cb = cb;
}

static inline uint32_t app_adc_get_avg_data(app_adc_shared_channel_t ch) {
    uint32_t adc_dat = 0;

    for (uint8_t i = 0; i < APP_ADC_SHARED_AVG_COUNT; i++) {
        adc_dat += APP_ADC_DAT_TO_VALUE(s_app_adc_dma_results[i][ch]);
    }

    adc_dat /= APP_ADC_SHARED_AVG_COUNT;

    return adc_dat;
}

static void app_adc_data_ready_callback(dma_handle_t *handle, void *userData, bool transferDone, uint32_t intmode) {
    //GPIO_PinWrite(BOARD_INITDBGPINS_DBG0_GPIO, BOARD_INITDBGPINS_DBG0_PORT, BOARD_INITDBGPINS_DBG0_PIN, 1);
    if (s_app_adc_cb != NULL) {
        s_app_adc_cb();
    }

    //GPIO_PinWrite(BOARD_INITDBGPINS_DBG0_GPIO, BOARD_INITDBGPINS_DBG0_PORT, BOARD_INITDBGPINS_DBG0_PIN, 0);
}
