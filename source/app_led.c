/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include <string.h>

/* SDK drivers */
#include "fsl_ftm.h"
#include "fsl_power.h"

/* Board */
#include "pin_mux.h"

/* Shared ADC */
#include "app_adc_shared.h"

/* Private */
#include "app_led.h"

#define LED_DRIVER_FTM_INSTANCE FTM0
#define LED_DRIVER_SWITCH_FREQ  99000 /* Switching frequency 99kHz */
#define LED_DRIVER_DEADTIME_NS  850   /* DeadTime 800ns */
#define LED_DRIVER_LED_IF_MAX   150   /* 150mA per channel */
#define LED_DRIVER_LED_SHUNT    5     /* 5R for LED shunt */

#define LED_PID_KP 5000
#define LED_PID_KI 2
#define LED_PID_KD 0

#define LED_PID_MULT 100000

#define LED_PID_FORMULA(err, prev_err, err_sum, prev_out) \
    ((LED_PID_KP * err + LED_PID_KI * err_sum + LED_PID_KD * (err - prev_err) + prev_out * LED_PID_MULT) / LED_PID_MULT)

static void app_led_set_duty_cycle(app_led_channel_t led_ch, uint16_t duty);
static void app_led_control_loop(void);

static uint16_t s_dead_cnv = 0U;
static int      s_target_adc_value[3];
static uint16_t s_control_loop_prev_output[3];
static int      s_control_loop_err_sum[3];
static int      s_control_loop_prev_err[3];

int app_led_init(void) {
    ftm_config_t                ftm_cfg;
    ftm_chnl_pwm_signal_param_t chn_param[3];
    uint32_t                    ftm_root_clock;

    memset(s_control_loop_err_sum, 0x00, sizeof(s_control_loop_err_sum));
    memset(s_control_loop_prev_err, 0x00, sizeof(s_control_loop_prev_err));
    memset(s_control_loop_prev_output, 0x00, sizeof(s_control_loop_prev_output));

    ftm_root_clock = CLOCK_GetCoreSysClkFreq(); /* FTM frequency same as AHB/CPU */

    FTM_GetDefaultConfig(&ftm_cfg);

    /* Configure FTM frequency & prescale */
    ftm_cfg.prescale = FTM_CalculateCounterClkDiv(LED_DRIVER_FTM_INSTANCE, LED_DRIVER_SWITCH_FREQ, ftm_root_clock);

    /* Configure FTM safe state & BDM */
    ftm_cfg.bdmMode       = kFTM_BdmMode_1;
    ftm_cfg.chnlInitState = 0x00U;
    ftm_cfg.chnlPolarity  = 0x00U; /* Channel safe state: high side MOSFET OFF, low side MOSFET ON */

    /* Configure dead time */
    ftm_cfg.deadTimePrescale = kFTM_Deadtime_Prescale_4;
    ftm_cfg.deadTimeValue    = 1 + (LED_DRIVER_DEADTIME_NS / (1000000000 / (ftm_root_clock / 4)));

    s_dead_cnv = ftm_cfg.deadTimeValue * (1 << ftm_cfg.deadTimePrescale) / (1 << ftm_cfg.prescale);

    FTM_Init(LED_DRIVER_FTM_INSTANCE, &ftm_cfg);

    /* Configure FTM channel pairs as complementary PWM output */
    chn_param[0].chnlNumber            = kFTM_Chnl_0;
    chn_param[0].dutyCyclePercent      = 0U;
    chn_param[0].firstEdgeDelayPercent = 0U;
    chn_param[0].enableComplementary   = true;
    chn_param[0].enableDeadtime        = true;
    chn_param[0].level                 = kFTM_HighTrue;

    chn_param[1].chnlNumber            = kFTM_Chnl_1;
    chn_param[1].dutyCyclePercent      = 0U;
    chn_param[1].firstEdgeDelayPercent = 0U;
    chn_param[1].enableComplementary   = true;
    chn_param[1].enableDeadtime        = true;
    chn_param[1].level                 = kFTM_HighTrue;

    chn_param[2].chnlNumber            = kFTM_Chnl_2;
    chn_param[2].dutyCyclePercent      = 0U;
    chn_param[2].firstEdgeDelayPercent = 0U;
    chn_param[2].enableComplementary   = true;
    chn_param[2].enableDeadtime        = true;
    chn_param[2].level                 = kFTM_HighTrue;
    /*
     * In combined PWM mode, the channel number will be divided by 2 in the driver code
     * Real channel pairs: 0/1, 2/3, 4/5, use 0, 1, 2 here.
     */
    FTM_SetupPwm(LED_DRIVER_FTM_INSTANCE, chn_param, 3U, kFTM_EdgeAlignedCombinedPwm, LED_DRIVER_SWITCH_FREQ,
                 ftm_root_clock);

    /* Enable FTM outputs */
    BOARD_InitLEDPins();

    FTM_StartTimer(LED_DRIVER_FTM_INSTANCE, kFTM_SystemClock);

    app_adc_shared_set_callback(app_led_control_loop);

    return 0;
}

int app_led_deinit(void) {
    FTM_StopTimer(LED_DRIVER_FTM_INSTANCE);
    FTM_Deinit(LED_DRIVER_FTM_INSTANCE);

    BOARD_InitLEDPins_deinit();

    return 0;
}

/**
 * @brief Set LED brightness
 *
 * @param led_ch LED channel to be set
 * @param br brightness ranges from 0 to 255.
 */
void app_led_set_brightness(app_led_channel_t led_ch, uint8_t br) {
    int target_value = (LED_DRIVER_LED_IF_MAX * LED_DRIVER_LED_SHUNT) * br / 256;
    target_value     = target_value * 4096 / 3300;

    switch (led_ch) {
        case APP_LED_CH_R:
            s_target_adc_value[0] = target_value;
            break;
        case APP_LED_CH_G:
            s_target_adc_value[1] = target_value;
            break;
        case APP_LED_CH_B:
            s_target_adc_value[2] = target_value;
            break;
        default:
            break;
    }
}

/**
 * @brief Get Current LED brightness
 *
 * @param led_ch LED channel to be quried
 * @return uint8_t LED brightness ranges from 0 to 255
 */
uint8_t app_led_get_brightness(app_led_channel_t led_ch) {
    int actual_value = 0;
    switch (led_ch) {
        case APP_LED_CH_R:
            actual_value = app_adc_shared_get_voltage(APP_ADC_SHARED_CH_LED_R);
            break;
        case APP_LED_CH_G:
            actual_value = app_adc_shared_get_voltage(APP_ADC_SHARED_CH_LED_G);
            break;
        case APP_LED_CH_B:
            actual_value = app_adc_shared_get_voltage(APP_ADC_SHARED_CH_LED_B);
            break;
    }

    actual_value = actual_value * 256 / (LED_DRIVER_LED_IF_MAX * LED_DRIVER_LED_SHUNT);
    if (actual_value > 255) actual_value = 255;

    return (uint8_t)actual_value;
}

static void app_led_set_duty_cycle(app_led_channel_t led_ch, uint16_t duty) {
    LED_DRIVER_FTM_INSTANCE->CONTROLS[led_ch * 2 + 1].CnV = duty;
    FTM_SetSoftwareTrigger(LED_DRIVER_FTM_INSTANCE, true);
}

/**
 * @brief PI control loop
 *
 */
static void app_led_control_loop(void) {
    uint16_t adj_range = LED_DRIVER_FTM_INSTANCE->MOD - s_dead_cnv * 2;
    uint16_t adj_start = s_dead_cnv;
    uint16_t adj_end   = adj_range + s_dead_cnv;

    int adc_r = app_adc_shared_get_result(APP_ADC_SHARED_CH_LED_R);
    int adc_g = app_adc_shared_get_result(APP_ADC_SHARED_CH_LED_G);
    int adc_b = app_adc_shared_get_result(APP_ADC_SHARED_CH_LED_B);

    int err_r = s_target_adc_value[0] - adc_r;
    int err_g = s_target_adc_value[1] - adc_g;
    int err_b = s_target_adc_value[2] - adc_b;

    s_control_loop_err_sum[0] += err_r;
    s_control_loop_err_sum[1] += err_g;
    s_control_loop_err_sum[2] += err_b;

    int m_next_r =
        LED_PID_FORMULA(err_r, s_control_loop_prev_err[0], s_control_loop_err_sum[0], s_control_loop_prev_output[0]);
    int m_next_g =
        LED_PID_FORMULA(err_g, s_control_loop_prev_err[1], s_control_loop_err_sum[1], s_control_loop_prev_output[1]);
    int m_next_b =
        LED_PID_FORMULA(err_b, s_control_loop_prev_err[2], s_control_loop_err_sum[2], s_control_loop_prev_output[2]);

    if (m_next_r > adj_end) m_next_r = adj_end;
    if (m_next_g > adj_end) m_next_g = adj_end;
    if (m_next_b > adj_end) m_next_b = adj_end;

    if (m_next_r < adj_start) m_next_r = adj_start;
    if (m_next_g < adj_start) m_next_g = adj_start;
    if (m_next_b < adj_start) m_next_b = adj_start;

    if (s_target_adc_value[0] == 0U) m_next_r = 0;
    if (s_target_adc_value[1] == 0U) m_next_g = 0;
    if (s_target_adc_value[2] == 0U) m_next_b = 0;

    app_led_set_duty_cycle(APP_LED_CH_R, (uint16_t)m_next_r);
    app_led_set_duty_cycle(APP_LED_CH_G, (uint16_t)m_next_g);
    app_led_set_duty_cycle(APP_LED_CH_B, (uint16_t)m_next_b);

    s_control_loop_prev_err[0] = err_r;
    s_control_loop_prev_err[1] = err_g;
    s_control_loop_prev_err[2] = err_b;

    s_control_loop_prev_output[0] = m_next_r;
    s_control_loop_prev_output[1] = m_next_g;
    s_control_loop_prev_output[2] = m_next_b;
}