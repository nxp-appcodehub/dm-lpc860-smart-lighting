/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/* Board */
#include "pin_mux.h"

/* SDK drivers */
#include "fsl_clock.h"
#include "fsl_ftm.h"
#include "fsl_gpio.h"

/* Helpers */
#include "util_systick.h"

/* Private */
#include "dali_phy_ftm.h"

/* DALI timing parameters */
#define DALI_PHY_HB_TX_PERIOD    417                          /* TX: Half bit period, typ. 416.7us */
#define DALI_PHY_HB_PERIOD_MAX   500                          /* RX: Maximum half bit period, 466.7us */
#define DALI_PHY_HB_PERIOD_MIN   333                          /* RX: Minimum half bit period, 366.7us */
#define DALI_PHY_DHB_TX_PERIOD   (2 * DALI_PHY_HB_TX_PERIOD)  /* TX: Double half bit period */
#define DALI_PHY_DHB_PERIOD_MAX  (2 * DALI_PHY_HB_PERIOD_MAX) /* RX: Maximum double half bit period */
#define DALI_PHY_DHB_PERIOD_MIN  (2 * DALI_PHY_HB_PERIOD_MIN) /* RX: Minimum double half bit period */
#define DALI_PHY_STOP_TX_PERIOD  2500                         /* TX: Stop condition period */
#define DALI_PHY_STOP_PERIOD_MIN 2450                         /* RX: Minimum stop condition time */

/* FTM parameters */
#define DALI_PHY_FTM_INSTANCE   FTM1
#define DALI_PHY_FTM_IRQ_NUMBER FTM1_IRQn
#define DALI_PHY_FTM_IRQ_PRIO   0 /* Highest level */
#define DALI_PHY_FTM_IRQHANDLER FTM1_IRQHandler
#define DALI_PHY_FTM_RATE       (1000000U)                  /* Count rate set to 1MHz, results in 1us period */
#define DALI_PHY_FTM_CLOCK_FREQ (CLOCK_GetCoreSysClkFreq()) /* FTM clock source */

/* FTM channel assignment */
#define DALI_PHY_FTM_CH_MATCH_TX_HB          kFTM_Chnl_2
#define DALI_PHY_FTM_CH_MATCH_TX_HB_INT_MASK kFTM_Chnl2InterruptEnable
#define DALI_PHY_FTM_CH_MATCH_TX_HB_FLAG     kFTM_Chnl2Flag
#define DALI_PHY_FTM_CH_CAPTURE              kFTM_Chnl_1
#define DALI_PHY_FTM_CH_CAPTURE_INT_MASK     kFTM_Chnl1InterruptEnable
#define DALI_PHY_FTM_CH_CAPTURE_FLAG         kFTM_Chnl1Flag

/* Typical condition: If we connect MCU TX with MCU RX,
 * then TX inversion is required since RX signal logic is inverted by PHY.
 */
#ifndef DALI_PHY_FTM_INVERSE_TX_LOGIC
#define DALI_PHY_FTM_INVERSE_TX_LOGIC 0
#endif

#ifndef DALI_PHY_FTM_INVERSE_RX_LOGIC
#define DALI_PHY_FTM_INVERSE_RX_LOGIC 1
#endif

/* IO parameters */
#define DALI_PHY_FTM_TX_GPIO_INSTANCE BOARD_INITDALIPINS_TX_GPIO
#define DALI_PHY_FTM_TX_PORT          BOARD_INITDALIPINS_TX_PORT
#define DALI_PHY_FTM_TX_PIN           BOARD_INITDALIPINS_TX_PIN
#define DALI_PHY_FTM_TX_PIN_MASK      BOARD_INITDALIPINS_TX_PIN_MASK

#define DALI_PHY_FTM_RX_GPIO_INSTANCE GPIO
#define DALI_PHY_FTM_RX_PORT          BOARD_INITDALIPINS_RX_PORT
#define DALI_PHY_FTM_RX_PIN           BOARD_INITDALIPINS_RX_PIN
#define DALI_PHY_FTM_RX_PIN_MASK      BOARD_INITDALIPINS_RX_PIN_MASK

#define DALI_PHY_FTM_TIME_TO_TICK(us) (s_ftm_freq_scale * us)

/* Private struct and enums */
typedef enum {
    DALI_PHY_FTM_STATE_UNINITIALIZED, /* Uninitialized state */
    DALI_PHY_FTM_STATE_IDLE,          /* Initialized, IDLE state */
    DALI_PHY_FTM_STATE_TX,            /* TX in progress */
    DALI_PHY_FTM_STATE_RX,
} dali_phy_ftm_state_t;

typedef enum {
    DALI_PHY_FTM_EVENT_TX_COMPLETED,
    DALI_PHY_FTM_EVENT_RX_COMPLETED,
    DALI_PHY_FTM_EVENT_TX_COLLISION,
    DALI_PHY_FTM_EVENT_RX_FRAMING_ERROR,
} dali_phy_ftm_event_t;

typedef enum {
    DALI_PHY_FTM_FLAG_TX_HB      = (1 << 0U),
    DALI_PHY_FTM_FLAG_RX_RISING  = (1 << 1U),
    DALI_PHY_FTM_FLAG_RX_FALLING = (1 << 2U),
    DALI_PHY_FTM_FLAG_STOP       = (1 << 3U),
} dali_phy_ftm_flag_t;

static void                 dali_phy_ftm_event_callback(dali_phy_ftm_flag_t flags);
static ftm_clock_prescale_t dali_phy_ftm_find_prescale(void);
static uint8_t              dali_phy_ftm_get_capture(void);
static void                 dali_phy_ftm_encode_frame(dali_phy_frame_t type, uint32_t frame, uint32_t *hb_frame);
static dali_ret_t           dali_phy_ftm_decode_frame(dali_phy_frame_t type, uint32_t *frame, uint32_t *hb_frame);
static inline void          dali_phy_ftm_channel_reset_counter(ftm_chnl_t ch, bool enable);
static inline void          dali_phy_ftm_rx_mode(void);
static inline void          dali_phy_ftm_tx_mode(void);
static inline void          dali_phy_ftm_set_tx(bool val);
static inline bool          dali_phy_ftm_get_rx(void);

static uint8_t                   s_ftm_freq_scale = 1U;
static dali_phy_ftm_state_t      s_current_state  = DALI_PHY_FTM_STATE_UNINITIALIZED;
static volatile uint32_t         s_rx_raw[2];
static volatile uint32_t         s_tx_raw[2];
static uint32_t                  s_rx_frame  = 0U;
static volatile uint8_t          s_tx_hb_ptr = 0U;
static volatile uint8_t          s_rx_hb_ptr = 0U;
static volatile dali_phy_frame_t s_tx_frame_type;
static volatile dali_phy_frame_t s_rx_frame_type;
static volatile bool             s_tx_flag = false;
static volatile bool             s_rx_flag = false;

/**
 * @brief Initialize FTM PHY
 * 
 * @param pdev Pointer to user parameter, not used, pass NULL for now.
 * @return dali_ret_t DALI_RET_OK for success, others for failure
 */
dali_ret_t dali_phy_ftm_init(void *pdev) {
    ftm_config_t ftm_cfg;
    FTM_GetDefaultConfig(&ftm_cfg);

    ftm_cfg.prescale = dali_phy_ftm_find_prescale();

    if (ftm_cfg.prescale > 7) {
        /* Failed to find a suitable value. */
        return DALI_RET_FAIL;
    }

    if (FTM_Init(DALI_PHY_FTM_INSTANCE, &ftm_cfg) != kStatus_Success) {
        return DALI_RET_FAIL;
    }

    /* Setup capture channels for both edges */
    FTM_SetupInputCapture(DALI_PHY_FTM_INSTANCE, DALI_PHY_FTM_CH_CAPTURE, kFTM_RiseAndFallEdge, 0);

    FTM_SetupOutputCompare(DALI_PHY_FTM_INSTANCE, DALI_PHY_FTM_CH_MATCH_TX_HB, kFTM_NoOutputSignal,
                           DALI_PHY_FTM_TIME_TO_TICK(DALI_PHY_HB_TX_PERIOD));

    FTM_SetTimerPeriod(DALI_PHY_FTM_INSTANCE, DALI_PHY_FTM_TIME_TO_TICK(DALI_PHY_STOP_TX_PERIOD));
    FTM_SetSoftwareTrigger(DALI_PHY_FTM_INSTANCE, true);

    /* Overflow interrupt enable, we use overflow as stop condition match. */
    FTM_EnableInterrupts(DALI_PHY_FTM_INSTANCE, kFTM_TimeOverflowInterruptEnable);

    dali_phy_ftm_rx_mode();

    dali_phy_ftm_set_tx(true);

    dali_phy_ftm_channel_reset_counter(DALI_PHY_FTM_CH_MATCH_TX_HB, false);

    EnableIRQ(DALI_PHY_FTM_IRQ_NUMBER);
    NVIC_SetPriority(DALI_PHY_FTM_IRQ_NUMBER, DALI_PHY_FTM_IRQ_PRIO);

    s_current_state = DALI_PHY_FTM_STATE_IDLE;

    FTM_StartTimer(DALI_PHY_FTM_INSTANCE, kFTM_SystemClock);

    return DALI_RET_OK;
}

/**
 * @brief Transmit a frame to DALI bus, depends on the frame type.
 * TODO: Collision detection for 24bit FFs (multi-master)
 * @param pdev Pointer to user parameter, not used, pass NULL for now.
 * @param frame_type One of the 3 possible frame types defined in \ref dali_phy_frame_t
 * @param frame frame data, right aligned.
 * @return dali_ret_t DALI_RET_OK for success, others for failure
 */
dali_ret_t dali_phy_ftm_transmit(void *pdev, dali_phy_frame_t frame_type, uint32_t frame) {
    /* This function schedules a frame to be transmitted ASAP. */

    if (s_current_state != DALI_PHY_FTM_STATE_IDLE) {
        /* A new frame on the wire? This should not happen due to upper layer settling time... */
        return DALI_RET_PHY_BUSY_RX;
    }

    /* Convert frame to HB version */
    dali_phy_ftm_encode_frame(frame_type, frame, (uint32_t *)s_tx_raw);

    /* Enable Match IRQs and HB reset */
    dali_phy_ftm_tx_mode();
    dali_phy_ftm_channel_reset_counter(DALI_PHY_FTM_CH_MATCH_TX_HB, true);

    s_tx_flag       = false;
    s_tx_frame_type = frame_type;

    FTM_StartTimer(DALI_PHY_FTM_INSTANCE, kFTM_SystemClock);

    /* Ready to send frame in ISR. */

    while (s_tx_flag != true) {
        __WFI(); /* Wait... */
    }

    return DALI_RET_OK;
}

/**
 * @brief Receive a frame from DALI bus, blocking until timeout is reached
 * 
 * @param pdev  Pointer to user parameter, not used, pass NULL for now.
 * @param pframe_type Pointer to one of the 3 possible frame types defined in \ref dali_phy_frame_t
 * @param pframe Pointer to frame data, right aligned.
 * @param timeout_ms Blocking timeout in milliseconds
 * @return dali_ret_t DALI_RET_OK for success, others for failure
 */
dali_ret_t dali_phy_ftm_receive(void *pdev, dali_phy_frame_t *pframe_type, uint32_t *pframe, uint32_t timeout_ms) {
    s_rx_flag = false;

    uint32_t tick_timeout = util_systick_get() + timeout_ms;

    /* Receive a frame within timeout */
    while (s_rx_flag != true) {
        /* Check for timeout condition */
        if (util_systick_get() >= tick_timeout) {
            return DALI_RET_PHY_RX_TIMEOUT;
        }
        __WFI();
    }

    *pframe      = s_rx_frame;
    *pframe_type = s_rx_frame_type;

    return DALI_RET_OK;
}

static void dali_phy_ftm_phy_callback(dali_phy_ftm_event_t event) {
    /* Receive TX and RX event callback from ISR */
    switch (event) {
        case DALI_PHY_FTM_EVENT_TX_COMPLETED:
            s_tx_flag = true;
            break;
        case DALI_PHY_FTM_EVENT_RX_COMPLETED:
            s_rx_flag = true;
            break;
        default:
            break;
    }
}

void DALI_PHY_FTM_IRQHANDLER(void) {
    /* Set Debug IO */
    GPIO_PinWrite(BOARD_INITDBGPINS_DBG1_GPIO, BOARD_INITDBGPINS_DBG1_PORT, BOARD_INITDBGPINS_DBG1_PIN, 1);

    /* Check FTM IRQ flags */
    ftm_status_flags_t flags = FTM_GetStatusFlags(DALI_PHY_FTM_INSTANCE);
    FTM_ClearStatusFlags(DALI_PHY_FTM_INSTANCE, flags);

    dali_phy_ftm_flag_t p_flags = 0;

    if (flags & kFTM_TimeOverflowFlag) {
        p_flags |= DALI_PHY_FTM_FLAG_STOP;
    }

    if (flags & DALI_PHY_FTM_CH_MATCH_TX_HB_FLAG) {
        if (DALI_PHY_FTM_INSTANCE->CONTROLS[DALI_PHY_FTM_CH_MATCH_TX_HB].CnSC & FTM_CnSC_CHIE_MASK) {
            p_flags |= DALI_PHY_FTM_FLAG_TX_HB;
            GPIO_PortToggle(BOARD_INITDBGPINS_DBG0_GPIO, BOARD_INITDBGPINS_DBG0_PORT, BOARD_INITDBGPINS_DBG0_PIN_MASK);
        }
    }

    if (flags & DALI_PHY_FTM_CH_CAPTURE_FLAG) {
        if (DALI_PHY_FTM_INSTANCE->CONTROLS[DALI_PHY_FTM_CH_CAPTURE].CnSC & FTM_CnSC_CHIE_MASK) {
            if (dali_phy_ftm_get_rx()) {
                p_flags |= DALI_PHY_FTM_FLAG_RX_RISING;
            } else {
                p_flags |= DALI_PHY_FTM_FLAG_RX_FALLING;
            }
        }
    }

    dali_phy_ftm_event_callback(p_flags);

    /* Clear Debug IO */
}

/**
 * @brief Handles FTM match and capture events, run the state machine
 * Note: Use \ref kFTM_MultipleCallback
 * @param flags FTM status flags
 */
static void dali_phy_ftm_event_callback(dali_phy_ftm_flag_t flags) {
    if (flags & DALI_PHY_FTM_FLAG_TX_HB) {
        /* TXHB flag, if we are already in TX state, shift an half bit out.
         * If we are not in TX state, but there is a frame pending, which means
         * the timer has been started by the TX function.
         */

        DALI_PHY_FTM_INSTANCE->CNT = 0U;

        if (s_current_state == DALI_PHY_FTM_STATE_IDLE) { /* A new frame has started */
            s_current_state = DALI_PHY_FTM_STATE_TX;      /* Set state to TX */

            dali_phy_ftm_set_tx(false); /* 1st start bit, 01 */
            s_tx_hb_ptr = 0xFFU;        /* Special flag for the latter half of the START bit. */
        } else if (s_current_state == DALI_PHY_FTM_STATE_TX) {
            /* Special flag, send the rest of START bit */
            if (s_tx_hb_ptr == 0xFFU) {
                dali_phy_ftm_set_tx(true); /* START bit done. */
                s_tx_hb_ptr = 0U;          /* Reset TX counter */

                return;
            }

            /*
             * Current frame is completed, write STOP condition,
             * disable HB MR interrupt and reset, wait for STOP flag timeout.
             */
            if (s_tx_hb_ptr == DALI_PHY_FRAME_BITS(s_tx_frame_type) * 2) {
                FTM_DisableInterrupts(DALI_PHY_FTM_INSTANCE, DALI_PHY_FTM_CH_MATCH_TX_HB_INT_MASK);

                /* Do not clear on HB match... */
                dali_phy_ftm_channel_reset_counter(DALI_PHY_FTM_CH_MATCH_TX_HB, false);

                dali_phy_ftm_set_tx(true);

                return;
            }
            /* Shift 1 bit until we reach the end of the frame */

            bool bit_to_send = false;

            /* Our buffer is aligned to the left, but DALI requires MSB first. */
            uint8_t tx_bit_index = DALI_PHY_FRAME_BITS(s_tx_frame_type) * 2 - s_tx_hb_ptr - 1;

            if (tx_bit_index < 32) {
                bit_to_send = (s_tx_raw[0] & (1 << tx_bit_index)) ? true : false;
            } else {
                bit_to_send = (s_tx_raw[1] & (1 << (tx_bit_index - 32))) ? true : false;
            }

            dali_phy_ftm_set_tx(bit_to_send);

            s_tx_hb_ptr += 1;
        }

    } else if (flags & DALI_PHY_FTM_FLAG_STOP) {
        /* STOP flag, if we are in TX or RX states, this flag indicates a STOP condition has occurred. */

        if (s_current_state == DALI_PHY_FTM_STATE_TX) {
            /* TX state ends, no IO needs to be toggled, enable RX interrupts and back to IDLE mode. */
            dali_phy_ftm_rx_mode();

            dali_phy_ftm_phy_callback(DALI_PHY_FTM_EVENT_TX_COMPLETED);

            s_current_state = DALI_PHY_FTM_STATE_IDLE;

        } else if (s_current_state == DALI_PHY_FTM_STATE_RX) {
            /* We received an STOP condition during RX, check RX line level to see
             * whether we encountered a STOP condition or a bus timeout..
             */

            if (dali_phy_ftm_get_rx() == true) { /* Not a timeout condition */
                dali_phy_frame_t decoded_frame = DALI_PHY_FRAME_INVALID;
                if (s_rx_hb_ptr == 16 || s_rx_hb_ptr == 17) {
                    decoded_frame = DALI_PHY_FRAME_8BF;
                } else if (s_rx_hb_ptr == 32 || s_rx_hb_ptr == 33) {
                    decoded_frame = DALI_PHY_FRAME_16FF;
                } else if (s_rx_hb_ptr == 48 || s_rx_hb_ptr == 49) {
                    decoded_frame = DALI_PHY_FRAME_24FF;
                }

                if (decoded_frame != DALI_PHY_FRAME_INVALID) {
                    if (dali_phy_ftm_decode_frame(decoded_frame, &s_rx_frame, (uint32_t *)s_rx_raw) == DALI_RET_OK) {
                        s_rx_frame_type = decoded_frame;

                        dali_phy_ftm_phy_callback(DALI_PHY_FTM_EVENT_RX_COMPLETED);
                    }
                }
            }

            s_rx_hb_ptr     = 0U;
            s_current_state = DALI_PHY_FTM_STATE_IDLE;

            /* Try decoding the frame and check its type based on frame length, call upper layer callback. */
        } else if (s_current_state == DALI_PHY_FTM_STATE_IDLE) {
            /**
             * In IDLE state, STOP condition match is the only enabled interrupt source.
             * NOP.
             */
        }

        /*
         * For reception mode, if we haven't received any capture interrupts, but the STOP condition match
         * fired, this means we have received a stop condition or timeout condition, depends on the bus level.
         */

    } else if (flags & DALI_PHY_FTM_FLAG_RX_FALLING) {
        /* Falling edge, if we are in IDLE state, this indicates the start of a new frame. */
        /* Capture value needs to be checked to see we received one hb or two hbs. */

        /* Note: FTM can reset the counter on capture channels, so no manual reset is required */

        if (s_current_state == DALI_PHY_FTM_STATE_IDLE) { /* New frame, start bit falling edge. */
            /* Special flag for 2nd edge(rising) */
            s_rx_hb_ptr = 0xFFU;
        } else if (s_current_state == DALI_PHY_FTM_STATE_RX) { /* Falling edge capture... */
            uint8_t capture_count = dali_phy_ftm_get_capture();

            if (capture_count == 1) {
                /* 1HB */
                /* Falling edge, shift 0 (NO ACTION) */
                s_rx_hb_ptr++;

            } else if (capture_count == 2) {
                /* 2HB */
                if (s_rx_hb_ptr < 32) {
                    s_rx_raw[0] |= (1 << (s_rx_hb_ptr)); /* Since it's reversed, we need to shift '01'  */
                } else {
                    s_rx_raw[1] |= (1 << (s_rx_hb_ptr - 32));
                }
                s_rx_hb_ptr += 2;
            } else {
                s_rx_hb_ptr     = 0U;
                s_current_state = DALI_PHY_FTM_STATE_IDLE;
            }
        }

    } else if (flags & DALI_PHY_FTM_FLAG_RX_RISING) {
        /* Rising edge, see above. */

        /* Note: FTM can reset the counter on capture channels, so no manual reset is required */

        if (s_current_state == DALI_PHY_FTM_STATE_IDLE) {
            /* Received a rising edge during IDLE, check if this is a valid START bit. */
            uint8_t capture_count = dali_phy_ftm_get_capture();

            if (capture_count == 1) {
                /* This is a valid START bit, check RX special flag. */
                if (s_rx_hb_ptr == 0xFFU) {
                    /* Special flag valid, start receiving... */
                    s_current_state = DALI_PHY_FTM_STATE_RX; /* State transition to RX */
                    s_rx_hb_ptr     = 0U;
                    s_rx_raw[0]     = 0UL;
                    s_rx_raw[1]     = 0UL;
                }
            } else {
                s_rx_hb_ptr     = 0U;
                s_current_state = DALI_PHY_FTM_STATE_IDLE;
            }
        } else if (s_current_state == DALI_PHY_FTM_STATE_RX) {
            /* Received one rising edge bit, read capture value and check if this is valid HB or 2HB */
            uint8_t capture_count = dali_phy_ftm_get_capture();
            /*
             * Notes on the final rising edge: as we only depends on the edges, if the final bit is
             * an '01' bit, the last HB will be counted as STOP condition, this is tolerated since
             * the STOP condition will be extended to Tstop+Thb.
             */

            /*
             * 1HB: Shift 1bits (1), 2HB: Shift 2bits (01).
             * E.g.
             * F, 1, R, 2, F -> 0110
             * R, 1, F, 2, R -> 1001
             */

            if (capture_count == 1) {
                /* 1HB */
                if (s_rx_hb_ptr < 32) {
                    s_rx_raw[0] |= (1 << s_rx_hb_ptr);
                } else {
                    s_rx_raw[1] |= (1 << (s_rx_hb_ptr - 32));
                }
                s_rx_hb_ptr++; /* 1HB Rising, Shift 1 */

            } else if (capture_count == 2) {
                /* 2HB */
                if (s_rx_hb_ptr < 31) {
                    s_rx_raw[0] |= (1 << (s_rx_hb_ptr + 1)); /* Since it's reversed, we need to shift '10'  */
                } else {
                    s_rx_raw[1] |= (1 << (s_rx_hb_ptr - 31));
                }
                s_rx_hb_ptr += 2;
            } else {
                /* Framing error, cancel receive mode */
                s_rx_hb_ptr     = 0U;
                s_current_state = DALI_PHY_FTM_STATE_IDLE;
            }
        }

    } else {
        /* ???? */
    }
}

static ftm_clock_prescale_t dali_phy_ftm_find_prescale(void) {
    uint32_t ftm_root_clock = DALI_PHY_FTM_CLOCK_FREQ; /* FTM shares the same clock frequency as CPU */

    /* To get accurate 1us timing, the final frequency must be percisely 1-26MHz. */
    /* Try each combination from lowest frequency, and test if this is a exact freq. */
    for (uint8_t i = 0; i <= 7; i++) {
        uint32_t postdiv_clk = (ftm_root_clock / (1 << (7 - i)));
        /* Be */
        if (postdiv_clk % DALI_PHY_FTM_RATE == 0) {
            /* Check if this frequency is low enough for stop condition */
            if ((1000 * 65536 / postdiv_clk) > ((DALI_PHY_STOP_TX_PERIOD / 1000) + 1)) {
                s_ftm_freq_scale = postdiv_clk / DALI_PHY_FTM_RATE;
                return (7 - i);
            }
        }
    }

    return 8; /* Invalid value */
}

/**
 * @brief Encode a frame to its half bit version
 *
 * @param type One of \ref dali_phy_frame_t
 * @param frame DALI frame to be encoded
 * @param hb_frame [OUT] Pointer to the half bit frame buffer
 */
static void dali_phy_ftm_encode_frame(dali_phy_frame_t type, uint32_t frame, uint32_t *hb_frame) {
    /* Shift a bit from frame, write two bits to hb_frame. */
    /* We do not add start bit and STOP condition here, they will be handled by the ISR. */

    uint8_t bits_to_encode = DALI_PHY_FRAME_BITS(type);

    /* the maximum payload length is 48. */
    hb_frame[0] = 0UL;
    hb_frame[1] = 0UL;

    for (uint8_t i = 0; i < bits_to_encode; i++) {
        uint8_t dual_hb;

        if (frame & (1U << i)) {
            dual_hb = 1U; /* 0-1 transition */
        } else {
            dual_hb = 2U; /* 1-0 transition */
        }

        if (i < 16) { /* 16 is the word boundary (32bit), though I do not think we still use BE devices */
            hb_frame[0] |= (dual_hb << (2 * i));
        } else {
            hb_frame[1] |= (dual_hb << (2 * (i - 16)));
        }
    }
}

/**
 * @brief Decode a raw half bit frame
 *
 * @param type One of \ref dali_phy_frame_t
 * @param frame [OUT] DALI frame to be encoded
 * @param hb_frame [IN] Half bit frame, NOTE: This is bit-reversed (since RX is MSB first).
 * @return dali_ret_t DALI_RET_OK if success, DALI_RET_PHY_RX_ERROR if framing error occurred.
 */
static dali_ret_t dali_phy_ftm_decode_frame(dali_phy_frame_t type, uint32_t *frame, uint32_t *hb_frame) {
    /* We do not deal with start bit and STOP condition, either. Leave them to the ISR. */

    uint8_t bits_to_decode = DALI_PHY_FRAME_BITS(type);

    /* The maximum frame length is 24 */
    *frame = 0UL;

    for (uint8_t i = 0; i < bits_to_decode; i++) {
        uint8_t dual_hb = 0U;

        if (i < 16) {
            dual_hb = (hb_frame[0] >> (2 * i)) & 3U;
        } else {
            dual_hb = (hb_frame[1] >> (2 * (i - 16))) & 3U;
        }

        if (dual_hb == 2U) { /* The order is reversed, so '10' is a rising edge (filled from LSB) */
            *frame |= (1 << (bits_to_decode - (i + 1)));
        }

        if (dual_hb == 0 || dual_hb == 3) {
            /* Frame error */
            return DALI_RET_PHY_RX_ERROR;
        }
    }

    return DALI_RET_OK;
}

/**
 * @brief Get capture value and compare with window limits.
 *
 * @param ch one of ftm_capture_channel_t
 * @return uint8_t 0 for timing error, 1 for 1HB, 2 for 2HB
 */
static uint8_t dali_phy_ftm_get_capture(void) {
    uint32_t capture_value = 0UL;

    /* Get current channel capture value */
    capture_value = FTM_GetInputCaptureValue(DALI_PHY_FTM_INSTANCE, DALI_PHY_FTM_CH_CAPTURE) / s_ftm_freq_scale;

    /* HB min <= T <= HB MAX, 1 HB */
    if (capture_value <= DALI_PHY_HB_PERIOD_MAX && capture_value >= DALI_PHY_HB_PERIOD_MIN) {
        return 1;
    }

    /* 2HB min <= T <= 2HB max, 2 HB */
    if (capture_value <= DALI_PHY_DHB_PERIOD_MAX && capture_value >= DALI_PHY_DHB_PERIOD_MIN) {
        return 2;
    }

    /* Other condition, invalid timing */
    return 0;
}

/**
 * @brief Enable or disable counter reset on TXHB match event.
 * Useful when STOP condition needs to be generated or in RX mode.
 *
 * @param enable true for enable, false for disable
 */
static inline void dali_phy_ftm_channel_reset_counter(ftm_chnl_t ch, bool enable) {
    if (enable) {
        DALI_PHY_FTM_INSTANCE->CONTROLS[ch].CnSC |= FTM_CnSC_ICRST_MASK;
    } else {
        DALI_PHY_FTM_INSTANCE->CONTROLS[ch].CnSC &= ~FTM_CnSC_ICRST_MASK;
    }
}

/**
 * @brief Enable Match interrupts for TX mode, disable Capture interrupts.
 *
 */
static inline void dali_phy_ftm_tx_mode(void) {
    /* Disable counter reset on capture channel */
    dali_phy_ftm_channel_reset_counter(DALI_PHY_FTM_CH_CAPTURE, false);

    /* Disable capture interrupts */
    FTM_DisableInterrupts(DALI_PHY_FTM_INSTANCE, DALI_PHY_FTM_CH_CAPTURE_INT_MASK);

    /* Enable match interrupts */
    FTM_EnableInterrupts(DALI_PHY_FTM_INSTANCE, DALI_PHY_FTM_CH_MATCH_TX_HB_INT_MASK);
}

/**
 * @brief Enable Capture interrupts for RX mode, disable TXHB Match interrupts.
 *
 */
static inline void dali_phy_ftm_rx_mode(void) {
    /* Enable counter reset on capture channel */
    dali_phy_ftm_channel_reset_counter(DALI_PHY_FTM_CH_CAPTURE, true);

    /* Disable match interrupts */
    FTM_DisableInterrupts(DALI_PHY_FTM_INSTANCE, DALI_PHY_FTM_CH_MATCH_TX_HB_INT_MASK);

    /* Enable capture interrupts */
    FTM_EnableInterrupts(DALI_PHY_FTM_INSTANCE, DALI_PHY_FTM_CH_CAPTURE_INT_MASK);
}

/**
 * @brief Set TX line value, true for high level.
 *
 * @param val value to be set
 */
static inline void dali_phy_ftm_set_tx(bool val) {
#if DALI_PHY_FTM_INVERSE_TX_LOGIC
    if (val) {
        GPIO_PortClear(DALI_PHY_FTM_TX_GPIO_INSTANCE, DALI_PHY_FTM_TX_PORT, DALI_PHY_FTM_TX_PIN_MASK);
    } else {
        GPIO_PortSet(DALI_PHY_FTM_TX_GPIO_INSTANCE, DALI_PHY_FTM_TX_PORT, DALI_PHY_FTM_TX_PIN_MASK);
    }
#else
    if (val) {
        GPIO_PortSet(DALI_PHY_FTM_TX_GPIO_INSTANCE, DALI_PHY_FTM_TX_PORT, DALI_PHY_FTM_TX_PIN_MASK);
    } else {
        GPIO_PortClear(DALI_PHY_FTM_TX_GPIO_INSTANCE, DALI_PHY_FTM_TX_PORT, DALI_PHY_FTM_TX_PIN_MASK);
    }
#endif
}

/**
 * @brief Get RX line value
 *
 * @return true for high level, false for low level.
 */
static inline bool dali_phy_ftm_get_rx(void) {
#if DALI_PHY_FTM_INVERSE_RX_LOGIC
    if (GPIO_PortRead(DALI_PHY_FTM_RX_GPIO_INSTANCE, DALI_PHY_FTM_RX_PORT) & DALI_PHY_FTM_RX_PIN_MASK) {
        return false;
    }

    return true;
#else
    if (GPIO_PortRead(DALI_PHY_FTM_RX_GPIO_INSTANCE, DALI_PHY_FTM_RX_PORT) & DALI_PHY_FTM_RX_PIN_MASK) {
        return true;
    }

    return false;
#endif
}