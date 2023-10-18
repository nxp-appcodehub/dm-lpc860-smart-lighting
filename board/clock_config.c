/*
* Copyright 2023 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*
 * How to set up clock using clock driver functions:
 *
 * 1. Setup clock sources.
 *
 * 2. Set up all dividers.
 *
 * 3. Set up all selectors to provide selected clocks.
 */

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Clocks v11.0
processor: LPC865
package_id: LPC865M201JBD64
mcu_data: ksdk2_0
processor_version: 13.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

#include "fsl_power.h"
#include "fsl_clock.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/
void BOARD_InitBootClocks(void)
{
    BOARD_BootClockPLL48M();
}

/*******************************************************************************
 ******************** Configuration BOARD_BootClockFRO60M **********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockFRO60M
outputs:
- {id: FROHF_clock.outFreq, value: 60 MHz}
- {id: System_clock.outFreq, value: 60 MHz}
- {id: WKT_clock.outFreq, value: 60 MHz}
settings:
- {id: SYSCON.FRG0_DIV.scale, value: '320'}
- {id: SYSCON.FRG1_DIV.scale, value: '320'}
- {id: SYSCON.FRO_DIRECT.sel, value: SYSCON.fro_osc}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockFRO60M configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockFRO60M configuration
 ******************************************************************************/
void BOARD_BootClockFRO60M(void)
{
    /*!< Set up the clock sources */
    /*!< Set up FRO */
    POWER_DisablePD(kPDRUNCFG_PD_FRO_OUT);                      /*!< Ensure FRO is on  */
    POWER_DisablePD(kPDRUNCFG_PD_FRO);                          /*!< Ensure FRO is on  */
    CLOCK_SetFroOscFreq(kCLOCK_FroOscOut60M);                   /*!< Set up FRO freq */
    CLOCK_SetFroOutClkSrc(kCLOCK_FroSrcFroOsc);                 /*!< Set FRO clock source */
    CLOCK_Select(kFRG0_Clk_From_Fro);                           /*!< select fro for frg0 */
    CLOCK_SetFRG0ClkFreq(48000000U);                            /*!< select frg0 freq */
    CLOCK_Select(kFRG1_Clk_From_Fro);                           /*!< select fro for frg1 */
    CLOCK_SetFRG1ClkFreq(48000000U);                            /*!< select frg1 freq */
    CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcFro);                  /*!< select fro for main clock */
    CLOCK_Select(kCLKOUT_From_Fro);                             /*!< select FRO for CLKOUT */
    CLOCK_Select(kADC_Clk_From_Fro);                            /*!< select FRO for ADC */
    CLOCK_Select(kWKT_Clk_From_Fro);                            /*!< select FRO for WKT */
    CLOCK_SetCoreSysClkDiv(1U);
    /*!< Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKFRO60M_CORE_CLOCK;
}

/*******************************************************************************
 ******************** Configuration BOARD_BootClockPLL48M **********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockPLL48M
called_from_default_init: true
outputs:
- {id: SYSPLL_clock.outFreq, value: 48 MHz}
- {id: System_clock.outFreq, value: 48 MHz}
settings:
- {id: SYSCON.FRG0CLKSEL.sel, value: NO_CLOCK}
- {id: SYSCON.FRG1CLKSEL.sel, value: NO_CLOCK}
- {id: SYSCON.MAINCLKPLLSEL.sel, value: SYSCON.PLL}
- {id: SYSCON.MAINCLKSEL.sel, value: SYSCON.EXTCLKSEL}
- {id: SYSCON.M_MULT.scale, value: '4'}
- {id: SYSCON.SYSPLLCLKSEL.sel, value: SYSCON.EXTCLKSEL}
- {id: SYSCON_PDRUNCFG0_PDEN_FRO_HF_CFG, value: Power_down}
- {id: SYSCON_PDRUNCFG0_PDEN_MAIN_SOC_CFG, value: Power_up}
- {id: SYSCON_PDRUNCFG0_PDEN_PLL_CFG, value: Power_up}
sources:
- {id: SYSCON.sys_osc.outFreq, value: 12 MHz, enabled: true}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockPLL48M configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockPLL48M configuration
 ******************************************************************************/
void BOARD_BootClockPLL48M(void)
{
    /*!< Set up the clock sources */
    POWER_DisablePD(kPDRUNCFG_PD_SYSOSC);                       /*!< Ensure Main osc is on */
    CLOCK_InitSysOsc(12000000U);                                /*!< Set main osc freq */
    CLOCK_Select(kEXT_Clk_From_SysOsc);                         /*!< select external clock source to sys_osc */
    clock_sys_pll_t config;
    config.src = kCLOCK_SysPllSrcExtClk;                        /*!< select main osc or mclkin for SYSPLL */
    config.targetFreq = 48000000U;                              /*!< set pll target freq */
    CLOCK_InitSystemPll(&config);                               /*!< set parameters */
    CLOCK_SetClkDivider(kCLOCK_DivPllClk, 1U);                  /*!< set SYSPLL div */
    CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcSysPll);               /*!< select syspll for main clock */
    CLOCK_Select(kCLKOUT_From_Fro);                             /*!< select FRO for CLKOUT */
    CLOCK_Select(kADC_Clk_From_Fro);                            /*!< select FRO for ADC */
    CLOCK_Select(kWKT_Clk_From_Fro);                            /*!< select FRO for WKT */
    CLOCK_SetCoreSysClkDiv(1U);
    /*!< Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKPLL48M_CORE_CLOCK;
}

/*******************************************************************************
 ******************** Configuration BOARD_BootClockFRO30M **********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockFRO30M
outputs:
- {id: FROHF_clock.outFreq, value: 60 MHz}
- {id: System_clock.outFreq, value: 30 MHz}
- {id: WKT_clock.outFreq, value: 30 MHz}
settings:
- {id: SYSCON.FRG0_DIV.scale, value: '320'}
- {id: SYSCON.FRG1_DIV.scale, value: '320'}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockFRO30M configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockFRO30M configuration
 ******************************************************************************/
void BOARD_BootClockFRO30M(void)
{
    /*!< Set up the clock sources */
    /*!< Set up FRO */
    POWER_DisablePD(kPDRUNCFG_PD_FRO_OUT);                      /*!< Ensure FRO is on  */
    POWER_DisablePD(kPDRUNCFG_PD_FRO);                          /*!< Ensure FRO is on  */
    CLOCK_SetFroOscFreq(kCLOCK_FroOscOut60M);                   /*!< Set up FRO freq */
    CLOCK_SetFroOutClkSrc(kCLOCK_FroSrcFroOscDiv);              /*!< Set FRO clock source */
    CLOCK_Select(kFRG0_Clk_From_Fro);                           /*!< select fro for frg0 */
    CLOCK_SetFRG0ClkFreq(24000000U);                            /*!< select frg0 freq */
    CLOCK_Select(kFRG1_Clk_From_Fro);                           /*!< select fro for frg1 */
    CLOCK_SetFRG1ClkFreq(24000000U);                            /*!< select frg1 freq */
    CLOCK_SetMainClkSrc(kCLOCK_MainClkSrcFro);                  /*!< select fro for main clock */
    CLOCK_Select(kCLKOUT_From_Fro);                             /*!< select FRO for CLKOUT */
    CLOCK_Select(kADC_Clk_From_Fro);                            /*!< select FRO for ADC */
    CLOCK_Select(kWKT_Clk_From_Fro);                            /*!< select FRO for WKT */
    CLOCK_SetCoreSysClkDiv(1U);
    /*!< Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKFRO30M_CORE_CLOCK;
}

