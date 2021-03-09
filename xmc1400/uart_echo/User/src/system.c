/*
 * system.c
 *
 *  Created on: Mar 9, 2021
 *      Author: pjh
 */

#include <xmc_scu.h>

void SystemCoreSetup(void)
{
#if UC_SERIES == XMC14
  /* Enable Prefetch unit */
  SCU_GENERAL->PFUCR &= ~SCU_GENERAL_PFUCR_PFUBYP_Msk;
#endif
}


void SystemCoreClockSetup(void)
{
/* LOCAL DATA STRUCTURES */
const XMC_SCU_CLOCK_CONFIG_t CLOCK_XMC1_0_CONFIG =
{
  .pclk_src = XMC_SCU_CLOCK_PCLKSRC_DOUBLE_MCLK,
  .rtc_src = XMC_SCU_CLOCK_RTCCLKSRC_DCO2,
  .fdiv = 0U,  /**< 8/10 Bit Fractional divider */
  .idiv = 1U,  /**< 8 Bit integer divider */

  .dclk_src = XMC_SCU_CLOCK_DCLKSRC_EXT_XTAL,
  .oschp_mode = XMC_SCU_CLOCK_OSCHP_MODE_OSC,
  .osclp_mode = XMC_SCU_CLOCK_OSCLP_MODE_DISABLED

};

  /* Configure FDIV, IDIV, PCLKSEL dividers*/
  XMC_SCU_CLOCK_Init(&CLOCK_XMC1_0_CONFIG);
}
