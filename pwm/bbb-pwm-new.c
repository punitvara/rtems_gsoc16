/**
 *  \file   bbb-pwm.c
 *
 *  \brief  This file contains the device abstraction layer APIs for EHRPWM.
 */

/*
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 */
/*
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *   
 */


#include<libcpu/am335x.h>
#include<stdio.h>
#include<bsp/gpio.h>
#include<bsp/bbb-gpio.h>
#include<bsp.h>


/**
 * @brief This function intilize clock and pinmuxing for pwm sub system.
 *
 * @param PWMSS_ID It is the instance number of EPWM of pwm sub system.
 **/
void pwm_init(unsigned int baseAddr, unsigned int PWMSS_ID)
{
  PWMSSModuleClkConfig(PWMSS_ID);
  EPWMPinMuxSetup();
  EHRPWMClockEnable(baseAddr);
   PWMSSTBClkEnable(PWMSS_ID);

}   


/**
 * \brief   This function Enables TBCLK(Time Base Clock) for specific
 *          EPWM instance of pwmsubsystem.
 *
 * \param   instance  It is the instance number of EPWM of pwmsubsystem.
 *
 **/
void PWMSSTBClkEnable(unsigned int instance)
{
	switch(instance)
	{

		case 0:
			REG(AM335X_PADCONF_BASE + CONTROL_PWMSS_CTRL) |=
				BBBIO_PWMSS_CTRL_PWMSS0_TBCLKEN;
			break;

		case 1:
			REG(AM335X_PADCONF_BASE + CONTROL_PWMSS_CTRL) |=
				BBBIO_PWMSS_CTRL_PWMSS1_TBCLKEN;
			break;

		case 2:
			REG(AM335X_PADCONF_BASE + CONTROL_PWMSS_CTRL) |=
				BBBIO_PWMSS_CTRL_PWMSS2_TBCLKEN;
			break;

		default:
			break;
	}
}

/**
 * \brief   This function Enables pinmuxing for PWM module.
 *          
 *
 * \param   instance  It is the instance number of EPWM of pwmsubsystem.
 *
 **/



void EPWMPinMuxSetup(void)
{	
  REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(9)) = BBB_MUXMODE(4);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(8)) = BBB_MUXMODE(4);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(0)) = BBB_MUXMODE(3);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(1)) = BBB_MUXMODE(3);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(11)) = BBB_MUXMODE(2);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(10)) = BBB_MUXMODE(2);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(2)) = BBB_MUXMODE(6);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(3)) = BBB_MUXMODE(6);

  REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_D0) = BBB_MUXMODE(3);

  REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_SCLK) = BBB_MUXMODE(3);

  REG(AM335X_PADCONF_BASE + AM335X_CONF_MCASP0_FSX) = BBB_MUXMODE(1);

  REG(AM335X_PADCONF_BASE + AM335X_CONF_MCASP0_ACLKX) = BBB_MUXMODE(1);
}




/**
 * \brief   This functions enables clock for EHRPWM module in PWMSS subsystem.
 *
 * \param   baseAdd   It is the Memory address of the PWMSS instance used.
 *
 * \return  None.
 *
 **/

void EHRPWMClockEnable(unsigned int baseAdd)
{
	REG(baseAdd + PWMSS_CLKCONFIG) |= PWMSS_CLK_EN_ACK;
}


/**
 * \brief   This function configures the L3 and L4_PER system clocks.
 *          It also configures the system clocks for the specified ePWMSS
 *          instance.
 *
 * \param   instanceNum    The instance number of ePWMSS whose system clocks
 *                         have to be configured.
 *
 * 'instanceNum' can take one of the following values:
 * (0 <= instanceNum <= 2)
 *
 * \return  None.
 *
 */
void PWMSSModuleClkConfig(unsigned int instanceNum)
{
/*
	REG(SOC_PRCM_REGS + CM_PER_L3S_CLKSTCTRL) |=
		CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

	while((REG(SOC_PRCM_REGS + CM_PER_L3S_CLKSTCTRL) &
				CM_PER_L3S_CLKSTCTRL_CLKTRCTRL) != CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

	REG(SOC_PRCM_REGS + CM_PER_L3_CLKSTCTRL) |=
		CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

	while((REG(SOC_PRCM_REGS + CM_PER_L3_CLKSTCTRL) &
				CM_PER_L3_CLKSTCTRL_CLKTRCTRL) != CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

	REG(SOC_PRCM_REGS + CM_PER_L3_INSTR_CLKCTRL) |=
		CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;
	while((REG(SOC_PRCM_REGS + CM_PER_L3_INSTR_CLKCTRL) &
				CM_PER_L3_INSTR_CLKCTRL_MODULEMODE) !=
			CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE);

	REG(SOC_PRCM_REGS + CM_PER_L3_CLKCTRL) |=
		CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

	while((REG(SOC_PRCM_REGS + CM_PER_L3_CLKCTRL) &
				CM_PER_L3_CLKCTRL_MODULEMODE) != CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE);

	REG(SOC_PRCM_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) |=
		CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

	while((REG(SOC_PRCM_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
				CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL) !=
			CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

	REG(SOC_PRCM_REGS + CM_PER_L4LS_CLKSTCTRL) |=
		CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
	while((REG(SOC_PRCM_REGS + CM_PER_L4LS_CLKSTCTRL) &
				CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL) !=
			CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

	REG(SOC_PRCM_REGS + CM_PER_L4LS_CLKCTRL) |=
		CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE;

	while((REG(SOC_PRCM_REGS + CM_PER_L4LS_CLKCTRL) &
				CM_PER_L4LS_CLKCTRL_MODULEMODE) != CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE);
*/
	if(0 == instanceNum)
	{
		REG(BBBIO_CM_PER_ADDR + CM_PER_EPWMSS0_CLKCTRL) |=
			CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_ENABLE;

	//	while(CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_ENABLE !=
	//			(REG(SOC_PRCM_REGS + CM_PER_EPWMSS0_CLKCTRL) &
	//			 CM_PER_EPWMSS0_CLKCTRL_MODULEMODE));
	//	while((CM_PER_EPWMSS0_CLKCTRL_IDLEST_FUNC <<
	//				CM_PER_EPWMSS0_CLKCTRL_IDLEST_SHIFT) !=
	//			(REG(SOC_PRCM_REGS + CM_PER_EPWMSS0_CLKCTRL) &
	//			 CM_PER_EPWMSS0_CLKCTRL_IDLEST));

	}
	else if(1 == instanceNum)
	{
		REG(BBBIO_CM_PER_ADDR + CM_PER_EPWMSS1_CLKCTRL) |=
			CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_ENABLE;

	//	while(CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_ENABLE !=
	//			(REG(SOC_PRCM_REGS + CM_PER_EPWMSS1_CLKCTRL) &
	///			 CM_PER_EPWMSS1_CLKCTRL_MODULEMODE));

	//	while((CM_PER_EPWMSS1_CLKCTRL_IDLEST_FUNC <<
	//				CM_PER_EPWMSS1_CLKCTRL_IDLEST_SHIFT) !=
	//			(REG(SOC_PRCM_REGS + CM_PER_EPWMSS1_CLKCTRL) &
	//			 CM_PER_EPWMSS1_CLKCTRL_IDLEST));

	}
	else if(2 == instanceNum)
	{
		REG(BBBIO_CM_PER_ADDR + CM_PER_EPWMSS2_CLKCTRL) |=
			CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_ENABLE;

	//	while(CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_ENABLE !=
	//			(REG(SOC_PRCM_REGS + CM_PER_EPWMSS2_CLKCTRL) &
	//			 CM_PER_EPWMSS2_CLKCTRL_MODULEMODE));
//
//		while((CM_PER_EPWMSS2_CLKCTRL_IDLEST_FUNC <<
//					CM_PER_EPWMSS2_CLKCTRL_IDLEST_SHIFT) !=
//				(REG(SOC_PRCM_REGS + CM_PER_EPWMSS2_CLKCTRL) &
//				 CM_PER_EPWMSS2_CLKCTRL_IDLEST));
	}
	else
	{

	} /*
	while(!(REG(SOC_PRCM_REGS + CM_PER_L3S_CLKSTCTRL) &
				CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK));

	while(!(REG(SOC_PRCM_REGS + CM_PER_L3_CLKSTCTRL) &
				CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));

	while(!(REG(SOC_PRCM_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
				(CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK |
				 CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L4_GCLK)));

	while(!(REG(SOC_PRCM_REGS + CM_PER_L4LS_CLKSTCTRL) &
				(CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK )));
*/
}

/**
 * \brief  This API configures the clock divider of the Time base module.
 *         The clock divider can be calculated using the equation
 *         TBCLK = SYSCLKOUT/(HSPCLKDIV × CLKDIV)
 *
 * \param   baseAddr      Base Address of the PWM Module Registers.
 * \param   tbClk         Timebase clock to be generated.
 * \param   moduleClk     Input clock of the PWM module (sysclk2)
 *
 * \return  None.
 *
 **/

void EHRPWMTimebaseClkConfig(unsigned int baseAddr,
		unsigned int tbClk,
		unsigned int moduleClk)

{
	unsigned int clkDiv = moduleClk/tbClk;
	unsigned int hspClkDiv;
	unsigned int lspClkDiv, lspClkDivSetting = 0;

	if(clkDiv > EHRPWM_TBCTL_HSPCLKDIV_14)
	{
		hspClkDiv = EHRPWM_TBCTL_HSPCLKDIV_DIVBY14; /* reg setting */
		lspClkDiv = clkDiv/EHRPWM_TBCTL_HSPCLKDIV_14; /* divider */
		/* reg setting */
		while(lspClkDiv > 1)
		{
			lspClkDiv = lspClkDiv >> 1;
			lspClkDivSetting++;
		}
	printf ("lspclkdiv setting = %d hspclkdiv = %d \n",lspClkDivSetting,hspClkDiv);
	}
	else
	{
		hspClkDiv = clkDiv/2; /* reg setting */
		/* divide by 1 */
		lspClkDivSetting = EHRPWM_TBCTL_HSPCLKDIV_DIVBY1;
	printf ("lspclkdiv setting = %d hspclkdiv = %d \n",lspClkDivSetting,hspClkDiv);

	}
	
	REG16(baseAddr + EHRPWM_TBCTL) = (REG16(baseAddr + EHRPWM_TBCTL) &
            (~EHRPWM_TBCTL_CLKDIV)) | ((lspClkDivSetting <<
            EHRPWM_TBCTL_CLKDIV_SHIFT) & EHRPWM_TBCTL_CLKDIV);

        REG16(baseAddr + EHRPWM_TBCTL) = (REG16(baseAddr + EHRPWM_TBCTL) &
            (~EHRPWM_TBCTL_HSPCLKDIV)) | ((hspClkDiv <<
            EHRPWM_TBCTL_HSPCLKDIV_SHIFT) & EHRPWM_TBCTL_HSPCLKDIV);


}

/**
 * \brief  This API configures the PWM Frequency/Period. The period count
 *         determines the period of the final output waveform. For the given
 *         period count, in the case of UP and DOWN counter the count value will
 *         be loaded as is. In the case of UP_DOWN counter the count is halfed.
 *
 * \param   baseAddr     Base Address of the PWM Module Registers.
 * \param   tbClk        Timebase clock.
 * \param   pwmFreq      Frequency of the PWM Output. If the counter direction
 *                       is up-down this value has to be halfed, so that the
 *                       period of the final output is equal to pwmFreq.
 *
 * \param   counterDir           Direction of the counter(up, down, up-down)
 * \param   enableShadowWrite    Whether write to Period register is to be shadowed
 *
 * \return  None.
 *
 **/
void EHRPWMPWMOpFreqSet(unsigned int baseAddr,
                        unsigned int tbClk,
                        unsigned int pwmFreq,
                        unsigned int counterDir,
                        bool enableShadowWrite)
{
     unsigned int tbPeriodCount = tbClk/pwmFreq;

     REG16(baseAddr + EHRPWM_TBCTL) = (REG16(baseAddr + EHRPWM_TBCTL) &
             (~EHRPWM_PRD_LOAD_SHADOW_MASK)) | ((enableShadowWrite <<
            EHRPWM_TBCTL_PRDLD_SHIFT) & EHRPWM_PRD_LOAD_SHADOW_MASK);

     REG16(baseAddr + EHRPWM_TBCTL) = (REG16(baseAddr + EHRPWM_TBCTL) &
             (~EHRPWM_COUNTER_MODE_MASK)) | ((counterDir <<
            EHRPWM_TBCTL_CTRMODE_SHIFT) &  EHRPWM_COUNTER_MODE_MASK);
	
     if(EHRPWM_COUNT_UP_DOWN == counterDir)
     {
	  REG16(baseAddr + EHRPWM_TBPRD) = (unsigned short)tbPeriodCount/2;
	  printf("\n tbperiod count is %u \n", tbPeriodCount/2);
     }
     else
     {   
	 REG16(baseAddr + EHRPWM_TBPRD) = (unsigned short)tbPeriodCount;
	 printf("\n tberiod count is %u \n",tbPeriodCount);
     }

printf("\n frequency function is running \n");
}



void EHRPWMTimebaseSyncDisable(unsigned int baseAddr)
{
	REG16(baseAddr + EHRPWM_TBCTL) &= (~EHRPWM_SYNC_ENABLE);
}

/**
 * \brief   This API selects the output sync source. It determines on which of
 *          the following event sync-out has to be generated.
 *
 * \param   baseAddr        Base Address of the PWM Module Registers.
 * \param   syncOutMode     Sync out mode. Possible values are,
 *                          - EHRPWM_SYNCOUT_SYNCIN \n
 *                          - EHRPWM_SYNCOUT_COUNTER_EQUAL_ZERO \n
 *                          - EHRPWM_SYNCOUT_COUNTER_EQUAL_COMPAREB \n
 *                          - EHRPWM_SYNCOUT_DISABLE \n
 * \return  None.
 *
 **/
void EHRPWMSyncOutModeSet(unsigned int baseAddr, unsigned int syncOutMode)
{
	REG16(baseAddr + EHRPWM_TBCTL) = (REG16(baseAddr + EHRPWM_TBCTL) &
			(~EHRPWM_SYNCOUT_MASK)) | syncOutMode;
}

/**
 * \brief  This API configures emulation mode. This setting determines
 *         the behaviour of Timebase during emulation (debugging).
 *
 * \param   baseAddr      Base Address of the PWM Module Registers.
 * \param   mode          Emulation mode. Possible values are,
 *                        - EHRPWM_STOP_AFTER_NEXT_TB_INCREMENT \n
 *                        - EHRPWM_STOP_AFTER_A_COMPLETE_CYCLE \n
 *                        - EHRPWM_FREE_RUN \n
 * \return  None.
 *
 **/
void EHRPWMTBEmulationModeSet(unsigned int baseAddr, unsigned int mode)
{
	REG16(baseAddr + EHRPWM_TBCTL) = (REG16(baseAddr + EHRPWM_TBCTL) &
			(~EHRPWM_TBCTL_FREE_SOFT)) | (mode & EHRPWM_TBCTL_FREE_SOFT);
}

/**
 * \brief   This API loads the CMPA value. When CMPA value equals the counter
 *          value, then an event is generated both in the up direction and
 *          down direction.
 *
 * \param   baseAddr                   Base Address of the PWM Module Registers.
 * \param   CMPAVal                    CMPA value to be loaded.
 * \param   enableShadowWrite          Enable write to shadow register.
 * \param   ShadowToActiveLoadTrigger  Shadow to active register load trigger.
 * \param   OverwriteShadowFull        Overwrite even if previous value is not
 *                                     loaded to active register.
 *
 * \return  bool    Flag indicates whether the CMPA value is
 *                  written or not.
 *
 **/
bool EHRPWMLoadCMPA(unsigned int baseAddr,
		unsigned int CMPAVal,
		bool enableShadowWrite,
		unsigned int ShadowToActiveLoadTrigger,
		bool OverwriteShadowFull)
{
	bool status = FALSE;

	if((OverwriteShadowFull) ||
			((REG16(baseAddr+EHRPWM_CMPCTL) & EHRPWM_CMPCTL_SHDWAFULL) ==
			 EHRPWM_SHADOW_A_EMPTY))
	{
		REG16(baseAddr + EHRPWM_CMPCTL) = (REG16(baseAddr + EHRPWM_CMPCTL) &
				(~EHRPWM_CMPCTL_SHDWAMODE)) | ((enableShadowWrite <<
						EHRPWM_CMPCTL_SHDWAMODE_SHIFT) & EHRPWM_CMPCTL_SHDWAMODE);

		REG16(baseAddr + EHRPWM_CMPCTL) = (REG16(baseAddr + EHRPWM_CMPCTL) &
				(~EHRPWM_COMPA_LOAD_MASK)) |((ShadowToActiveLoadTrigger <<
						EHRPWM_CMPCTL_LOADAMODE_SHIFT) & EHRPWM_COMPA_LOAD_MASK);

		REG16(baseAddr + EHRPWM_CMPA) = CMPAVal & EHRPWM_CMPA_CMPA;

		status = TRUE;
	}

	return status;
}
/**
 * \brief  This API loads the CMPB value. When CMPB value equals the counter
 *         value, then an event is generated both in the up direction and
 *         down direction.
 *
 * \param   baseAddr                   Base Address of the PWM Module Registers.
 * \param   CMPBVal                    CMPB value to be loaded.
 * \param   enableShadowWrite          Enable write to shadow register.
 * \param   ShadowToActiveLoadTrigger  Shadow to active register load trigger.
 * \param   OverwriteShadowFull        Overwrite even if previous value is not
 *                                     loaded to active register.
 *
 * \return  bool   Flag indicates whether the CMPB value is
 *                 written or not.
 *
 **/
bool EHRPWMLoadCMPB(unsigned int baseAddr,
		unsigned int CMPBVal,
		bool enableShadowWrite,
		unsigned int ShadowToActiveLoadTrigger,
		bool OverwriteShadowFull)
{
	bool status = FALSE;

	if((OverwriteShadowFull) ||
			((REG16(baseAddr+EHRPWM_CMPCTL) & EHRPWM_CMPCTL_SHDWBFULL) ==
			 EHRPWM_SHADOW_B_EMPTY))
	{
		REG16(baseAddr + EHRPWM_CMPCTL) = (REG16(baseAddr + EHRPWM_CMPCTL)
				& (~EHRPWM_CMPCTL_SHDWBMODE)) | ((enableShadowWrite <<
					EHRPWM_CMPCTL_SHDWBMODE_SHIFT) & EHRPWM_CMPCTL_SHDWBMODE);

		REG16(baseAddr + EHRPWM_CMPCTL) = (REG16(baseAddr + EHRPWM_CMPCTL) &
				(~EHRPWM_COMPB_LOAD_MASK)) | ((ShadowToActiveLoadTrigger <<
						EHRPWM_CMPCTL_LOADBMODE_SHIFT) & EHRPWM_COMPB_LOAD_MASK);

		REG16(baseAddr + EHRPWM_CMPB) = CMPBVal & EHRPWM_CMPB_CMPB;

		status = TRUE;
	}
	return status;
}
/**
 * \brief  his API configures the action to be taken on B by the Action
 *         qualifier module upon receiving the events. This will determine
 *         the output waveform.
 *
 * \param   zero      Action to be taken when CTR = 0
 * \param   period    Action to be taken when CTR = PRD
 * \param   CAUp      Action to be taken when CTR = CAUp
 * \param   CADown    Action to be taken when CTR = CADown
 * \param   CBUp      Action to be taken when CTR = CBUp
 * \param   CBDown    Action to be taken when CTR = CBDown
 * \param   SWForced  Action to be taken when SW forced event has been generated
 *
 *     Possible values for the actions are
 *         - EHRPWM_XXXX_XXXX_DONOTHING \n
 *         - EHRPWM_XXXX_XXXX_CLEAR \n
 *         - EHRPWM_XXXX_XXXX_SET \n
 *         - EHRPWM_XXXX_XXXX_TOGGLE \n
 *
 * \return  None
 *
 **/
void EHRPWMConfigureAQActionOnB(unsigned int baseAddr,
		unsigned int zero,
		unsigned int period,
		unsigned int CAUp,
		unsigned int CADown,
		unsigned int CBUp,
		unsigned int CBDown,
		unsigned int SWForced)
{
	REG16(baseAddr + EHRPWM_AQCTLB) =
		((CBDown << EHRPWM_AQCTLB_CBD_SHIFT) & EHRPWM_AQCTLB_CBD) |
		((CBUp << EHRPWM_AQCTLB_CBU_SHIFT) & EHRPWM_AQCTLB_CBU) |
		((CADown << EHRPWM_AQCTLB_CAD_SHIFT) & EHRPWM_AQCTLB_CAD) |
		((CAUp << EHRPWM_AQCTLB_CAU_SHIFT) & EHRPWM_AQCTLB_CAU) |
		((period << EHRPWM_AQCTLB_PRD_SHIFT) & EHRPWM_AQCTLB_PRD) |
		((zero << EHRPWM_AQCTLB_ZRO_SHIFT) & EHRPWM_AQCTLB_ZRO);


	REG16(baseAddr + EHRPWM_AQSFRC) =
		(REG16(baseAddr + EHRPWM_AQSFRC) & (~EHRPWM_AQSFRC_ACTSFB)) |
		((SWForced << EHRPWM_AQSFRC_ACTSFB_SHIFT) & EHRPWM_AQSFRC_ACTSFB);
}
/**
 * \brief  This API configures the action to be taken on A by the Action
 *         qualifier module upon receiving the events. This will determine
 *         the output waveform.
 *
 * \param   zero      Action to be taken when CTR = 0
 * \param   period    Action to be taken when CTR = PRD
 * \param   CAUp      Action to be taken when CTR = CAUp
 * \param   CADown    Action to be taken when CTR = CADown
 * \param   CBUp      Action to be taken when CTR = CBUp
 * \param   CBDown    Action to be taken when CTR = CBDown
 * \param   SWForced  Action to be taken when SW forced event has been generated
 *
 *     Possible values for the actions are
 *          - EHRPWM_XXXX_XXXX_DONOTHING \n
 *          - EHRPWM_XXXX_XXXX_CLEAR \n
 *          - EHRPWM_XXXX_XXXX_SET \n
 *          - EHRPWM_XXXX_XXXX_TOGGLE \n
 *
 * \return  None
 *
 **/
void EHRPWMConfigureAQActionOnA(unsigned int baseAddr,
		unsigned int zero,
		unsigned int period,
		unsigned int CAUp,
		unsigned int CADown,
		unsigned int CBUp,
		unsigned int CBDown,
		unsigned int SWForced)
{
	REG16(baseAddr + EHRPWM_AQCTLA) =
		((CBDown << EHRPWM_AQCTLA_CBD_SHIFT) & EHRPWM_AQCTLA_CBD) |
		((CBUp << EHRPWM_AQCTLA_CBU_SHIFT) & EHRPWM_AQCTLA_CBU) |
		((CADown << EHRPWM_AQCTLA_CAD_SHIFT) & EHRPWM_AQCTLA_CAD) |
		((CAUp << EHRPWM_AQCTLA_CAU_SHIFT) & EHRPWM_AQCTLA_CAU) |
		((period << EHRPWM_AQCTLA_PRD_SHIFT) & EHRPWM_AQCTLA_PRD) |
		((zero << EHRPWM_AQCTLA_ZRO_SHIFT) & EHRPWM_AQCTLA_ZRO);


	REG16(baseAddr + EHRPWM_AQSFRC) = (REG16(baseAddr + EHRPWM_AQSFRC) &
			(~EHRPWM_AQSFRC_ACTSFA)) | ((SWForced <<
					EHRPWM_AQSFRC_ACTSFA_SHIFT) & EHRPWM_AQSFRC_ACTSFA);
}


/**
 * \brief   This API selects output mode. This allows to selectively enable or
 *          bypass the dead-band generation for the falling-edge and rising-edge
 *          delay.
 *
 * \param   baseAddr      Base Address of the PWM Module Registers.
 * \param   DBgenOpMode   Output mode. The possible values can be :
 *                        - EHRPWM_DBCTL_OUT_MODE_BYPASS \n
 *                        - EHRPWM_DBCTL_OUT_MODE_NOREDBFED \n
 *                        - EHRPWM_DBCTL_OUT_MODE_AREDNOFED \n
 *                        - EHRPWM_DBCTL_OUT_MODE_AREDBFED \n
 * \return  None
 *
 **/
void EHRPWMDBOutput(unsigned int baseAddr, unsigned int DBgenOpMode)
{
	REG16(baseAddr + EHRPWM_DBCTL) =
		(REG16(baseAddr + EHRPWM_DBCTL) & (~EHRPWM_DBCTL_OUT_MODE)) |
		((DBgenOpMode << EHRPWM_DBCTL_OUT_MODE_SHIFT) & EHRPWM_DBCTL_OUT_MODE);
}

/**
 * \brief   This API disables the PWM chopper sub-module. This will cause the
 *          chopper module to be by-passed.
 *
 * \param   baseAddr    Base Address of the PWM Module Registers.
 *
 * \return  None
 *
 **/
void EHRPWMChopperDisable(unsigned int baseAddr)
{
	REG16(baseAddr + EHRPWM_PCCTL) &= (~EHRPWM_PCCTL_CHPEN);
}

/**
 * \brief   This API disable the trip event. The trip events will be ignored.
 *
 * \param   baseAddr    Base Address of the PWM Module Registers.
 * \param   osht_CBC    Disable OST or CBC event
 *
 * \return  None
 *
 **/
void EHRPWMTZTripEventDisable(unsigned int baseAddr, bool osht_CBC)
{
	if(EHRPWM_TZ_ONESHOT == osht_CBC)
	{
		REG16(baseAddr + EHRPWM_TZSEL) &= (~EHRPWM_TZSEL_OSHT1);
	}
	if(EHRPWM_TZ_CYCLEBYCYCLE == osht_CBC)
	{
		REG16(baseAddr + EHRPWM_TZSEL) &= (~EHRPWM_TZSEL_CBC1);
	}
}

/**
 * \brief   This API prescales the event on which interrupt is to be generated
 *
 * \param   baseAddr    Base Address of the PWM Module Registers.
 * \param   prescale    prescalar value
 *
 * \return  None
 *
 **/

void EHRPWMETIntPrescale(unsigned int baseAddr, unsigned int prescale)
{
	REG16(baseAddr + EHRPWM_ETPS) =
		(REG16(baseAddr + EHRPWM_ETPS) & (~EHRPWM_ETPS_INTPRD)) |
		((prescale << EHRPWM_ETPS_INTPRD_SHIFT) & EHRPWM_ETPS_INTPRD);
}

/**
 * \brief   This API selects the interrupt source.
 *
 * \param   baseAddr    Base Address of the PWM Module Registers.
 * \param   selectInt   Event which triggers interrupt. The possible source can be,
 *                      - EHRPWM_ETSEL_INTSEL_TBCTREQUZERO \n
 *                      - EHRPWM_ETSEL_INTSEL_TBCTREQUPRD \n
 *                      - EHRPWM_ETSEL_INTSEL_TBCTREQUCMPAINC \n
 *                      - EHRPWM_ETSEL_INTSEL_TBCTREQUCMPADEC \n
 *                      - EHRPWM_ETSEL_INTSEL_TBCTREQUCMPBINC \n
 *                      - EHRPWM_ETSEL_INTSEL_TBCTREQUCMPBDEC \n
 *
 * \return  None
 *
 **/
void EHRPWMETIntSourceSelect(unsigned int baseAddr, unsigned int selectInt)
{
	REG16(baseAddr + EHRPWM_ETSEL) =
		(REG16(baseAddr + EHRPWM_ETSEL) & (~EHRPWM_ETSEL_INTSEL)) |
		((selectInt << EHRPWM_ETSEL_INTSEL_SHIFT) & EHRPWM_ETSEL_INTSEL);
}

/**
 * \brief   This API disables the HR sub-module.
 *
 * \param   baseAddr    Base Address of the PWM Module Registers.
 *
 * \return  None
 *
 **/
void EHRPWMHRDisable(unsigned int baseAddr)
{
	REG16(baseAddr + EHRPWM_HRCNFG) &= (~EHRPWM_HR_EDGEMODE);
}

/**
 * \brief   This API enables the particular PWM module.
 *
 * \param   baseAddr    Base Address of the PWM Module Registers.
 *
 * \return  None
 *
 **/
void ehrPWM_Enable(unsigned int baseAddr)
{
	REG16(baseAddr + EHRPWM_AQCTLA) = 0x2 | (0x3 << 4);
	REG16(baseAddr + EHRPWM_AQCTLB) = 0x2 | (0x3 << 8);
	REG16(baseAddr + EHRPWM_TBCNT) = 0;
	REG16(baseAddr + EHRPWM_TBCTL) = ~0x3;
}

/**
 * \brief   This API disables the HR sub-module.
 *
 * \param   baseAddr    Base Address of the PWM Module Registers.
 *
 * \return  None
 *
 **/

void ehrPWM_Disable(unsigned int baseAddr)
{

	REG16(baseAddr + EHRPWM_TBCTL) = 0x3;
	REG16(baseAddr + EHRPWM_AQCTLA) = 0x1 | ( 0x3 << 4 );
	REG16(baseAddr + EHRPWM_AQCTLB) = 0x1 | ( 0x3 << 8 );
	REG16(baseAddr + EHRPWM_TBCNT)  = 0;
}

void configure_tbclk(unsigned int baseAddr, float freq)
{
	float Cyclens = 0.0f;
	float Divisor = 0;
	unsigned int counter,hspclkdiv=0,clkdiv,x;

	Cyclens = 1000000000.0f / freq; /** 10^9 /Hz compute time per cycle (ns)
				       */
	Divisor = (Cyclens / 655350.0f);  /** am335x provide (128* 14) divider,
					   *  and per TBPRD means 10ns when divider 
					   *  and max TBPRD is 65535 so max cycle 
					   *  is 128 * 8 * 14 * 65535 * 10ns
					   */
	if(Divisor > (128 * 14)) {
		printf("Can't generate %f HZ",freq);
	}
	else {
		if(Divisor > (0xE)) {
			hspclkdiv = TBCTL_HSPCLKDIV_DIVBY14;
			clkdiv = (int)Divisor/14;
		while(clkdiv > 1) {
		clkdiv = clkdiv >> 1;
		counter++;
		}
		} 
		else {
		if(Divisor <=1)
			hspclkdiv = TBCTL_HSPCLKDIV_DIVBY1;
		else if ((Divisor <=2) & (Divisor >1))
			hspclkdiv = TBCTL_HSPCLKDIV_DIVBY2;
		else if ((Divisor <=4) & (Divisor >2))
			hspclkdiv = TBCTL_HSPCLKDIV_DIVBY4;
		else if ((Divisor <=6) & (Divisor >4))
			hspclkdiv = TBCTL_HSPCLKDIV_DIVBY6;
		else if ((Divisor <=8) & (Divisor >6))
			hspclkdiv = TBCTL_HSPCLKDIV_DIVBY8;
		else if ((Divisor <=10) & (Divisor >8))
			hspclkdiv = TBCTL_HSPCLKDIV_DIVBY10;
		else if ((Divisor <=12) & (Divisor >10))
			hspclkdiv = TBCTL_HSPCLKDIV_DIVBY12;
		else if ((Divisor <=14) & (Divisor >12))
			hspclkdiv = TBCTL_HSPCLKDIV_DIVBY14;
		else 	
		{
		}
		counter = TBCTL_CLKDIV_DIVBY1;
		}
		REG16(baseAddr + EHRPWM_TBCTL) = (REG16(baseAddr + EHRPWM_TBCTL) &
	      	(~TBCTL_CLKDIV)) | ((counter << TBCTL_CLKDIV_SHIFT) & TBCTL_CLKDIV);                

        	REG16(baseAddr + EHRPWM_TBCTL) = (REG16(baseAddr + EHRPWM_TBCTL) &
            	(~TBCTL_HSPCLKDIV)) | ((hspclkdiv << 
		TBCTL_HSPCLKDIV_SHIFT) & TBCTL_HSPCLKDIV);

		printf("writing CLKDIV = %d and HSPCLKDIV = %d\n",counter,hspclkdiv);
		
		x = REG16(baseAddr + EHRPWM_TBCTL);
		printk("reading TBCTL = %x \n",x);

	}
}

/* PWMSS setting
 *      set pulse argument of epwm module
 *
 *      @param PWMID    : EPWMSS number , 0~2
 *      @param HZ       : pulse HZ
 *      @param dutyA    : Duty Cycle in ePWM A
 *      @param dutyB    : Duty Cycle in ePWM B
 *
 *      @return         : 1 for success , 0 for failed
 *
 *      @example        :  PWMSS_Setting(0 , 50.0f , 50.0f , 25.0f);      // Generate 50HZ pwm in PWM0 ,
 *                                                                              // duty cycle is 50% for ePWM0A , 25% for ePWM0B
 *
 *      @Note :
 *              find an number nearst 65535 for TBPRD , to improve duty precision,
 *
 *              Using big TBPRD can increase the range of CMPA and CMPB ,
 *              and it means we can get better precision on duty cycle.
 *
 *              EX : 20.25% duty cycle
 *                  on TBPRD = 62500 , CMPA = 12656.25 ( .25 rejection) , real duty : 20.2496% (12656 /62500)
 *                  on TBPRD = 6250  , CMPA = 1265.625 ( .625 rejection), real duty : 20.24%   (1265 6250)
 *                  on TBPRD = 500   , CMPA = 101.25   ( .25 rejection) , real duty : 20.2%    (101/500)
 *
 *              Divisor = CLKDIV * HSPCLKDIV
 *                      1 TBPRD : 10 ns (default)
 *                      65535 TBPRD : 655350 ns
 *                      65535 TBPRD : 655350 * Divisor ns  = X TBPRD : Cyclens
 *
 *              accrooding to that , we must find a Divisor value , let X nearest 65535 .
 *              so , Divisor must  Nearest Cyclens/655350
 */

int PWMSS_Setting(unsigned int baseAddr, float HZ, float dutyA, float dutyB)
{	
	unsigned int  y,z,p;
	int param_error =1;
	if(HZ < 0)
		param_error =0;
	if(dutyA < 0.0f || dutyA > 100.0f || dutyB < 0.0f || dutyB > 100.0f)
		param_error = 0;
	if(param_error == 0) {
		printf("ERROR in parameter \n");
	}
	dutyA /= 100.0f;
	dutyB /= 100.0f;

	/*Compute necessary TBPRD*/
	float Cyclens = 0.0f;
	float Divisor =0;
	int i,j;
	const float CLKDIV_div[] = {1.0,2.0,4.0,8.0,16.0,32.0,64.0,128.0};
	const float HSPCLKDIV_div[] = {1.0, 2.0, 4.0, 6.0, 8.0, 10.0,12.0, 14.0};
	int NearCLKDIV =7;
	int NearHSPCLKDIV =7;
	int NearTBPRD =0;

	Cyclens = 1000000000.0f / HZ; /** 10^9 /Hz compute time per cycle (ns)
				       */
	Divisor = (Cyclens / 655350.0f);  /** am335x provide (128* 14) divider,
					   *  and per TBPRD means 10ns when divider 
					   *  and max TBPRD is 65535 so max cycle 
					   *  is 128 * 8 * 14 * 65535 * 10ns
					   */
	if(Divisor > (128 * 14)) {
		printf("Can't generate %f HZ",HZ);
		return 0;
	}
	else {
		for (i=0;i<8;i++) {
			for(j=0 ; j<8; j++) {
				if((CLKDIV_div[i] * HSPCLKDIV_div[j]) < (CLKDIV_div[NearCLKDIV] 
							* HSPCLKDIV_div[NearHSPCLKDIV]) && (CLKDIV_div[i] * HSPCLKDIV_div[j] > Divisor)) {
					NearCLKDIV = i;
					NearHSPCLKDIV = j;
				}
			}
		}
		printf("BBBIO CLKDIV = %d and HSPCLKDIV = %d\n",NearCLKDIV,NearHSPCLKDIV);
		NearTBPRD = (Cyclens / (10.0 * CLKDIV_div[NearCLKDIV] * HSPCLKDIV_div[NearHSPCLKDIV]));
		printf("writing TBPRD = %x \n",NearTBPRD);
		/*setting clock divider and freeze time base*/
		configure_tbclk(baseAddr, HZ);
		REG16(baseAddr + EHRPWM_CMPB) = (unsigned short)((float)(NearTBPRD) * dutyB);
		z = REG16(baseAddr + EHRPWM_CMPB);
		printk("read CMPB = %x\t",z);
		REG16(baseAddr + EHRPWM_CMPA) = (unsigned short)((float)(NearTBPRD) * dutyA);
		p = REG16(baseAddr + EHRPWM_CMPA);
		printk("read CMPA = %x\n",p);
		REG16(baseAddr + EHRPWM_TBPRD) = (unsigned short)NearTBPRD;
		y = REG16(baseAddr + EHRPWM_TBPRD);
		printk("TBPRD read = %x \n",y);
		REG16(baseAddr + EHRPWM_TBCNT) = 0;
		printf("\nfinished setting \n"); 
	}
	return 1;
}
/*
void extra(unsigned int baseAddr)
{
	REG16(baseAddr + EHRPWM_TBCTL) = 0xC031;
	REG16(baseAddr + EHRPWM_TBPHS) = 0;
	REG16(baseAddr + EHRPWM_TBCNT) = 0;
	REG16(baseAddr + EHRPWM_TBPRD) = 0x258;
	REG16(baseAddr + EHRPWM_CMPA) = 0xc8;
	REG16(baseAddr + EHRPWM_CMPB) = 0x190;
	REG16(baseAddr + EHRPWM_CMPCTL) = 0;
	REG16(baseAddr + EHRPWM_AQCTLA) = 0x0120;
	REG16(baseAddr + EHRPWM_AQCTLB) = 0x0003;
}
*/
int PWMSS_TB_clock_check(unsigned int PWMSS_ID)
{
	unsigned int reg_value,value;
	
	/*control module check*/
	reg_value = REG(BBBIO_CONTROL_MODULE + BBBIO_PWMSS_CTRL);
	
	value = reg_value & (1 << PWMSS_ID);
	printf("\n PWMSS_CTRL =  %d and reg_value = %d \n",value,reg_value);
	return (reg_value & (1 << PWMSS_ID));
}

int PWMSS_module_ctrl (unsigned int PWMSS_ID,int enable)
{ 
	volatile unsigned int *reg = NULL;
	unsigned int module_set[] = {0, 1, 2};
	int ret =1;
 	unsigned int module_clk_set[] = {BBBIO_CM_PER_EPWMSS0_CLKCTRL, BBBIO_CM_PER_EPWMSS1_CLKCTRL, BBBIO_CM_PER_EPWMSS2_CLKCTRL};
		reg = (void*)BBBIO_CM_PER_ADDR + module_clk_set[PWMSS_ID];

	if(enable) 
 	{
	/*Module enable and fully functional */
	if(PWMSS_TB_clock_check(module_set[PWMSS_ID]))
	*reg = 0x2; 
	return ret;
	}
	else {
	printf("time base clock disable\n");
	ret =0;
	return 0;
}
return ret;
}



