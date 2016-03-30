#include<libcpu/am335x.h>
#include<stdio.h>
#include<bsp/gpio.h>
#include<bsp/bbb-gpio.h>

//*****************************************************************************
//
// Macros for hardware access, both direct and via the bit-band region.
//
//*****************************************************************************
#define HWREG(x)                                                              \
        (*((volatile unsigned int *)(x)))
#define HWREGH(x)                                                             \
        (*((volatile unsigned short *)(x)))
#define HWREGB(x)                                                             \
        (*((volatile unsigned char *)(x)))
#define HWREGBITW(x, b)                                                       \
        HWREG(((unsigned int)(x) & 0xF0000000) | 0x02000000 |                \
              (((unsigned int)(x) & 0x000FFFFF) << 5) | ((b) << 2))
#define HWREGBITH(x, b)                                                       \
        HWREGH(((unsigned int)(x) & 0xF0000000) | 0x02000000 |               \
               (((unsigned int)(x) & 0x000FFFFF) << 5) | ((b) << 2))
#define HWREGBITB(x, b)                                                       \
        HWREGB(((unsigned int)(x) & 0xF0000000) | 0x02000000 |               \
               (((unsigned int)(x) & 0x000FFFFF) << 5) | ((b) << 2))


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
               HWREG(SOC_CONTROL_REGS + CONTROL_PWMSS_CTRL) |=
                                             CONTROL_PWMSS_CTRL_PWMSS0_TBCLKEN;
               break;

         case 1:
               HWREG(SOC_CONTROL_REGS + CONTROL_PWMSS_CTRL) |=
                                             CONTROL_PWMSS_CTRL_PWMMS1_TBCLKEN;
               break;

         case 2:
               HWREG(SOC_CONTROL_REGS + CONTROL_PWMSS_CTRL) |=
                                             CONTROL_PWMSS_CTRL_PWMSS2_TBCLKEN;
               break;

         default:
         break;
    }
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
    HWREG(SOC_PRCM_REGS + CM_PER_L3S_CLKSTCTRL) |=
                             CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    while((HWREG(SOC_PRCM_REGS + CM_PER_L3S_CLKSTCTRL) &
     CM_PER_L3S_CLKSTCTRL_CLKTRCTRL) != CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

    HWREG(SOC_PRCM_REGS + CM_PER_L3_CLKSTCTRL) |=
                             CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    while((HWREG(SOC_PRCM_REGS + CM_PER_L3_CLKSTCTRL) &
     CM_PER_L3_CLKSTCTRL_CLKTRCTRL) != CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

    HWREG(SOC_PRCM_REGS + CM_PER_L3_INSTR_CLKCTRL) |=
                             CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;
while((HWREG(SOC_PRCM_REGS + CM_PER_L3_INSTR_CLKCTRL) &
                               CM_PER_L3_INSTR_CLKCTRL_MODULEMODE) !=
                                   CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_PRCM_REGS + CM_PER_L3_CLKCTRL) |=
                             CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_PRCM_REGS + CM_PER_L3_CLKCTRL) &
        CM_PER_L3_CLKCTRL_MODULEMODE) != CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE);

    HWREG(SOC_PRCM_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) |=
                             CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    while((HWREG(SOC_PRCM_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
                              CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL) !=
                                CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

    HWREG(SOC_PRCM_REGS + CM_PER_L4LS_CLKSTCTRL) |=
                             CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
while((HWREG(SOC_PRCM_REGS + CM_PER_L4LS_CLKSTCTRL) &
                             CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL) !=
                               CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

    HWREG(SOC_PRCM_REGS + CM_PER_L4LS_CLKCTRL) |=
                             CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE;

    while((HWREG(SOC_PRCM_REGS + CM_PER_L4LS_CLKCTRL) &
      CM_PER_L4LS_CLKCTRL_MODULEMODE) != CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE);

    if(0 == instanceNum)
    {
        HWREG(SOC_PRCM_REGS + CM_PER_EPWMSS0_CLKCTRL) |=
            CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_ENABLE;

        while(CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_ENABLE !=
              (HWREG(SOC_PRCM_REGS + CM_PER_EPWMSS0_CLKCTRL) &
               CM_PER_EPWMSS0_CLKCTRL_MODULEMODE));
while((CM_PER_EPWMSS0_CLKCTRL_IDLEST_FUNC <<
               CM_PER_EPWMSS0_CLKCTRL_IDLEST_SHIFT) !=
              (HWREG(SOC_PRCM_REGS + CM_PER_EPWMSS0_CLKCTRL) &
               CM_PER_EPWMSS0_CLKCTRL_IDLEST));

    }
    else if(1 == instanceNum)
    {
        HWREG(SOC_PRCM_REGS + CM_PER_EPWMSS1_CLKCTRL) |=
            CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_ENABLE;

        while(CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_ENABLE !=
              (HWREG(SOC_PRCM_REGS + CM_PER_EPWMSS1_CLKCTRL) &
               CM_PER_EPWMSS1_CLKCTRL_MODULEMODE));

        while((CM_PER_EPWMSS1_CLKCTRL_IDLEST_FUNC <<
               CM_PER_EPWMSS1_CLKCTRL_IDLEST_SHIFT) !=
               (HWREG(SOC_PRCM_REGS + CM_PER_EPWMSS1_CLKCTRL) &
               CM_PER_EPWMSS1_CLKCTRL_IDLEST));

    }
 else if(2 == instanceNum)
    {
        HWREG(SOC_PRCM_REGS + CM_PER_EPWMSS2_CLKCTRL) |=
            CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_ENABLE;

        while(CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_ENABLE !=
              (HWREG(SOC_PRCM_REGS + CM_PER_EPWMSS2_CLKCTRL) &
               CM_PER_EPWMSS2_CLKCTRL_MODULEMODE));

        while((CM_PER_EPWMSS2_CLKCTRL_IDLEST_FUNC <<
               CM_PER_EPWMSS2_CLKCTRL_IDLEST_SHIFT) !=
               (HWREG(SOC_PRCM_REGS + CM_PER_EPWMSS2_CLKCTRL) &
                CM_PER_EPWMSS2_CLKCTRL_IDLEST));
    }
    else
    {

    }
while(!(HWREG(SOC_PRCM_REGS + CM_PER_L3S_CLKSTCTRL) &
            CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK));

    while(!(HWREG(SOC_PRCM_REGS + CM_PER_L3_CLKSTCTRL) &
            CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));

    while(!(HWREG(SOC_PRCM_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
           (CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK |
            CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L4_GCLK)));

    while(!(HWREG(SOC_PRCM_REGS + CM_PER_L4LS_CLKSTCTRL) &
           (CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK )));

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
    }
    else
    {
        hspClkDiv = clkDiv/2; /* reg setting */
        /* divide by 1 */
        lspClkDivSetting = EHRPWM_TBCTL_HSPCLKDIV_DIVBY1;
    }
 HWREGH(baseAddr + EHRPWM_TBCTL) = (HWREGH(baseAddr + EHRPWM_TBCTL) &
            (~EHRPWM_TBCTL_CLKDIV)) | ((lspClkDivSetting <<
            EHRPWM_TBCTL_CLKDIV_SHIFT) & EHRPWM_TBCTL_CLKDIV);

    HWREGH(baseAddr + EHRPWM_TBCTL) = (HWREGH(baseAddr + EHRPWM_TBCTL) &
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

     HWREGH(baseAddr + EHRPWM_TBCTL) = (HWREGH(baseAddr + EHRPWM_TBCTL) &
             (~EHRPWM_PRD_LOAD_SHADOW_MASK)) | ((enableShadowWrite <<
            EHRPWM_TBCTL_PRDLD_SHIFT) & EHRPWM_PRD_LOAD_SHADOW_MASK);

     HWREGH(baseAddr + EHRPWM_TBCTL) = (HWREGH(baseAddr + EHRPWM_TBCTL) &
             (~EHRPWM_COUNTER_MODE_MASK)) | ((counterDir <<
            EHRPWM_TBCTL_CTRMODE_SHIFT) &  EHRPWM_COUNTER_MODE_MASK);

     if(EHRPWM_COUNT_UP_DOWN == counterDir)
     {
         HWREGH(baseAddr + EHRPWM_TBPRD) = (unsigned short)tbPeriodCount/2;
     }
     else
     {
         HWREGH(baseAddr + EHRPWM_TBPRD) = (unsigned short)tbPeriodCount;
     }

}

void EHRPWMTimebaseSyncDisable(unsigned int baseAddr)
{
     HWREGH(baseAddr + EHRPWM_TBCTL) &= (~EHRPWM_SYNC_ENABLE);
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
     HWREGH(baseAddr + EHRPWM_TBCTL) = (HWREGH(baseAddr + EHRPWM_TBCTL) &
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
     HWREGH(baseAddr + EHRPWM_TBCTL) = (HWREGH(baseAddr + EHRPWM_TBCTL) &
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
        ((HWREGH(baseAddr+EHRPWM_CMPCTL) & EHRPWM_CMPCTL_SHDWAFULL) ==
                             EHRPWM_SHADOW_A_EMPTY))
    {
        HWREGH(baseAddr + EHRPWM_CMPCTL) = (HWREGH(baseAddr + EHRPWM_CMPCTL) &
            (~EHRPWM_CMPCTL_SHDWAMODE)) | ((enableShadowWrite <<
            EHRPWM_CMPCTL_SHDWAMODE_SHIFT) & EHRPWM_CMPCTL_SHDWAMODE);

        HWREGH(baseAddr + EHRPWM_CMPCTL) = (HWREGH(baseAddr + EHRPWM_CMPCTL) &
            (~EHRPWM_COMPA_LOAD_MASK)) |((ShadowToActiveLoadTrigger <<
            EHRPWM_CMPCTL_LOADAMODE_SHIFT) & EHRPWM_COMPA_LOAD_MASK);

        HWREGH(baseAddr + EHRPWM_CMPA) = CMPAVal & EHRPWM_CMPA_CMPA;

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
        ((HWREGH(baseAddr+EHRPWM_CMPCTL) & EHRPWM_CMPCTL_SHDWBFULL) ==
                             EHRPWM_SHADOW_B_EMPTY))
    {
        HWREGH(baseAddr + EHRPWM_CMPCTL) = (HWREGH(baseAddr + EHRPWM_CMPCTL)
            & (~EHRPWM_CMPCTL_SHDWBMODE)) | ((enableShadowWrite <<
            EHRPWM_CMPCTL_SHDWBMODE_SHIFT) & EHRPWM_CMPCTL_SHDWBMODE);

        HWREGH(baseAddr + EHRPWM_CMPCTL) = (HWREGH(baseAddr + EHRPWM_CMPCTL) &
            (~EHRPWM_COMPB_LOAD_MASK)) | ((ShadowToActiveLoadTrigger <<
            EHRPWM_CMPCTL_LOADBMODE_SHIFT) & EHRPWM_COMPB_LOAD_MASK);

        HWREGH(baseAddr + EHRPWM_CMPB) = CMPBVal & EHRPWM_CMPB_CMPB;

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
    HWREGH(baseAddr + EHRPWM_AQCTLB) =
        ((CBDown << EHRPWM_AQCTLB_CBD_SHIFT) & EHRPWM_AQCTLB_CBD) |
        ((CBUp << EHRPWM_AQCTLB_CBU_SHIFT) & EHRPWM_AQCTLB_CBU) |
        ((CADown << EHRPWM_AQCTLB_CAD_SHIFT) & EHRPWM_AQCTLB_CAD) |
        ((CAUp << EHRPWM_AQCTLB_CAU_SHIFT) & EHRPWM_AQCTLB_CAU) |
        ((period << EHRPWM_AQCTLB_PRD_SHIFT) & EHRPWM_AQCTLB_PRD) |
        ((zero << EHRPWM_AQCTLB_ZRO_SHIFT) & EHRPWM_AQCTLB_ZRO);


    HWREGH(baseAddr + EHRPWM_AQSFRC) =
        (HWREGH(baseAddr + EHRPWM_AQSFRC) & (~EHRPWM_AQSFRC_ACTSFB)) |
     ((SWForced << EHRPWM_AQSFRC_ACTSFB_SHIFT) & EHRPWM_AQSFRC_ACTSFB);
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
    HWREGH(baseAddr + EHRPWM_DBCTL) =
        (HWREGH(baseAddr + EHRPWM_DBCTL) & (~EHRPWM_DBCTL_OUT_MODE)) |
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
    HWREGH(baseAddr + EHRPWM_PCCTL) &= (~EHRPWM_PCCTL_CHPEN);
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
         HWREGH(baseAddr + EHRPWM_TZSEL) &= (~EHRPWM_TZSEL_OSHT1);
     }
     if(EHRPWM_TZ_CYCLEBYCYCLE == osht_CBC)
     {
         HWREGH(baseAddr + EHRPWM_TZSEL) &= (~EHRPWM_TZSEL_CBC1);
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
    HWREGH(baseAddr + EHRPWM_ETPS) =
        (HWREGH(baseAddr + EHRPWM_ETPS) & (~EHRPWM_ETPS_INTPRD)) |
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
    HWREGH(baseAddr + EHRPWM_ETSEL) =
        (HWREGH(baseAddr + EHRPWM_ETSEL) & (~EHRPWM_ETSEL_INTSEL)) |
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
     HWREGH(baseAddr + EHRPWM_HRCNFG) &= (~EHRPWM_HR_EDGEMODE);
}

