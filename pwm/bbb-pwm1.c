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
  EHRPWMClockEnable(PWMSS2_MMAP_ADDR);
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
			REG(AM335X_PADCONF_BASE + BBBIO_PWMSS_CTRL) |=
				BBBIO_PWMSS_CTRL_PWMSS0_TBCLKEN;
			break;

		case 1:
			REG(AM335X_PADCONF_BASE + BBBIO_PWMSS_CTRL) |=
				BBBIO_PWMSS_CTRL_PWMSS1_TBCLKEN;
			break;

		case 2:
			REG(AM335X_PADCONF_BASE + BBBIO_PWMSS_CTRL) |=
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

	if(0 == instanceNum)
	{
		REG(BBBIO_CM_PER_ADDR + CM_PER_EPWMSS0_CLKCTRL) |=
			CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_ENABLE;

	}
	else if(1 == instanceNum)
	{
		REG(BBBIO_CM_PER_ADDR + CM_PER_EPWMSS1_CLKCTRL) |=
			CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_ENABLE;
	}
	else if(2 == instanceNum)
	{
		REG(BBBIO_CM_PER_ADDR + CM_PER_EPWMSS2_CLKCTRL) |=
			CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_ENABLE;

	}
	else
	{

	} 
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
	REG16(baseAddr + EPWM_AQCTLA) = 0x2 | (0x3 << 4);
	REG16(baseAddr + EPWM_AQCTLB) = 0x2 | (0x3 << 8);
	REG16(baseAddr + EPWM_TBCNT) = 0;
	REG16(baseAddr + EPWM_TBCTL) = ~0x3;
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

	REG16(baseAddr + EPWM_TBCTL) = 0x3;
	REG16(baseAddr + EPWM_AQCTLA) = 0x1 | ( 0x3 << 4 );
	REG16(baseAddr + EPWM_AQCTLB) = 0x1 | ( 0x3 << 8 );
	REG16(baseAddr + EPWM_TBCNT)  = 0;
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
		printf("CLKDIV = %d and HSPCLKDIV = %d\n",NearCLKDIV,NearHSPCLKDIV);
		NearTBPRD = (Cyclens / (10.0 * CLKDIV_div[NearCLKDIV] * HSPCLKDIV_div[NearHSPCLKDIV]));
		printf("TBPRD = %d \n",NearTBPRD);
		/*setting clock divider and freeze time base*/
		REG16(baseAddr + EPWM_TBCTL) = (REG16(baseAddr + EPWM_TBCTL) & 
				(~EPWM_TBCTL_CLKDIV)) | ((NearCLKDIV << 10) & EPWM_TBCTL_CLKDIV);
		REG16(baseAddr + EPWM_TBCTL) = (REG16(baseAddr + EPWM_TBCTL) & 
				(~EPWM_TBCTL_HSPCLKDIV)) | ((NearHSPCLKDIV << 
				7) & EPWM_TBCTL_HSPCLKDIV);
	//	REG16(baseAddr + EPWM_TBCTL) = TBCTL_CTRMODE_FREEZE | (NearCLKDIV << 10) | (NearHSPCLKDIV << 7);
		REG16(baseAddr + EPWM_CMPB) = (unsigned short)((float)(NearTBPRD) * dutyB);
		REG16(baseAddr + EPWM_CMPA) = (unsigned short)((float)(NearTBPRD) * dutyA);
		REG16(baseAddr + EPWM_TBPRD) = (unsigned short)NearTBPRD;
		REG16(baseAddr + EPWM_TBCNT) = 0;
		printf("\nfinished setting \n"); 
	}
	return 1;
}

int PWMSS_TB_clock_check(unsigned int PWMSS_ID)
{
	unsigned int reg_value,value;
	
	/*control module check*/
	reg_value = REG(BBBIO_CONTROL_MODULE + BBBIO_PWMSS_CTRL);
	
	value = reg_value & (1 << PWMSS_ID);
	printf("\n PWMSS_CTRL =  %d and reg_value = %d \n",value,reg_value);
	return (reg_value & (1 << PWMSS_ID));
}

