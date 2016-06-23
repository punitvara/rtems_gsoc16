#include<libcpu/am335x.h>
#include<stdio.h>
#include<bsp/gpio.h>
#include<bsp/bbb-gpio.h>
#include<bsp.h>


/**
 * @brief This function intilize clock and pinmuxing for pwm sub system.
 *
 * @param PWMSS_ID It is the instance number of EPWM of pwm sub system.
 * 
 * @return None
 **/
void pwm_init(uint32_t pin_no, uint32_t pwmss_id)
{
  module_clk_config(pwmss_id);
  epwm_pinmux_setup(pin_no);
  EPWM_clock_enable(pwmss_id);
  pwmss_tbclk_enable(pwmss_id);
}

uint32_t select_pwmss(uint32_t pwm_id)
{
uint32_t baseAddr=0;
   if (pwm_id == BBBIO_PWMSS0)
   {
	baseAddr = PWMSS0_MMAP_ADDR;
	return baseAddr;
   }
   else if (pwm_id == BBBIO_PWMSS1)
   {
	baseAddr = PWMSS1_MMAP_ADDR;
	return baseAddr;
   } 
   else if (pwm_id == BBBIO_PWMSS2)
   {
	baseAddr = PWMSS2_MMAP_ADDR;
	return baseAddr;
   }
   else 
   {
	printf("Invalid PWM Id\n");
	return 0;	
   }
}

/**
 * @brief   This function Enables pinmuxing for PWM module.
 *          
 *
 * @param   instance  It is the instance number of EPWM of pwmsubsystem.
 *
 *
 * @return None
 **/

void epwm_pinmux_setup(int pin_no)
{
  switch(pin_no)  {
	case P8_13_2B:
  		REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(9)) = BBB_MUXMODE(MUXMODE4);
		break;
	case P8_19_2A:
  		REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(8)) = BBB_MUXMODE(MUXMODE4);
		break;
	case P8_45_2A:
  		REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(0)) = BBB_MUXMODE(MUXMODE3);
		break;
	case P8_46_2B
  		REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(1)) = BBB_MUXMODE(MUXMODE3);
		break;
	case P8_34_1B:
  		REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(11)) = BBB_MUXMODE(MUXMODE2);
		break;
	case P8_36_1A:
  		REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(10)) = BBB_MUXMODE(MUXMODE2);
		break;
	case P9_14_1A:
  		REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(2)) = BBB_MUXMODE(MUXMODE6);
		break;
	case P9_16_1B:
  		REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(3)) = BBB_MUXMODE(MUXMODE6);
		break;
	case P9_21_0B:
  		REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_D0) = BBB_MUXMODE(MUXMODE3);
		break;
	case P9_22_0A:
  		REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_SCLK) = BBB_MUXMODE(MUXMODE3);
		break;
	case P9_29_0B:
  		REG(AM335X_PADCONF_BASE + AM335X_CONF_MCASP0_FSX) = BBB_MUXMODE(MUXMODE1);
		break;
	case P9_31_0A:
  		REG(AM335X_PADCONF_BASE + AM335X_CONF_MCASP0_ACLKX) = BBB_MUXMODE(MUXMODE1);
		break;
	case EPWM_GROUP2:
		REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(9)) = BBB_MUXMODE(MUXMODE4);
		REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(8)) = BBB_MUXMODE(MUXMODE4);
		REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(0)) = BBB_MUXMODE(MUXMODE3);
		REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(1)) = BBB_MUXMODE(MUXMODE3);
		break;
	case EPWM_GROUP1:
		REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(11)) = BBB_MUXMODE(MUXMODE2);
		REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(10)) = BBB_MUXMODE(MUXMODE2);
		REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(2)) = BBB_MUXMODE(MUXMODE6);
		REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(3)) = BBB_MUXMODE(MUXMODE6);
		break;
	case EPWM_GROUP0:
		REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_D0) = BBB_MUXMODE(MUXMODE3);
		REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_SCLK) = BBB_MUXMODE(MUXMODE3);
		REG(AM335X_PADCONF_BASE + AM335X_CONF_MCASP0_FSX) = BBB_MUXMODE(MUXMODE1);
		REG(AM335X_PADCONF_BASE + AM335X_CONF_MCASP0_ACLKX) = BBB_MUXMODE(MUXMODE1);
		break;
	
	default:
		printf("PWM output is not available on this pin\n");
		break;
}

/* PWMSS setting
 *      set pulse argument of epwm module
 *
 *      @param PWMID    : EPWMSS number , 0~2
 *      @param pwm_freq : frequency to be generated
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

int PWMSS_Setting(uint32_t pwm_id, float pwm_freq, float dutyA, float dutyB)
{	
  uint32_t baseAddr;
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

  /** 10^9 /Hz compute time per cycle (ns) */
  Cyclens = 1000000000.0f / pwm_freq;

  /** am335x provide (128* 14) divider and per TBPRD means 10ns when divider 
    * and max TBPRD is 65535 so max cycle is 128 * 8 * 14 * 65535 * 10ns */
  Divisor = (Cyclens / 655350.0f);
	
  if(Divisor > (128 * 14)) {
	printf("Can't generate %f HZ",pwm_freq);
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
  baseAddr = select_pwmss(pwm_id);	
  REG16(baseAddr + EPWM_TBCTL) &= ~(TBCTL_CLKDIV_MASK | TBCTL_HSPCLKDIV_MASK);
			
  REG16(baseAddr + EPWM_TBCTL) = (REG16(baseAddr + EPWM_TBCTL) &
  (~EPWM_TBCTL_CLKDIV)) | ((NearCLKDIV 
  << EPWM_TBCTL_CLKDIV_SHIFT) & EPWM_TBCTL_CLKDIV);

  REG16(baseAddr + EPWM_TBCTL) = (REG16(baseAddr + EPWM_TBCTL) &
  (~EPWM_TBCTL_HSPCLKDIV)) | ((NearHSPCLKDIV << 
  EPWM_TBCTL_HSPCLKDIV_SHIFT) & EPWM_TBCTL_HSPCLKDIV);

  NearTBPRD = (Cyclens / (10.0 * CLKDIV_div[NearCLKDIV] * HSPCLKDIV_div[NearHSPCLKDIV]));
		
  REG16(baseAddr + EPWM_TBCTL) = (REG16(baseAddr + EPWM_TBCTL) &
  (~EPWM_PRD_LOAD_SHADOW_MASK)) | (((bool)EPWM_SHADOW_WRITE_DISABLE <<
  EPWM_TBCTL_PRDLD_SHIFT) & EPWM_PRD_LOAD_SHADOW_MASK);

  REG16(baseAddr + EPWM_TBCTL) = (REG16(baseAddr + EPWM_TBCTL) &
  (~EPWM_COUNTER_MODE_MASK)) | (((unsigned int)EPWM_COUNT_UP <<
  EPWM_TBCTL_CTRMODE_SHIFT) &  EPWM_COUNTER_MODE_MASK);

  /*setting clock divider and freeze time base*/
  REG16(baseAddr + EPWM_CMPB) = (unsigned short)((float)(NearTBPRD) * dutyB);
  REG16(baseAddr + EPWM_CMPA) = (unsigned short)((float)(NearTBPRD) * dutyA);
  REG16(baseAddr + EPWM_TBPRD) = (unsigned short)NearTBPRD;
  REG16(baseAddr + EPWM_TBCNT) = 0;
  }
  return 1;
}


/**
 * @brief   This API enables the particular PWM module.
 *
 * @param   baseAddr    Base Address of the PWM Module Registers.
 *
 * @return  None
 *
 **/
void ehrpwm_enable(uint32_t pwmid)
{
	uint32_t baseAddr;
	baseAddr = select_pwmss(pwmid);
        REG16(baseAddr + EPWM_AQCTLA) = EPWM_AQCTLA_ZRO_EPWMXAHIGH | (EPWM_AQCTLA_CAU_EPWMXATOGGLE << EPWM_AQCTLA_CAU_SHIFT);
        REG16(baseAddr + EPWM_AQCTLB) = EPWM_AQCTLB_ZRO_EPWMXBHIGH | (EPWM_AQCTLB_CBU_EPWMXBTOGGLE << EPWM_AQCTLB_CBU_SHIFT);
        REG16(baseAddr + EPWM_TBCNT) = 0;
        REG16(baseAddr + EPWM_TBCTL) |=  TBCTL_FREERUN  | TBCTL_CTRMODE_UP;
}

/**
 * @brief   This API disables the HR sub-module.
 *
 * @param   baseAddr    Base Address of the PWM Module Registers.
 *
 * @return  None
 *
 **/

void ehrpwm_disable(uint32_t pwmid)
{
	uint32_t baseAddr;
	baseAddr = select_pwmss(pwmid);
        REG16(baseAddr + EPWM_TBCTL) = EPWM_TBCTL_CTRMODE_STOPFREEZE;
        REG16(baseAddr + EPWM_AQCTLA) = EPWM_AQCTLA_ZRO_EPWMXALOW | (EPWM_AQCTLA_CAU_EPWMXATOGGLE << EPWM_AQCTLA_CAU_SHIFT);
        REG16(baseAddr + EPWM_AQCTLB) = EPWM_AQCTLA_ZRO_EPWMXBLOW | (EPWM_AQCTLB_CBU_EPWMXBTOGGLE << EPWM_AQCTLB_CBU_SHIFT);
        REG16(baseAddr + EPWM_TBCNT)  = 0;
}

/**
 * @brief   This function Enables TBCLK(Time Base Clock) for specific
 *          EPWM instance of pwmsubsystem.
 *
 * @param   instance  It is the instance number of EPWM of pwmsubsystem.
 *
 **/
bool pwmss_tbclk_enable(unsigned int instance)
{

uint32_t enable_bit;
bool is_valid = true;
  
  if (is_valid)
  {
    	REG(AM335X_PADCONF_BASE + CONTROL_PWMSS_CTRL) |= enable_bit;
  }
  
  if (instance == BBBIO_PWMSS0)
  {
    	enable_bit = BBBIO_PWMSS_CTRL_PWMSS0_TBCLKEN;
  }
  else if (instance == BBBIO_PWMSS1)
  {
     	enable_bit = BBBIO_PWMSS_CTRL_PWMSS1_TBCLKEN;
  }
  else if (instance == BBBIO_PWMSS2)
  {
     	enable_bit = BBBIO_PWMSS_CTRL_PWMSS2_TBCLKEN;
  }
  else
  {
     	is_valid = false;
  }
  return is_valid;
 }

/**
 * @brief   This functions enables clock for EHRPWM module in PWMSS subsystem.
 *
 * @param   baseAdd   It is the Memory address of the PWMSS instance used.
 *
 * @return  None.
 *
 **/

void epwm_clock_enable(uint32_t pwm_id)
{	
	uint32_t baseAddr;
	baseAddr = select_pwmss(pwm_id);
        REG(baseAdd + PWMSS_CLKCONFIG) |= PWMSS_CLK_EN_ACK;
}

/**
 * @brief   This function configures the L3 and L4_PER system clocks.
 *          It also configures the system clocks for the specified ePWMSS
 *          instance.
 *
 * @param   pwmss_id    The instance number of ePWMSS whose system clocks
 *                         have to be configured.
 *
 * 'pwmss_id' can take one of the following values:
 * (0 <= pwmss_id <= 2)
 *
 * @return  None.
 *
 */

void module_clk_config(uint32_t pwmss_id)
{
        if(pwmss_id == 0)
        {
                REG(BBBIO_CM_PER_ADDR + BBBIO_CM_PER_EPWMSS0_CLKCTRL) |=
                        BBBIO_CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_ENABLE;


        }
        else if(pwmss_id == 1)
        {
                REG(BBBIO_CM_PER_ADDR + BBBIO_CM_PER_EPWMSS1_CLKCTRL) |=
                        BBBIO_CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_ENABLE;

        }
        else if(pwmss_id == 2)
        {
                REG(BBBIO_CM_PER_ADDR + BBBIO_CM_PER_EPWMSS2_CLKCTRL) |=
                        BBBIO_CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_ENABLE;

        }
        else
        {
		printf("Please enter valid pwm Id \n");
        }
}



