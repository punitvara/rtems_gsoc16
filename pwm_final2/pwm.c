/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief Support for PWM for the BeagleBone Black.
 */

/**
 * Copyright (c) 2016 Punit Vara <punitvara at gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

/** This file is based on 
  * https://github.com/VegetableAvenger/BBBIOlib/blob/master/BBBio_lib/BBBiolib_PWMSS.c
  */

#include <libcpu/am335x.h>
#include <stdio.h>
#include <bsp/gpio.h>
#include <bsp/bbb-gpio.h>
#include <bsp.h>
#include <bsp/bbb-pwm.h>
#include <bsp/beagleboneblack.h>

/* Currently these definitions are for BeagleBone Black board only
 * Later on Beagle-xM board support can be added in this code.
 * After support gets added if condition should be removed
 */
#if IS_AM335X

/*
 * @brief This function select PWM module to be enabled
 * 
 * @param pwm_id It is the instance number of EPWM of pwm sub system.
 * 
 * @return Base Address of respective pwm instant.
*/
static uint32_t select_pwm(BBB_PWMSS pwm_id)
{
  uint32_t baseAddr=0;
  if (pwm_id == BBB_PWMSS0)
    baseAddr = AM335X_EPWM_0_REGS;
  else if (pwm_id == BBB_PWMSS1)
    baseAddr = AM335X_EPWM_1_REGS;
  else if (pwm_id == BBB_PWMSS2)
    baseAddr = AM335X_EPWM_2_REGS;
  else 
    baseAddr = 0;
  return baseAddr;	
}

bool beagle_pwm_pinmux_setup(bbb_pwm_pin_t pin_no, BBB_PWMSS pwm_id)
{
bool is_valid = true;
  if(pwm_id == BBB_PWMSS0) {
    if (pin_no == BBB_P9_21_0B)
      REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_D0) = BBB_MUXMODE(BBB_P9_21_MUX_PWM);
    else if (pin_no == BBB_P9_22_0A)
      REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_SCLK) = BBB_MUXMODE(BBB_P9_22_MUX_PWM);
    else if (pin_no == BBB_P9_29_0B)
      REG(AM335X_PADCONF_BASE + AM335X_CONF_MCASP0_FSX) = BBB_MUXMODE(BBB_P9_29_MUX_PWM);
    else if (pin_no == BBB_P9_31_0A)
      REG(AM335X_PADCONF_BASE + AM335X_CONF_MCASP0_ACLKX) = BBB_MUXMODE(BBB_P9_31_MUX_PWM);
    else 
      is_valid = false;
    } else if (pwm_id == BBB_PWMSS1) {
      	if (pin_no == BBB_P8_34_1B)
      	  REG(AM335X_PADCONF_BASE + BBB_CONTROL_CONF_LCD_DATA(11)) = BBB_MUXMODE(BBB_P8_34_MUX_PWM);
	else if (pin_no == BBB_P8_36_1A)
	  REG(AM335X_PADCONF_BASE + BBB_CONTROL_CONF_LCD_DATA(10)) = BBB_MUXMODE(BBB_P8_36_MUX_PWM);
	else if (pin_no == BBB_P9_14_1A)
	  REG(AM335X_PADCONF_BASE + BBB_CONTROL_CONF_GPMC_AD(2)) = BBB_MUXMODE(BBB_P9_14_MUX_PWM);
	else if (pin_no == BBB_P9_16_1B)
	  REG(AM335X_PADCONF_BASE + BBB_CONTROL_CONF_GPMC_AD(3)) = BBB_MUXMODE(BBB_P9_14_MUX_PWM);
	else 
	  is_valid = false;
   } else if (pwm_id == BBB_PWMSS2) {
	if (pin_no == BBB_P8_13_2B)
	  REG(AM335X_PADCONF_BASE + BBB_CONTROL_CONF_GPMC_AD(9)) = BBB_MUXMODE(BBB_P8_13_MUX_PWM);
	else if (pin_no == BBB_P8_19_2A)
	  REG(AM335X_PADCONF_BASE + BBB_CONTROL_CONF_GPMC_AD(8)) = BBB_MUXMODE(BBB_P8_19_MUX_PWM);
	else if (pin_no == BBB_P8_45_2A)
	  REG(AM335X_PADCONF_BASE + BBB_CONTROL_CONF_LCD_DATA(0)) = BBB_MUXMODE(BBB_P8_45_MUX_PWM);
	else if (pin_no == BBB_P8_46_2B)
	  REG(AM335X_PADCONF_BASE + BBB_CONTROL_CONF_LCD_DATA(1)) = BBB_MUXMODE(BBB_P8_46_MUX_PWM);
	else
	  is_valid = false;
  } else 
	is_valid = false;
  return is_valid;
}

/**
 * @brief   This function Enables TBCLK(Time Base Clock) for specific
 *          EPWM instance of pwmsubsystem.
 *
 * @param   instance  It is the instance number of EPWM of pwmsubsystem.
 *
 * @return  true if successful
 **/
static bool pwmss_tbclk_enable(BBB_PWMSS instance)
{
uint32_t enable_bit;
bool is_valid = true;
  
  if (instance == BBB_PWMSS0)  {
    enable_bit = AM335X_PWMSS_CTRL_PWMSS0_TBCLKEN;
  }  else if (instance == BBB_PWMSS1)  {
       enable_bit = AM335X_PWMSS_CTRL_PWMSS1_TBCLKEN;
  }  else if (instance == BBB_PWMSS2)  {
       enable_bit = AM335X_PWMSS_CTRL_PWMSS2_TBCLKEN;
  }  else  {
       is_valid = false;
  }

  if (is_valid)
  {
    REG(AM335X_PADCONF_BASE + AM335X_PWMSS_CTRL) |= enable_bit;
  }

  return is_valid;
 }

/**
 * @brief   This functions enables clock for EHRPWM module in PWMSS subsystem.
 *
 * @param   pwm_id  It is the instance number of EPWM of pwm sub system.
 *
 * @return  None.
 *
 **/
static bool pwm_clock_enable(BBB_PWMSS pwm_id)
{
  const bool id_is_valid = pwm_id < BBB_PWMSS_COUNT;	
  bool status = true;
  if (id_is_valid) {
    const uint32_t baseAddr = select_pwm(pwm_id);
    REG(baseAddr - AM335X_EPWM_REGS + AM335X_PWMSS_CLKCONFIG) |= AM335X_PWMSS_CLK_EN_ACK;
  }  else  {
       status = false;
  }
  return status;
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
static bool module_clk_config(BBB_PWMSS pwmss_id)
{
  bool is_valid = true;
  if(pwmss_id == BBB_PWMSS0) {
    const uint32_t is_functional = AM335X_CM_PER_EPWMSS0_CLKCTRL_IDLEST_FUNC <<
                         AM335X_CM_PER_EPWMSS0_CLKCTRL_IDLEST_SHIFT;
    const uint32_t clkctrl = AM335X_CM_PER_ADDR + AM335X_CM_PER_EPWMSS0_CLKCTRL;
    const uint32_t idle_bits = AM335X_CM_PER_EPWMSS0_CLKCTRL_IDLEST;
    const uint32_t is_enable = AM335X_CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_ENABLE;
    const uint32_t module_mode = AM335X_CM_PER_EPWMSS0_CLKCTRL_MODULEMODE;
    
    REG(clkctrl) |= is_enable;
    while((REG(clkctrl) & module_mode) != is_enable);
    while((REG(clkctrl) & idle_bits) != is_functional);
    }
    else if(pwmss_id == BBB_PWMSS1) {
    const uint32_t is_functional = AM335X_CM_PER_EPWMSS1_CLKCTRL_IDLEST_FUNC <<
                         AM335X_CM_PER_EPWMSS1_CLKCTRL_IDLEST_SHIFT;
    const uint32_t clkctrl = AM335X_CM_PER_ADDR + AM335X_CM_PER_EPWMSS1_CLKCTRL;
    const uint32_t idle_bits = AM335X_CM_PER_EPWMSS1_CLKCTRL_IDLEST;
    const uint32_t is_enable = AM335X_CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_ENABLE;
    const uint32_t module_mode = AM335X_CM_PER_EPWMSS1_CLKCTRL_MODULEMODE;

    REG(clkctrl) |= is_enable;
    while((REG(clkctrl) & module_mode) != is_enable);
    while((REG(clkctrl) & idle_bits) != is_functional);
    } else if(pwmss_id == BBB_PWMSS2) {
    const uint32_t is_functional = AM335X_CM_PER_EPWMSS2_CLKCTRL_IDLEST_FUNC <<
                         AM335X_CM_PER_EPWMSS2_CLKCTRL_IDLEST_SHIFT;
    const uint32_t clkctrl = AM335X_CM_PER_ADDR + AM335X_CM_PER_EPWMSS2_CLKCTRL;
    const uint32_t idle_bits = AM335X_CM_PER_EPWMSS2_CLKCTRL_IDLEST;
    const uint32_t is_enable = AM335X_CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_ENABLE;
    const uint32_t module_mode = AM335X_CM_PER_EPWMSS2_CLKCTRL_MODULEMODE;

    REG(clkctrl) |= is_enable;
    while((REG(clkctrl) & module_mode) != is_enable);
    while((REG(clkctrl) & idle_bits) != is_functional);
    } else 
	is_valid = false;
  return is_valid;
}

bool beagle_pwm_init(BBB_PWMSS pwmss_id)
{
  const bool id_is_valid = pwmss_id < BBB_PWMSS_COUNT;
  bool status = true;
  if(id_is_valid) {
    module_clk_config(pwmss_id);
    pwm_clock_enable(pwmss_id);	
    pwmss_tbclk_enable(pwmss_id);
  } else {
      status =false;
  }
  return status;
}

int beagle_pwm_configure(BBB_PWMSS pwm_id, float pwm_freq, float duty_a, float duty_b)
{	
  uint32_t baseAddr;
  int status = 1;
  float Cycle = 0.0f,Divisor =0;
  unsigned int i,j;
  /* Go through README to know about dividers CLKDIV and HSPCLKDIV */
  const float CLKDIV_div[] = {1.0,2.0,4.0,8.0,16.0,32.0,64.0,128.0};
  const float HSPCLKDIV_div[] = {1.0, 2.0, 4.0, 6.0, 8.0, 10.0,12.0, 14.0};
  int NearCLKDIV =7,NearHSPCLKDIV =7,NearTBPRD =0;
	
  if(pwm_freq <= BBB_PWM_FREQ_THRESHOLD) {
    status =0;
  }

  if(duty_a < 0.0f || duty_a > 100.0f || duty_b < 0.0f || duty_b > 100.0f) {
    status = 0;
  }
  duty_a /= 100.0f;
  duty_b /= 100.0f;

  const uint16_t clkdiv_clear = REG16(baseAddr + AM335X_EPWM_TBCTL) &
  (~AM335X_EPWM_TBCTL_CLKDIV);
  const uint16_t clkdiv_write = (NearCLKDIV
  << AM335X_EPWM_TBCTL_CLKDIV_SHIFT) & AM335X_EPWM_TBCTL_CLKDIV;
  const uint16_t hspclkdiv_clear = REG16(baseAddr + AM335X_EPWM_TBCTL) &
  (~AM335X_EPWM_TBCTL_HSPCLKDIV);
  const uint16_t hspclkdiv_write = (NearHSPCLKDIV <<
  AM335X_EPWM_TBCTL_HSPCLKDIV_SHIFT) & AM335X_EPWM_TBCTL_HSPCLKDIV; 
  const uint16_t shadow_write_clear = REG16(baseAddr + AM335X_EPWM_TBCTL) &
  (~AM335X_EPWM_PRD_LOAD_SHADOW_MASK);
  const uint16_t shadow_write_disable = ((bool)AM335X_EPWM_SHADOW_WRITE_DISABLE <<
  AM335X_EPWM_TBCTL_PRDLD_SHIFT) & AM335X_EPWM_PRD_LOAD_SHADOW_MASK;
  const uint16_t ctrmode_clear = REG16(baseAddr + AM335X_EPWM_TBCTL) &
  (~AM335X_EPWM_COUNTER_MODE_MASK);
  const uint16_t ctrmode_set = ((unsigned int)AM335X_EPWM_COUNT_UP <<
  AM335X_TBCTL_CTRMODE_SHIFT) &  AM335X_EPWM_COUNTER_MODE_MASK;
   
  /** 10^9 /Hz compute time per cycle (ns) */
  Cycle = 1000000000.0f / pwm_freq;

  /** am335x provide (128* 14) divider and per TBPRD means 10ns when divider 
    * and max TBPRD is 65535 so max cycle is 128 * 8 * 14 * 65535 * 10ns */
  Divisor = (Cycle / 655350.0f);
	
  if(Divisor > (128 * 14))  {
		return 0;
  }  else  {
	for (i=0;i<8;i++)  {
	  for(j=0 ; j<8; j++)  {
	    if((CLKDIV_div[i] * HSPCLKDIV_div[j]) < (CLKDIV_div[NearCLKDIV] 
						* HSPCLKDIV_div[NearHSPCLKDIV]) && (CLKDIV_div[i] * HSPCLKDIV_div[j] > Divisor)) {
		NearCLKDIV = i;
		NearHSPCLKDIV = j;
	    }
	  }
	}

  baseAddr = select_pwm(pwm_id);	
  /*setting clock divider and freeze time base*/
  REG16(baseAddr + AM335X_EPWM_TBCTL) &= ~(AM335X_TBCTL_CLKDIV_MASK | AM335X_TBCTL_HSPCLKDIV_MASK);
  REG16(baseAddr + AM335X_EPWM_TBCTL) = clkdiv_clear | clkdiv_write;
  REG16(baseAddr + AM335X_EPWM_TBCTL) = hspclkdiv_clear | hspclkdiv_write;
  NearTBPRD = (Cycle / (10.0 * CLKDIV_div[NearCLKDIV] * HSPCLKDIV_div[NearHSPCLKDIV]));
  REG16(baseAddr + AM335X_EPWM_TBCTL) = shadow_write_clear | shadow_write_disable;
  REG16(baseAddr + AM335X_EPWM_TBCTL) = ctrmode_clear | ctrmode_set;
  REG16(baseAddr + AM335X_EPWM_CMPB) = ((float)(NearTBPRD) * duty_b);
  REG16(baseAddr + AM335X_EPWM_CMPA) = ((float)(NearTBPRD) * duty_a);
  REG16(baseAddr + AM335X_EPWM_TBPRD) = NearTBPRD;
  REG16(baseAddr + AM335X_EPWM_TBCNT) = 0;
  }
  return status;
}

bool beagle_pwm_enable(BBB_PWMSS pwmid)
{
  const bool id_is_valid = pwmid < BBB_PWMSS_COUNT;
  bool status = true;
  if (id_is_valid)  {
    const uint32_t baseAddr = select_pwm(pwmid);
  /* Initially set EPWMxA o/p high , when increasing counter = CMPA toggle o/p of EPWMxA */
    REG16(baseAddr + AM335X_EPWM_AQCTLA) = AM335X_EPWM_AQCTLA_ZRO_XAHIGH | (AM335X_EPWM_AQCTLA_CAU_EPWMXATOGGLE << AM335X_EPWM_AQCTLA_CAU_SHIFT);
  /* Initially set EPWMxB o/p high , when increasing counter = CMPA toggle o/p of EPWMxB */  
    REG16(baseAddr + AM335X_EPWM_AQCTLB) = AM335X_EPWM_AQCTLB_ZRO_XBHIGH | (AM335X_EPWM_AQCTLB_CBU_EPWMXBTOGGLE << AM335X_EPWM_AQCTLB_CBU_SHIFT);
    REG16(baseAddr + AM335X_EPWM_TBCNT) = 0;
  /* Set counter mode : Up-count mode */
    REG16(baseAddr + AM335X_EPWM_TBCTL) |=  AM335X_TBCTL_FREERUN  | AM335X_TBCTL_CTRMODE_UP;
    return status;
  }  else  {
       status =false;
       return status;
  }
}

bool beagle_pwm_disable(BBB_PWMSS pwmid)
{
  const bool id_is_valid = pwmid < BBB_PWMSS_COUNT;
  bool status = true;
  if (id_is_valid) {
    const uint32_t baseAddr = select_pwm(pwmid);
    REG16(baseAddr + AM335X_EPWM_TBCTL) = AM335X_EPWM_TBCTL_CTRMODE_STOPFREEZE;
    REG16(baseAddr + AM335X_EPWM_AQCTLA) = AM335X_EPWM_AQCTLA_ZRO_XALOW | (AM335X_EPWM_AQCTLA_CAU_EPWMXATOGGLE << AM335X_EPWM_AQCTLA_CAU_SHIFT);
    REG16(baseAddr + AM335X_EPWM_AQCTLB) = AM335X_EPWM_AQCTLA_ZRO_XBLOW | (AM335X_EPWM_AQCTLB_CBU_EPWMXBTOGGLE << AM335X_EPWM_AQCTLB_CBU_SHIFT);
    REG16(baseAddr + AM335X_EPWM_TBCNT)  = 0;
    return status;
  }  else  {
 	status = false;
	return status;
  }
}

#endif

/* For support of BeagleboardxM */
#if IS_DM3730

/* Currently this section is just to satisfy
 * GPIO API and to make the build successful.
 * Later on support can be added here.
 */
bool beagle_pwm_init(BBB_PWMSS pwmss_id)
{
  return false;
}
bool beagle_pwm_disable(BBB_PWMSS pwmid)
{
  return false;
}
bool beagle_pwm_enable(BBB_PWMSS pwmid)
{
  return false;
}
int beagle_pwm_configure(BBB_PWMSS pwm_id, float pwm_freq, float duty_a, float duty_b)
{
  return -1;
}
bool beagle_pwm_pinmux_setup(bbb_pwm_pin_t pin_no, BBB_PWMSS pwm_id)
{
  return false;
}

#endif
