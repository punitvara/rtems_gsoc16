/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief BeagleBone Black BSP definitions.
 */

/**
 * Copyright (c) 2016 Punit Vara <punitvara@gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

/** Some constants are taken from 
 * https://github.com/VegetableAvenger/BBBIOlib/blob/master/BBBio_lib/BBBiolib_PWMSS.h
 */

#ifndef LIBBSP_ARM_BEAGLE_BBB_PWM_H
#define LIBBSP_ARM_BEAGLE_BBB_PWM_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief  BeagleBone Black PWM Macros.
 */
#define BBBIO_PWMSS_COUNT       3
#define BBBIO_PWMSS0    	0
#define BBBIO_PWMSS1    	1  
#define BBBIO_PWMSS2    	2

#define MUXMODE0		0
#define MUXMODE1                1
#define MUXMODE2		2
#define MUXMODE3		3
#define MUXMODE4 		4
#define MUXMODE5		5
#define MUXMODE6		6
#define MUXMODE7		7

#define EPWM_GROUP1	1
#define EPWM_GROUP2	2
#define EPWM_GROUP0 	0

/**
 * @brief  BeagleBone Black PWM API.
 */
bool beagle_pwm_init(uint32_t pwmss_id);
int beagle_pwmss_setting(uint32_t pwm_id, float pwm_freq, float dutyA, float dutyB);
bool beagle_ehrpwm_enable(uint32_t pwmid);
bool beagle_ehrpwm_disable(uint32_t pwmid);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_BEAGLE_BBB_PWM_H */
