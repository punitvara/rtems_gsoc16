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

#ifndef LIBBSP_ARM_BEAGLE_BBB_PWM_H
#define LIBBSP_ARM_BEAGLE_BBB_PWM_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief  BeagleBone Black PWM functions.
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
void pwm_init(uint32_t pwmss_id);
int pwmss_setting(uint32_t pwm_id, float pwm_freq, float dutyA, float dutyB);
void ehrpwm_enable(uint32_t pwmid);
void ehrpwm_disable(uint32_t pwmid);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LIBBSP_ARM_BEAGLE_BBB_PWM_H */
