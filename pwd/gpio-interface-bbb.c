/* This file should be added at ..libbsp/arm/beagle/gpio/ */

/**
 * @file gpio-interface-bbb.c
 *
 * @ingroup Beaglebone Black
 *
 * @brief Beaglebone Black GPIO interface definitions
 *
 */

/*
 *  Copyright (c) 2016 Punit Vara <punitvara@gmail.com>
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.org/license/LICENSE.
 */
#define EPWM_PIN_COUNT 12

const rtems_gpio_pin_conf epwm_config[EPWM_PIN_COUNT] = {
	{/*EHRPWM 0B*/
	.pin_number = 143,
	.function = BSP_SPECIFIC,
	.pull_mode = NO_PULL_RESISTOR,
	.interrupt = NULL,
	.output_enabled = TRUE,
	.logic_invert = FALSE,
	.bsp_specific = &alt_func_def[1]
	},
	{/*EHRPWM 0B*/
	.pin_number = 2,
	.function = BSP_SPECIFIC,
	.pull_mode = NO_PULL_RESISTOR,
	.interrupt = NULL,
	.output_enabled = TRUE,
	.logic_invert = FALSE,
	.bsp_specific = &alt_func_def[3]
	},
	{/*EHRPWM 0A*/
	.pin_number = 142,
	.function = BSP_SPECIFIC,
	.pull_mode = NO_PULL_RESISTOR,
	.interrupt = NULL,
	.output_enabled = TRUE,
	.logic_invert = FALSE,
	.bsp_specific = &alt_func_def[1]
	},
	{/*EHRPWM 0A*/
	.pin_number = 3,
	.function = BSP_SPECIFIC,
	.pull_mode = NO_PULL_RESISTOR,
	.interrupt = NULL,
	.output_enabled = TRUE,
	.logic_invert = FALSE,
	.bsp_specific = &alt_func_def[3]
	},
	{/*EHRPWM 1B*/
	.pin_number = 85,
	.function = BSP_SPECIFIC,
	.pull_mode = NO_PULL_RESISTOR,
	.interrupt = NULL,
	.output_enabled = TRUE,
	.logic_invert = FALSE,
	.bsp_specific = &alt_func_def[6]
	},
	{/*EHRPWM 1B*/
	.pin_number = 113,
	.function = BSP_SPECIFIC,
	.pull_mode = NO_PULL_RESISTOR,
	.interrupt = NULL,
	.output_enabled = TRUE,
	.logic_invert = FALSE,
	.bsp_specific = &alt_func_def[2]
	},
	{/*EHRPWM 1A*/
	.pin_number = 84,
	.function = BSP_SPECIFIC,
	.pull_mode = NO_PULL_RESISTOR,
	.interrupt = NULL,
	.output_enabled = TRUE,
	.logic_invert = FALSE,
	.bsp_specific = &alt_func_def[6]
	},
	{/*EHRPWM 1A*/
	.pin_number = 112,
	.function = BSP_SPECIFIC,
	.pull_mode = NO_PULL_RESISTOR,
	.interrupt = NULL,
	.output_enabled = TRUE,
	.logic_invert = FALSE,
	.bsp_specific = &alt_func_def[2]
	},
	{/*EHRPWM 2B*/
	.pin_number = 23,
	.function = BSP_SPECIFIC,
	.pull_mode = NO_PULL_RESISTOR,
	.interrupt = NULL,
	.output_enabled = TRUE,
	.logic_invert = FALSE,
	.bsp_specific = &alt_func_def[4]
	},
	{/*EHRPWM 2B*/
	.pin_number = 103,
	.function = BSP_SPECIFIC,
	.pull_mode = NO_PULL_RESISTOR,
	.interrupt = NULL,
	.output_enabled = TRUE,
	.logic_invert = FALSE,
	.bsp_specific = &alt_func_def[3]
	},
	{/*EHRPWM 2A*/
	.pin_number = 22,
	.function = BSP_SPECIFIC,
	.pull_mode = NO_PULL_RESISTOR,
	.interrupt = NULL,
	.output_enabled = TRUE,
	.logic_invert = FALSE,
	.bsp_specific = &alt_func_def[4]
	},
	{/*EHRPWM 2A*/
	.pin_number = 102,
	.function = BSP_SPECIFIC,
	.pull_mode = NO_PULL_RESISTOR,
	.interrupt = NULL,
	.output_enabled = TRUE,
	.logic_invert = FALSE,
	.bsp_specific = &alt_func_def[3]
	},
};
