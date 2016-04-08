/*
This is sample application to test PWM driver 
Hard-coded configuration of other parameters: 
Instance -2
Duty cycle - 50%
Synchronization - disabled 
Dead-band - disabled 
PWM-chopper - disabled 
High resolution PWM - disabled 
Trip-zone - disabled  

This application demostratest generation of PWM B-channel 
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include<rtems/test.h>
#include<bsp.h>
#include<bsp/gpio.h>
#include<stdio.h>
#include<stdlib.h>
#include<bsp/bbb-pwm.h>
#define CLOCK_DIV_VAL                 (10)
#define SOC_EHRPWM_2_MODULE_FREQ      (100000000)

const char rtems_test_name[] = "Punit PWM test GSOC 2016";

rtems_task Init(
	rtems_task_argument ignored
)
{
	rtems_test_begin();
	printf("Starting PWM Testing");

/* Initialization PWM API*/
rtems_gpio_initialize();
EHRPWMTimebaseClkConfig(SOC_EPWM_2_REGS,SOC_EHRPWM_2_MODULE_FREQ/CLOCK_DIV_VAL,SOC_EHRPWM_2_MODULE_FREQ);

EHRPWMPWMOpFreqSet(SOC_EPWM_2_REGS,
                       SOC_EHRPWM_2_MODULE_FREQ/CLOCK_DIV_VAL,
                       (unsigned int)(SOC_EHRPWM_2_MODULE_FREQ/CLOCK_DIV_VAL)/0xFF,
                       (unsigned int)EHRPWM_COUNT_UP,
                       (bool)EHRPWM_SHADOW_WRITE_DISABLE);
/* Disable synchronization*/
    EHRPWMTimebaseSyncDisable(SOC_EPWM_2_REGS);

    /* Disable syncout*/
    EHRPWMSyncOutModeSet(SOC_EPWM_2_REGS, EHRPWM_SYNCOUT_DISABLE);

    /* Configure the emulation behaviour*/
    EHRPWMTBEmulationModeSet(SOC_EPWM_2_REGS, EHRPWM_STOP_AFTER_NEXT_TB_INCREMENT);
/* Configure Counter compare cub-module */
    /* Load Compare A value */
    EHRPWMLoadCMPA(SOC_EPWM_2_REGS,
                   50,
                   (bool)EHRPWM_SHADOW_WRITE_DISABLE,
                   (unsigned int)EHRPWM_COMPA_NO_LOAD,
                   (bool)EHRPWM_CMPCTL_OVERWR_SH_FL);

    /* Load Compare B value */
    EHRPWMLoadCMPB(SOC_EPWM_2_REGS,
                   200,
                   (bool)EHRPWM_SHADOW_WRITE_DISABLE,
                   (unsigned int) EHRPWM_COMPB_NO_LOAD,
                   (bool)EHRPWM_CMPCTL_OVERWR_SH_FL);
/* Configure Action qualifier */
    /* Toggle when CTR = CMPA */
    EHRPWMConfigureAQActionOnB(SOC_EPWM_2_REGS,
                               EHRPWM_AQCTLB_ZRO_DONOTHING,
                               EHRPWM_AQCTLB_PRD_DONOTHING,
                               EHRPWM_AQCTLB_CAU_EPWMXBTOGGLE,
                               EHRPWM_AQCTLB_CAD_DONOTHING,
                               EHRPWM_AQCTLB_CBU_DONOTHING,
                               EHRPWM_AQCTLB_CBD_DONOTHING,
                               EHRPWM_AQSFRC_ACTSFB_DONOTHING);

    /* Bypass dead band sub-module */
    EHRPWMDBOutput(SOC_EPWM_2_REGS, EHRPWM_DBCTL_OUT_MODE_BYPASS);

    /* Disable Chopper sub-module */
    EHRPWMChopperDisable(SOC_EPWM_2_REGS);
    /* Disable trip events */
    EHRPWMTZTripEventDisable(SOC_EPWM_2_REGS,(bool)EHRPWM_TZ_ONESHOT);
    EHRPWMTZTripEventDisable(SOC_EPWM_2_REGS,(bool)EHRPWM_TZ_CYCLEBYCYCLE);

    /* Event trigger */
    /* Generate interrupt every 3rd occurance of the event */
    EHRPWMETIntPrescale(SOC_EPWM_2_REGS, EHRPWM_ETPS_INTPRD_THIRDEVENT);
    /* Generate event when CTR = CMPB */
    EHRPWMETIntSourceSelect(SOC_EPWM_2_REGS, EHRPWM_ETSEL_INTSEL_TBCTREQUCMPBINC);

    /* Disable High resolution capability */
    EHRPWMHRDisable(SOC_EPWM_2_REGS);




printf("PWM Test Completed\n");
rtems_test_end();
exit(0);
}

/* NOTICE: the clock driver is enabled */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

#define CONFIGURE_MAXIMUM_TASKS            1
#define CONFIGURE_USE_DEVFS_AS_BASE_FILESYSTEM

#define CONFIGURE_MAXIMUM_SEMAPHORES    1

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE 

#define CONFIGURE_EXTRA_TASK_STACKS         (2 * RTEMS_MINIMUM_STACK_SIZE)

#define CONFIGURE_INITIAL_EXTENSIONS RTEMS_TEST_INITIAL_EXTENSION

#define CONFIGURE_INIT
#include <rtems/confdefs.h>


