#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/test.h>
#include <bsp.h>
#include <bsp/gpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <bsp/bbb-pwm.h>
const char rtems_test_name[] = "Punit PWM test GSOC 2016";
rtems_printer rtems_test_printer;

static void inline delay_sec(int sec)
{
  rtems_task_wake_after(sec*rtems_clock_get_ticks_per_second());
}  

rtems_task Init(rtems_task_argument argument);

rtems_task Init(
	rtems_task_argument ignored
)
{
	rtems_test_begin();
	printf("Starting PWM Testing");

/* Initialization PWM API*/
rtems_gpio_initialize();
pwm_init(BBBIO_PWMSS2);
PWMSS_TB_clock_check(BBBIO_PWMSS2);

       float PWM_HZ = 1.0f ;   /* 100 Hz */
       float duty_A = 50.0f ;    /* 20% Duty cycle for PWM 0_A output */
       const float duty_B = 50.0f ;    /* 50% Duty cycle for PWM 0_B output*/

        

      	pwmss_setting(BBBIO_PWMSS2, PWM_HZ ,duty_A , duty_B);
	printf("Hello 1 2 3..... \n"); 
        printf("PWM  enable for 10s ....\n");
        ehrpwm_enable(BBBIO_PWMSS2);
	delay_sec(10);
	
	ehrpwm_disable(BBBIO_PWMSS2);
	printf("close\n");

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

