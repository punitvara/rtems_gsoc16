#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtems/test.h>
#include <bsp/beagleboneblack.h> /* Calls the BBB specific library */
#include <bsp/gpio.h> /* Calls the BSP gpio library */
#include <bsp/bbb-gpio.h>
#include <stdio.h>
#include <stdlib.h>

static void inline delay_sec(int sec)
{
  rtems_task_wake_after(sec*rtems_clock_get_ticks_per_second());
}
/* forward declarations to avoid warnings */
rtems_task Init(rtems_task_argument argument);

const char rtems_test_name[] = "Punit Gpio Test GSOC 2016 ";

rtems_task Init(
  rtems_task_argument ignored
)
{
  rtems_test_begin();
  printf("Starting Gpio Testing\n");
  
  /* Intializing GPIO API */
  rtems_gpio_initialize();
  
  rtems_gpio_request_pin(BBB_P8_14,DIGITAL_OUTPUT,TRUE,FALSE,NULL); 
  rtems_gpio_bsp_select_output(0,BBB_P8_14,NULL); 
  rtems_gpio_set(BBB_P8_14); 
  rtems_gpio_bsp_set(0,BBB_P8_14);
  delay_sec(1);
  rtems_gpio_clear(BBB_P8_14);
  rtems_gpio_bsp_clear(0,BBB_P8_14);
  delay_sec(1);
  rtems_gpio_release_pin(BBB_P8_14); 
 
  rtems_gpio_request_pin(BBB_P8_13,DIGITAL_OUTPUT,TRUE,FALSE,NULL);
  rtems_gpio_bsp_select_output(0,BBB_P8_13,NULL);
  rtems_gpio_set(BBB_P8_13);
  rtems_gpio_bsp_set(0,BBB_P8_13);
  delay_sec(1);
  rtems_gpio_clear(BBB_P8_13);
  rtems_gpio_bsp_clear(0,BBB_P8_13);
  delay_sec(1);
  rtems_gpio_release_pin(BBB_P8_13);

  rtems_gpio_request_pin(BBB_P8_12,DIGITAL_OUTPUT,TRUE,FALSE,NULL);
  rtems_gpio_bsp_select_output(1,12,NULL);
  rtems_gpio_set(BBB_P8_12);
  rtems_gpio_bsp_set(1,12);
  delay_sec(1);
  rtems_gpio_clear(BBB_P8_12);
  rtems_gpio_bsp_clear(1,12);
  delay_sec(1);
  rtems_gpio_release_pin(BBB_P8_12);
  
  printf("Gpio Test Completed\n");
  rtems_test_end();
  exit( 0 );
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
