/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief BeagleBoard I2C bus initialization.
 */

/*
 * Copyright (c) 2016 Punit Vara <punitvara at gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <bsp/i2c.h>
#include <cpu/am335x.h>

typedef enum{
  I2C1 = 0,
  I2C2,
  I2C_COUNT
}bbb_i2c_t;

rtems_status_code beagle_i2c_init(rtems_libi2c_bus_t * bushdl)
{

}

rtems_status_code beagle_i2c_send_start(rtems_libi2c_bus_t * bushdl)
{

}

rtems_status_code beagle_i2c_send_stop(rtems_libi2c_bus_t * bushdl)
{

}

rtems_status_code beagle_i2c_send_addr(rtems_libi2c_bus_t * bushdl, 
					uint32_t addr, int rw)
{

}

int beagle_i2c_read_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes,
                     int nbytes)
{

}

int beagle_i2c_write_bytes(rtems_libi2c_bus_t * bushdl, unsigned char *bytes,
                      int nbytes)
{

}

int beagle_i2c_ioctl()
{

}
static rtems_libi2c_bus_ops_t beagle_ops = {
  .init = beagle_i2c_init,
  .send_start = beagle_i2c_send_start,
  .send_stop = beagle_i2c_send_stop,
  .send_addr = beagle_i2c_send_addr,
  .read_bytes = beagle_i2c_read_bytes,
  .write_bytes = beagle_i2c_write_bytes,
  .ioctl = beagle_i2c_ioctl
};

static static rtems_libi2c_drv_t beagle_drv_tbl = {
  .ops =         &beagle_ops,
  .size =        sizeof (beagle_drv_tbl),
};

void beagle_bsp_i2c_init(void)
{
  int bus_no, minor_no,i2caddr;
  /*Initialize libi2c API library*/
  rtems_libi2c_initialize();
/*  i2caddr = ADDR_CONST */
/**pre-driver hook
 * Bus registration + Device/Driver Registration 
 * int rtems_libi2c_register_bus (char *name,
                               rtems_libi2c_bus_t * bus)
 * By default name is /dev/i2c if *name == NULL
 */
/* bus_no = rtems_libi2c_register_bus(NULL,**) */
  

/**Device/Driver Registration
 * int
 * rtems_libi2c_register_drv (char *name, rtems_libi2c_drv_t * drvtbl,
                           unsigned bus, unsigned i2caddr);
 * minor_no = rtems_libi2c_register_drv (NAME,drvtbl,bus_no,ADDR_CONSTANT)
 */
  minor_no = rtems_libi2c_register_drv (NAME,&beagle_drv_tbl,bus_no,ADDR_CONSTANT);
}
/*
Pin_list 
P9_17  I2C1_SCL
P9_18  I2C1_SDA
P9_19  I2C2_SCL
P9_20  I2C2_SDA
P9_21  I2C2_SCL
P9_22  I2C2_SDA
P9_24  I2C1_SCL
P9_26  I2C1_SDA
*/
/*
May need to consider RXACTIVE and SLEWCTRL
*/
void i2c_pinmux(bbb_i2c_t i2c_id)
{
  const bool id_is_valid = i2c_id < I2C_COUNT;
  bool status = true;
  if (id_is_valid) {
    if (i2c_id == I2C1) {
      REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_CS0) = (BBB_MUXMODE(MODE2) | BBB_PU_EN); 
      REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_D1) = (BBB_MUXMODE(MODE2) | BBB_PU_EN);
      REG(AM335X_PADCONF_BASE + AM335X_CONF_UART1_TXD) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
      REG(AM335X_PADCONF_BASE + AM335X_CONF_UART1_RXD) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
    } else if (i2c_id == I2C2) {
      REG(AM335X_PADCONF_BASE + AM335X_CONF_UART1_RTSN) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
      REG(AM335X_PADCONF_BASE + AM335X_CONF_UART1_CTSN) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
      REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_D0) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
      REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_SCLK) = (BBB_MUXMODE(MODE3) | BBB_PU_EN);
    } else {
    status = false;
    }
  return status;
}

/* ref. Table 21-4 I2C Clock Signals */
/* 
 Interface clock - 100MHz - CORE_LKOUTM4 / 2 - pd_per_l4ls_gclk

 Functional clock - 48MHz - PER_CLKOUTM2 / 4 - pd_per_ic2_fclk
*/
void i2c1_i2c2_module_clk_config(bbb_i2c_t i2c_id)
{
/*0x2 = SW_WKUP : SW_WKUP: Start a software forced wake-up
transition on the domain. */

  REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKSTCTRL) |=
			AM335X_CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP; 
  while((REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKSTCTRL) &
			AM335X_CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL) !=
                        AM335X_CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

   
/* 0x2 = ENABLE : Module is explicitly enabled. Interface clock (if not
used for functions) may be gated according to the clock domain
state. Functional clocks are guarantied to stay present. As long as in
this configuration, power domain sleep transition cannot happen.*/
  REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKCTRL) |=
			AM335X_CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE;

  while((REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKCTRL) &
      AM335X_CM_PER_L4LS_CLKCTRL_MODULEMODE) != AM335X_CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE);

/*0x2 = ENABLE : Module is explicitly enabled. Interface clock (if not
used for functions) may be gated according to the clock domain
state. Functional clocks are guarantied to stay present. As long as in
this configuration, power domain sleep transition cannot happen.*/
  if (i2c_id == I2C1) {
  REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_I2C1_CLKCTRL) |=
                             AM335X_CM_PER_I2C1_CLKCTRL_MODULEMODE_ENABLE;

  while((REG((AM335X_CM_PER_ADDR + AM335X_CM_PER_I2C1_CLKCTRL) &
     AM335X_ CM_PER_I2C1_CLKCTRL_MODULEMODE) != AM335X_CM_PER_I2C1_CLKCTRL_MODULEMODE_ENABLE);
  } else if (i2c_id == I2C2) {
  REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_I2C2_CLKCTRL) |=
                             AM335X_CM_PER_I2C2_CLKCTRL_MODULEMODE_ENABLE;

  while((REG((AM335X_CM_PER_ADDR + AM335X_CM_PER_I2C2_CLKCTRL) &
     AM335X_CM_PER_I2C2_CLKCTRL_MODULEMODE) != AM335X_CM_PER_I2C2_CLKCTRL_MODULEMODE_ENABLE);

  while(!(REG(AM335X_CM_PER_ADDR + AM335X_CM_PER_L4LS_CLKSTCTRL) &
           (AM335X_CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK |
            AM335X_CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_I2C_FCLK)));

}

static uint32_t select_i2c(bbb_i2c_t i2c_id)
{
  uint32_t baseAddr=0;

  if (i2c_id == I2C1) {
    baseAddr = AM335X_I2C1_BASE;
  } else if (i2c_id == I2C2) {
    baseAddr = AM335X_I2C2_BASE;
  } else {
    baseAddr = 0;
  }
  return baseAddr;
}

void i2c_module_en_dis(bool status,bbb_i2c_t i2c_id)
{
  uint32_t baseAddr = select_i2c(i2c_id);
  
  if (status) {
    /* Bring the I2C module out of reset */
    REG(baseAddr + AM335X_I2C_CON) |= AM335X_I2C_CON_I2C_EN; 
  } else {
    /* Reset I2C module*/
    REG(baseAddr + AM335X_I2C_CON) &= ~(AM335X_I2C_CON_I2C_EN);
  }

}

 
