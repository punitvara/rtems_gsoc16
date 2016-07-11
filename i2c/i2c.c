/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief BeagleBoard I2C API support definitions.
 */

/*
 * Copyright (c) 2016 Punit Vara <punitvara at gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <bsp/i2c.h>

typedef enum{
  I2C1 = 0,
  I2C2,
  I2C_COUNT
}bbb_i2c_t;

void beagle_bsp_i2c_init(void)
{
  int bus_noi, minor_no;
  /*Initialize libi2c API library*/
  rtems_libi2c_initialize();
 
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
/*what is drvtbl here ????*/

}

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
