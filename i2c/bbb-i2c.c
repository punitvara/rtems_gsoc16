/**
 * @file
 *
 * @ingroup arm_beagle
 *
 * @brief BeagleBoard I2C bus initialization and API Support.
 */

/*
 * Copyright (c) 2016 Punit Vara <punitvara at gmail.com>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#include <stdio.h>
#include <bsp/i2c.h>
#include <libcpu/am335x.h>
#include <rtems/irq-extension.h>
#include <bsp/bbb-gpio.h>
#include <rtems/score/assert.h>

static void am335x_i2c0_pinmux(bbb_i2c_bus *bus)
{
  REG(bus->regs + AM335X_CONF_I2C0_SDA) =
  (BBB_RXACTIVE | BBB_SLEWCTRL | BBB_PU_EN);

  REG(bus->regs + AM335X_CONF_I2C0_SCL) =
  (BBB_RXACTIVE | BBB_SLEWCTRL | BBB_PU_EN); 
}

static void I2C0ModuleClkConfig(void)
{
    /* Configuring L3 Interface Clocks. */

    /* Writing to MODULEMODE field of CM_PER_L3_CLKCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKCTRL) |=
          CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_MODULEMODE));

    /* Writing to MODULEMODE field of CM_PER_L3_INSTR_CLKCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3_INSTR_CLKCTRL) |=
          CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3_CLKSTCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKSTCTRL) |=
          CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /* Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_OCPWP_L3_CLKSTCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_OCPWP_L3_CLKSTCTRL) |=
          CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    REG(AM335X_CM_PER_ADDR + CM_PER_L3S_CLKSTCTRL) |=
          CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3S_CLKSTCTRL) &
           CM_PER_L3S_CLKSTCTRL_CLKTRCTRL));

    /* Checking fields for necessary values.  */

    /* Waiting for IDLEST field in CM_PER_L3_CLKCTRL register to be set to 0x0. */
    while((CM_PER_L3_CLKCTRL_IDLEST_FUNC << CM_PER_L3_CLKCTRL_IDLEST_SHIFT)!=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_IDLEST));

    /*
    ** Waiting for IDLEST field in CM_PER_L3_INSTR_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_L3_INSTR_CLKCTRL_IDLEST_FUNC <<
           CM_PER_L3_INSTR_CLKCTRL_IDLEST_SHIFT)!=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_GCLK field in CM_PER_L3_CLKSTCTRL register to
    ** attain the desired value.
    */
    while(CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_OCPWP_L3_GCLK field in CM_PER_OCPWP_L3_CLKSTCTRL
    ** register to attain the desired value.
    */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L3S_GCLK field in CM_PER_L3S_CLKSTCTRL register
    ** to attain the desired value.
    */
    while(CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK !=
          (REG(AM335X_CM_PER_ADDR + CM_PER_L3S_CLKSTCTRL) &
          CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK));


    /* Configuring registers related to Wake-Up region. */

    /* Writing to MODULEMODE field of CM_WKUP_CONTROL_CLKCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) |=
          CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) |=
          CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_L3_AON_CLKSTCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) |=
          CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL));

    /* Writing to MODULEMODE field of CM_WKUP_I2C0_CLKCTRL register. */
    REG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) |=
          CM_WKUP_I2C0_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_I2C0_CLKCTRL_MODULEMODE_ENABLE !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) &
           CM_WKUP_I2C0_CLKCTRL_MODULEMODE));

    /* Verifying if the other bits are set to required settings. */

    /*
    ** Waiting for IDLEST field in CM_WKUP_CONTROL_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_CONTROL_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_CONTROL_CLKCTRL_IDLEST_SHIFT) !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_AON_GCLK field in CM_L3_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_L4WKUP_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_L4WKUP_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST_SHIFT) !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL) &
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_GCLK field in CM_WKUP_CLKSTCTRL register
    ** to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_AON_GCLK field in CM_L4_WKUP_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL) &
           CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK));

    /*
    ** Waiting for CLKACTIVITY_I2C0_GFCLK field in CM_WKUP_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_I2C0_GFCLK !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_I2C0_GFCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_I2C0_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_I2C0_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_I2C0_CLKCTRL_IDLEST_SHIFT) !=
          (REG(SOC_CM_WKUP_REGS + CM_WKUP_I2C0_CLKCTRL) &
           CM_WKUP_I2C0_CLKCTRL_IDLEST));
}

static bool am335x_i2c_busbusy(volatile bbb_i2c_regs *regs)
{
  bool status;

  
  if (REG(&regs->BBB_I2C_IRQSTATUS_RAW) & AM335X_I2C_IRQSTATUS_RAW_BB)
  {
    status = true; 
  } else {
    status = false;
  }
  udelay(1000000);
  return status; 
}

static void am335x_i2c_reset(bbb_i2c_bus *bus)
{
  volatile bbb_i2c_regs *regs = bus->regs;
  
  printk("reset bus->reg is %x \n",bus->regs);
  printk("inside BBB_I2C_CON value is %x \n",&regs->BBB_I2C_CON);
  REG(&regs->BBB_I2C_CON) = 0;
  udelay(50000);
  mmio_clear((&regs->BBB_I2C_SYSC),AM335X_I2C_SYSC_AUTOIDLE);  
}  

/*
Possible values for msg->flag 
   * - @ref I2C_M_TEN,
   * - @ref I2C_M_RD,
   * - @ref I2C_M_STOP,
   * - @ref I2C_M_NOSTART,
   * - @ref I2C_M_REV_DIR_ADDR,
   * - @ref I2C_M_IGNORE_NAK,
   * - @ref I2C_M_NO_RD_ACK, and
   * - @ref I2C_M_RECV_LEN.
*/

static unsigned int am335x_i2c_intrawstatus(volatile bbb_i2c_regs *regs)
{
  return (REG(&regs->BBB_I2C_IRQSTATUS_RAW));
}

static void am335x_int_clear(volatile bbb_i2c_regs *regs, unsigned int flag)
{
  REG(&regs->BBB_I2C_IRQSTATUS) = flag;
}

static uint32_t am335x_i2c_poll(volatile bbb_i2c_regs *regs, uint32_t mask)
{
  uint32_t status;

  status = am335x_i2c_intrawstatus(regs);

  if((status & mask) != 0) {
  return status;
  }
  udelay(1000000);
  return status; 
}

static void am335x_i2c_flush(volatile bbb_i2c_regs *regs)
{
  int tries;
  int status;

  for (tries=0; tries < 1000; tries++) {
    status = am335x_i2c_poll(regs, (1 << 3)); //RRDY
    
    if ((status & (1 <<3)) !=0)  {
    (void)REG(&regs->BBB_I2C_DATA);
    am335x_int_clear(regs,(1 << 3));
    
  } else {
    break; // buffer drained
  }
  }
}

static void am335x_i2c_setup_read_transfer(bbb_i2c_bus *bus, volatile bbb_i2c_regs *regs, const i2c_msg *msgs, bool send_stop)
{
  int r,i;
  uint32_t conopts,pollmask,errmask,no_bytes;

  conopts =0;
 
  if ((msgs->flags & I2C_M_TEN) !=0 ) {
    conopts |= (1 << 8); //XSA
  }
  
  pollmask = 0;
  pollmask |= I2C_STAT_RRDY; //RRDY=3

  errmask = 0;
  errmask |= (1 << 11); //ROVR
  errmask |= (1 << 7); //AERR
  errmask |= (1 << 1); //NACK
  errmask |= (1 << 0); //AL
  
  REG(&regs->BBB_I2C_CNT) = bus->msg_todo;
  no_bytes =  REG(&regs->BBB_I2C_CNT);
  REG(&regs->BBB_I2C_SA) = msgs->addr;

 /* Set control register */
  conopts |= (1 << 15);   /* enabled */ //I2C_EN
  conopts |= (1 << 10);   /* master mode */ // MST
  conopts |= (1 << 0); // STT
  
  if (send_stop) {
    conopts |= (1 <<1); // STP stop condition
  } 
 
  // I2C Controller in Master Mode
  REG(&regs->BBB_I2C_CON) = conopts;
 
  for (i=0; i < no_bytes; i++) {
    r = am335x_i2c_poll(regs, (pollmask | errmask));

    if ((r & errmask) != 0) {
      printf("read error\n");
    } else if ((r & pollmask) == 0) {
      printf("not ready for read ? \n");
    }
    
    msgs->buf[i] = REG(&regs->BBB_I2C_DATA) & 0xff;
  
    am335x_int_clear(regs, pollmask);
  }
 
  r = am335x_i2c_intrawstatus(regs);

  if ((r & (1 << 1)) != 0){
    printf("read NACK \n");
  }

  pollmask = (1 << 2); //ARDY

  r = am335x_i2c_poll(regs, pollmask);
  
  if ((r & pollmask) == 0) {
    printf("read operation never finish\n");
  }

  am335x_int_clear(regs,0x7fff);

}

static void am335x_i2c_setup_write_transfer(bbb_i2c_bus *bus, const i2c_msg *msgs, volatile bbb_i2c_regs *regs, bool send_stop)
{
  volatile unsigned int no_bytes;
  unsigned int i,r,errmask,pollmask,conopts;
 
  conopts = 0;
  
  if ((msgs->flags & I2C_M_TEN) !=0 ) {
    conopts |= (1 << 8); //XSA
  }
  pollmask = 0;
  pollmask |= I2C_STAT_XRDY; //XRDY=4

  errmask = 0;
  errmask |= (1 << 11); //ROVR
  errmask |= (1 << 7); //AERR
  errmask |= (1 << 1); //NACK
  errmask |= (1 << 0); //AL
 
  // Following data count specify bytes to be transmitted
  REG(&regs->BBB_I2C_CNT) = bus->msg_todo;
  no_bytes = REG(&regs->BBB_I2C_CNT);
  REG(&regs->BBB_I2C_SA) = msgs->addr;

  /* Set control register */
	conopts |= (1 << 15);	/* enabled */ //I2C_EN
	conopts |= (1 << 10);	/* master mode */ // MST
	conopts |= (1 << 9);	/* TRX mode */ //TRX
	conopts |= (1 << 0);	/* start condition */ //STT  
  
  if (send_stop) {
    conopts |= (1 << 1); //stop condition // STP
  }
  
  // clear status of all interrupts
  // Already cleaned during reset
  am335x_int_clear(regs,0x7fff);    
  // I2C Controller in Master transmitter Mode
  REG(&regs->BBB_I2C_CON) = conopts;
  
  for (i=0; i < no_bytes; i++) {
    r = am335x_i2c_poll(regs, (pollmask | errmask));
    if ((r & errmask) != 0) {
      printf("write error\n");
    } else if ((r & pollmask) == 0) {
      printf("not ready for write ? \n");
    }
    REG(&regs->BBB_I2C_DATA) = msgs->buf[i]; 
    am335x_int_clear(regs,pollmask);
  }
  r = am335x_i2c_intrawstatus(regs);
  
  if ((r & (1 << 1)) != 0){
    printf("NACK \n");
  }
  pollmask = (1 << 2); //ARDY

  r = am335x_i2c_poll(regs,pollmask);
  if ((r & pollmask) == 0) {
    printf("write operation never finish\n");
  }
  am335x_int_clear(regs,0x7fff);
 
}


static void am335x_i2c_setup_transfer(bbb_i2c_bus *bus, volatile bbb_i2c_regs *regs)
{
  const i2c_msg *msgs = bus->msgs;
  uint32_t msg_todo = bus->msg_todo;
  bool send_stop = false;
  int r;

  printk("Enter setup transfer\n");
  
  regs = bus->regs;
  
  //am335x_i2c_set_address_size(msgs,regs);
  bus->read = ((bus->read == true) ? 0:1); 
  bus->already_transferred = (bus->read == true) ? 0 : 1;
  
  am335x_i2c_flush(regs);

  r = am335x_i2c_busbusy(regs);
  
  if (r == 0) {
    printf("bus is busy \n");
  }
 
  if (bus->read) {
    
    if (REG(&regs->BBB_I2C_CNT) == 1) {
      send_stop = true;
    }

    printk("configure to read bus\n");
    am335x_i2c_setup_read_transfer(bus,regs,msgs,send_stop);
  
  } else {
    printk("configure to write bus\n");
    am335x_i2c_setup_write_transfer(bus,msgs,regs,send_stop);
  }
  
}


static int am335x_i2c_transfer(i2c_bus *base, i2c_msg *msgs, uint32_t msg_count)
{
  rtems_status_code sc;
  bbb_i2c_bus *bus = (bbb_i2c_bus *)base;
  volatile bbb_i2c_regs *regs;
  uint32_t i;
  printk("\n enter transfer ");
  rtems_task_wake_after(1);
  

  if (msg_count < 1){
    return 1;
  }
 
  for (i=0; i<msg_count;++i) {
      if ((msgs[i].flags & I2C_M_RECV_LEN) != 0) {
        return -EINVAL;
      }
  }
  
  bus->msgs = &msgs[0];
  bus->msg_todo = msg_count;
  printk("total msg = msg_count : %x \n",bus->msg_todo);
  bus->task_id = rtems_task_self();

  regs = bus->regs;
  am335x_i2c_setup_transfer(bus,regs);
  REG(&regs->BBB_I2C_IRQENABLE_SET) = BBB_I2C_IRQ_USED;

  sc = rtems_event_transient_receive(RTEMS_WAIT, bus->base.timeout);
  // If timeout then return timeout error
  if (sc != RTEMS_SUCCESSFUL) {
    am335x_i2c_reset(bus);

    rtems_event_transient_clear();

    return -ETIMEDOUT;
  }
  printk("exit transfer\n");
  // return bus->regs->BBB_I2C_IRQSTATUS == 0 ? 0 : -EIO;
  return 0;
}

static int am335x_i2c_set_clock(i2c_bus *base, unsigned long clock)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  volatile bbb_i2c_regs *regs = bus->regs;
  uint32_t prescaler,divider;

  printk("set clock start\n"); 
  prescaler = (BBB_I2C_SYSCLK / BBB_I2C_INTERNAL_CLK) -1;
  printk("PSC offset %x \n ",&regs->BBB_I2C_PSC);
  printk("PSC offset %x \n", &bus->regs->BBB_I2C_PSC);
  //mmio_write((&regs->BBB_I2C_PSC), prescaler);
  REG(&bus->regs->BBB_I2C_PSC) = prescaler;
  
  divider = BBB_I2C_INTERNAL_CLK/(2*clock);
  printk("SCLL offset %x \n",&bus->regs->BBB_I2C_SCLL); 
  //mmio_write((&regs->BBB_I2C_SCLL), (divider - 7));
  REG(&bus->regs->BBB_I2C_SCLL) = (divider - 7);
  //mmio_write((&regs->BBB_I2C_SCLH), (divider - 5));
  printk("SCHL offset %x\n",&bus->regs->BBB_I2C_SCLH);
  REG(&bus->regs->BBB_I2C_SCLH) = (divider - 5);
  printk("set clock end \n");
  return 0;
}

static void am335x_i2c_destroy(i2c_bus *base)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  rtems_status_code sc;
  
  printk(" starting destroy\n"); 
  sc = bsp_interrupt_vector_disable(bus->irq);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void)sc;
  printk("end destroy\n");
  i2c_bus_destroy_and_free(&bus->base);
}

int am335x_i2c_bus_register(
  const char *bus_path,
  uintptr_t register_base,
  uint32_t input_clock,
  rtems_vector_number irq
)
{
  
  bbb_i2c_bus *bus;
  int err;
  rtems_status_code r;
  uint32_t intmask;
  static int policy_set = 0;
  static int enabled = 0;

  /*check bus number is >0 & <MAX*/

  bus = (bbb_i2c_bus *) i2c_bus_alloc_and_init(sizeof(*bus));
  
  if (bus == NULL) {
    return -1;
  }

  bus->regs = (volatile bbb_i2c_regs *) register_base;
 
// 1. Enable clock for I2CX
  I2C0ModuleClkConfig();
// 2. pinmux setup
  am335x_i2c0_pinmux(bus);
  REG(&bus->regs->BBB_I2C_CON) =0;
  udelay(50000);
  //soft reset
  REG(&bus->regs->BBB_I2C_SYSC) = 1 << 1;
  REG(&bus->regs->BBB_I2C_CON) = 1 << 15;
  udelay(50000);
  while(REG(&bus->regs->BBB_I2C_SYSS) & (1 << 0)); 
// 3. RESET : Disable Master, autoideal 
  am335x_i2c_reset(bus);
  
// 4. configure bus speed  
  bus->input_clock = input_clock; // By default 100KHz. Normally pass 100KHz as argument 
 
  printk("Before set clock \n"); 
  err = am335x_i2c_set_clock(&bus->base, I2C_BUS_CLOCK_DEFAULT);
 
  if (err != 0) {
    (*bus->base.destroy)(&bus->base);
    
    rtems_set_errno_and_return_minus_one(-err);
  }
   bus->irq = irq;
 
  REG(&bus->regs->BBB_I2C_OA) = 0x01;
  REG(&bus->regs->BBB_I2C_BUF) = 0x0000; 
  //bring I2C out of reset

  REG(&bus->regs->BBB_I2C_CON) |= AM335X_I2C_CON_I2C_EN;
  udelay(50000);
  // 5. Start interrupt service routine & one interrupt at a time 
 
  if (!policy_set) {
    r = bsp_interrupt_facility_initialize();
    if (r == RTEMS_SUCCESSFUL) {
      policy_set = 1;
    } else {
        printk("couldn't set irq facility\n");
    }
  }
  
  if (policy_set && !enabled) {
    r = bsp_interrupt_vector_enable(irq);
    if (r == RTEMS_SUCCESSFUL) {
      enabled =1;
    } else {
        printk("couldn't enable irq \n");
    }
  }

  intmask = 0;
  intmask |= (1 << 11); //ROVR
  intmask |= (1 << 7); //AERR
  intmask |= (1 << 4); // XRDY
  intmask |= (1 << 3); //RRDY
  intmask |= (1 << 2); //ARDY
  intmask |= (1 << 1); //NACK
  intmask |= (1 << 0); //AL

  REG(&bus->regs->BBB_I2C_IRQENABLE_SET) = intmask;
 
  // 6. start transfer for reading and writing 
  bus->base.transfer = am335x_i2c_transfer;
  bus->base.set_clock = am335x_i2c_set_clock;
  bus->base.destroy = am335x_i2c_destroy;
  printk("exit register\n");
  return i2c_bus_register(&bus->base,bus_path);
}
