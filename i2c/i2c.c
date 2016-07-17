#include <dev/i2c/i2c.h>
#include <bsp/i2c.h>
#include <cpu/am335x.h>
#include <bsp.h>

#define BBB_I2C_SYSCLK 48000000
#define BBB_I2C_INTERNAL_CLK 12000000
#define BBB_I2C_SPEED_CLK 100000

#define BBB_I2C_IRQ_ERROR \
  (AM335X_I2C_IRQSTATUS_NACK \
    | AM335X_I2C_IRQSTATUS_ROVR \
    | AM335X_I2C_IRQSTATUS_AERR \
    | AM335X_I2C_IRQSTATUS_AL \
    | AM335X_I2C_IRQSTATUS_ARDY \
    | AM335X_I2C_IRQSTATUS_RRDY \
    | I2C_IRQSTATUS_XRDY)

#define BBB_I2C_IRQ_USED \
  (BBB_I2C_IRQ_ERROR \
    | AM335X_I2C_IRQSTATUS_AAS \
    | AM335X_I2C_IRQSTATUS_BF \
    | AM335X_I2C_IRQSTATUS_STC \
    | AM335X_I2C_IRQSTATUS_GC \
    | AM335X_I2C_IRQSTATUS_XDR \
    | AM335X_I2C_IRQSTATUS_RDR)

#define BBB_I2C_0_BUS_PATH "/dev/i2c-0"
#define BBB_I2C_1_BUS_PATH "/dev/i2c-1"
#define BBB_I2C_2_BUS_PATH "/dev/i2c-2"

static const i2c_base_addrs[] = {
	AM335X_I2C0_BASE, AM335X_I2C1_BASE, AM335X_I2C0_BASE
};

/*
ref : ch 6 TRM

30 I2C2INT
70 I2C0INT
71 I2C1INT
*/
#define BBB_I2C0_IRQ 70
#define BBB_I2C1_IRQ 71
#define BBB_I2C2_IRQ 30

static const i2c_irq_num[] = {
	BBB_I2C0_IRQ , BBB_I2C1_IRQ , BBB_I2C2_IRQ
};

typedef struct {
  i2c_bus base;
  volatile beagle_i2c_regs *regs;
  uint32_t i2c_base_regs;
  i2c_msg *msgs;
  rtems_id task_id;
  rtems_vector_number irq;
  int i2c_bus_id;
  }bbb_i2c_bus;

/*By default initialization*/
static bbb_i2c_bus beagle_i2c_bus = {
  {.transfer  = am335x_i2c_transfer,
  .set_clock = am335x_i2c_set_clock,
  .destroy = am335x_i2c_destroy,
  /*Need to search how to initialize mutex*/
  .default_address = 0x50,
  .ten_bit_address = false,
  .use_pec = false, /*Dont know about SMBus PEC*/
  .retries = 1,
  .timeout = 500,
  .functionality = 1 /*Dont know what is controller functionality*/
  },
  &am335_i2c_regs,
  i2c_base_addrs[0],  /*Default I2C[0] base address selected */
  /**do not know about i2c_msg*/
  /*yet to see how to initialize task_id*/
  i2c_irq_num[0]
  }; 
  
typedef struct i2c_regs 
{
  unsigned short BBB_I2C_REVNB_LO;	
  unsigned short BBB_I2C_REVNB_HI;	
  unsigned short BBB_I2C_SYSC;
  unsigned short BBB_I2C_IRQSTATUS_RAW;	
  unsigned short BBB_I2C_IRQSTATUS;	
  unsigned short BBB_I2C_IRQENABLE_SET;	
  unsigned short BBB_I2C_IRQENABLE_CLR;	
  unsigned short BBB_I2C_WE;
  unsigned short BBB_I2C_DMARXENABLE_SET;	
  unsigned short BBB_I2C_DMATXENABLE_SET;	
  unsigned short BBB_I2C_DMARXENABLE_CLR;	
  unsigned short BBB_I2C_DMATXENABLE_CLR;	
  unsigned short BBB_I2C_DMARXWAKE_EN;	
  unsigned short BBB_I2C_DMATXWAKE_EN;	
  unsigned short BBB_I2C_SYSS;
  unsigned short BBB_I2C_BUF;
  unsigned short BBB_I2C_CNT;
  unsigned short BBB_I2C_DATA;
  unsigned short BBB_I2C_CON;
  unsigned short BBB_I2C_OA;
  unsigned short BBB_I2C_SA;
  unsigned short BBB_I2C_PSC;
  unsigned short BBB_I2C_SCLL;
  unsigned short BBB_I2C_SCLH;
  unsigned short BBB_I2C_SYSTEST;
  unsigned short BBB_I2C_BUFSTAT;
  unsigned short BBB_I2C_OA1;
  unsigned short BBB_I2C_OA2;
  unsigned short BBB_I2C_OA3;
  unsigned short BBB_I2C_ACTOA;
  unsigned short BBB_I2C_SBLOCK;
} beagle_i2c_regs;

/*register definitions*/
static beagle_i2c_regs am335x_i2c_regs = {
  .BBB_I2C_REVNB_LO = AM335X_I2C_REVNB_LO,
  .BBB_I2C_REVNB_HI = AM335X_I2C_REVNB_HI,
  .BBB_I2C_SYSC = AM335X_I2C_SYSC,
  .BBB_I2C_IRQSTATUS_RAW = AM335X_I2C_IRQSTATUS_RAW,
  .BBB_I2C_IRQSTATUS = AM335X_I2C_IRQSTATUS,
  .BBB_I2C_IRQENABLE_SET = AM335X_I2C_IRQENABLE_SET,
  .BBB_I2C_IRQENABLE_CLR = AM335X_I2C_IRQENABLE_CLR,
  .BBB_I2C_WE = AM335X_I2C_WE,
  .BBB_I2C_DMARXENABLE_SET = AM335X_I2C_DMARXENABLE_SET,
  .BBB_I2C_DMATXENABLE_SET = AM335X_I2C_DMATXENABLE_SET,
  .BBB_I2C_DMARXENABLE_CLR = AM335X_I2C_DMARXENABLE_CLR,
  .BBB_I2C_DMATXENABLE_CLR = AM335X_I2C_DMATXENABLE_CLR,
  .BBB_I2C_DMARXWAKE_EN = AM335X_I2C_DMARXWAKE_EN,
  .BBB_I2C_DMATXWAKE_EN = AM335X_I2C_DMATXWAKE_EN,
  .BBB_I2C_SYSS = AM335X_I2C_SYSS,
  .BBB_I2C_BUF = AM335X_I2C_BUF,
  .BBB_I2C_CNT = AM335X_I2C_CNT,
  .BBB_I2C_DATA = AM335X_I2C_DATA,
  .BBB_I2C_CON = AM335X_I2C_CON,
  .BBB_I2C_OA = AM335X_I2C_OA,
  .BBB_I2C_SA = AM335X_I2C_SA,
  .BBB_I2C_PSC = AM335X_I2C_PSC,
  .BBB_I2C_SCLL = AM335X_I2C_SCLL,
  .BBB_I2C_SCLH = AM335X_I2C_SCLH,
  .BBB_I2C_SYSTEST = AM335X_I2C_SYSTEST,
  .BBB_I2C_BUFSTAT = AM335X_I2C_BUFSTAT,
  .BBB_I2C_OA1 = AM335X_I2C_OA1,
  .BBB_I2C_OA2 = AM335X_I2C_OA2,
  .BBB_I2C_OA3 = AM335X_I2C_OA3,
  .BBB_I2C_ACTOA = AM335X_I2C_ACTOA,
  .BBB_I2C_SBLOCK = AM335X_I2C_SBLOCK
}

static inline uint32_t get_reg_addr(uint32_t offset)
{
  return (bbb_i2c_bus->i2c_base_regs + offset);
}


int am335x_i2c_set_clock(i2c_bus *base)
{
bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
volatile beagle_i2c_regs *regs = bus->regs;
 
prescaler = (BBB_I2C_SYSCLK / BBB_I2C_INTERNAL_CLK) -1;
mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_PSC),prescaler);
divider = BBB_I2C_INTERNAL_CLK/(2*BBB_I2C_SPEED_CLK);
mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_SCLL),divider - 7);
mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_CON),divider - 5);
return 0;
}

void am335x_i2c_destroy(i2c_bus *base)
{
  bbb_i2c_bus *bus = (bbb_i2c_bus *) base;
  rtems_status_code sc;
 
  sc = rtems_interrupt_handler_remove(bus->irq, bbb_i2c_interrupt, bus);
  _Assert(sc == RTEMS_SUCCESSFUL);
  (void)sc;

  i2c_bus_destroy_and_free(&bus->base);
}
/*Any suggestions to disable interrupt here*/
void am335x_i2c_reset(bbb_i2c_bus *bus)
{

  /* Disable I2C module at the time of initialization*/
  /*Should I use write32 ?? I guess mmio_clear is correct choice here*/
  mmio_clear(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_CON),AM335X_I2C_CON_I2C_EN); 

  mmio_clear(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_SYSC),AM335X_I2C_SYSC_AUTOIDLE
);

/*
  can I clear all the interrupt here ?
  mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_IRQ_ENABLE_CLR), ??)
*/
}

void am335x_i2c_set_address_size(
  const i2c_msg *msg
)
{
/*can be configured multiple modes here. Need to think about own address modes*/
  if (msg->flags) {/*10-bit mode*/
    mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_CON),(AM335X_I2C_CFG_10BIT_SLAVE_ADDR | I2C_CON_I2C_EN));
  } else { /*7 bit mode*/
    mmio_write(get_reg_addr(bbb_i2c_bus->reg->AM335X_I2C_CON),(AM335X_I2C_CFG_7BIT_SLAVE_ADDR | I2C_CON_I2C_EN));
  }
}

int am335x_i2c_bus_register(
  const char *bus_path,
  uintptr_t register_base,
  uint32_t input_clock,
  rtems_vector_number irq,
  int i2c_bus_number
)
{
  bbb_i2c_bus *bus;
  rtems_status_code sc;
  int err;
 /*check bus number is >0 & <MAX*/
  bus = (bbb_i2c_bus *) i2c_bus_alloc_and_init(sizeof(*bus));
  if (bus == NULL) {
    return -1;
  }
  bus->i2c_base_regs = i2c_base_addrs[i2c_bus_number]; 
  bus->regs = &am335_i2c_regs; // beagle_i2c_regs ?? 
  bus->i2c_bus_id = i2c_bus_number;
  bus->input_clock = input_clock; // Dont require 
  bus->irq = irq;
 
  am335x_i2c_reset(bus);
 
  err = am335x_i2c_set_clock(&bus->base);
  rtems_set_errno_and_return_minus_one(-err);

  }

  sc  = rtems_interrupt_handler_install(
    irq,
    "BBB I2C",
    RTEMS_INTERRUPT_UNIQUE,
    am335x_i2c_interrupt,
    bus
  );
  if (sc != RTEMS_SUCCESSFUL) {
    (*bus->base.destroy)(&bus->base);
 
    rtems_set_errno_and_return_minux_one(EIO);
  }
 
  bus->bus.tranfer = am335x_i2c_transfer;
  bus->base.set_clock = am335x_i2c_set_clock;
  bus->base.destroy = am335x_i2c_destroy;
  return i2c_bus_register(&bus->base, bus_path);
}

