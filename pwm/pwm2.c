#include <libcpu/am335x.h>
#include <stdio.h>
#include <bsp.h>
#include <bsp/gpio.h>
#include <bsp/bbb-gpio.h>

#define nitems(x)       (sizeof((x)) / sizeof((x)[0]))
#define PWMREG(pwm, reg) REG((pwm)->baseaddr + (reg))
#define PWMREG16(pwm, reg) REG16((pwm)->baseaddr + (reg))
#define	DEFAULT_PWM_PERIOD	1000
#define	PWM_CLOCK		100000000UL
 
const unsigned int PWMSS_AddressOffset[]={PWMSS0_MMAP_ADDR,
                                          PWMSS1_MMAP_ADDR,
                                          PWMSS2_MMAP_ADDR};
static int am335x_ehrpwm_clkdiv[8] = { 1, 2, 4, 8, 16, 32, 64, 128 };

struct pwmss {
	volatile int *baseaddr;
	unsigned int pwm_clkdiv;
	unsigned int pwm_period;
	unsigned int pwm_dutyA;
	unsigned int pwm_dutyB;
	unsigned int pwm_freq;
	/* more state fields go here */
}; 

struct pwmss pwmss0, pwmss1, pwmss2;	/* uninitialized */

struct pwmss *pwmss_lookup(int id)
{
	switch(id) {
		case BBBIO_PWMSS0:
			return &pwmss0;
		case BBBIO_PWMSS1:
			return &pwmss1;
		case BBBIO_PWMSS2:
			return &pwmss2;
		default:
			return NULL;
	}
}

/**
 * \brief   This API enables the particular PWM module.
 *
 * \param   baseAddr    Base Address of the PWM Module Registers.
 *
 * \return  None
 *
 **/

void pwmss_enable(int id)
{
  struct pwmss *pwm = pwmss_lookup(id);
  if(!pwm) { return -RTEMS_INVALID_ID; }

  PWMREG16(pwm, EPWM_AQCTLA) = 0x2 | (0x3 << 4);
  PWMREG16(pwm, EPWM_AQCTLB) = 0x2 | (0x3 << 8);
  PWMREG16(pwm, EPWM_TBCNT) = 0;
  PWMREG16(pwm, EPWM_TBCTL) = ~0x3;
}

/**
 * \brief   This API disables the HR sub-module.
 *
 * \param   baseAddr    Base Address of the PWM Module Registers.
 *
 * \return  None
 *
 **/

void pwmss_disable(int id)
{
  struct pwmss *pwm = pwmss_lookup(id);
  if(!pwm) { return -RTEMS_INVALID_ID; }
  
  PWMREG16(pwm, EPWM_TBCTL) = 0x3;
  PWMREG16(pwm, EPWM_AQCTLA) = 0x1 | ( 0x3 << 4 );
  PWMREG16(pwm, EPWM_AQCTLB) = 0x1 | ( 0x3 << 8 );
  PWMREG16(pwm, EPWM_TBCNT)  = 0;
}

pwmss_init(volatile int *baseAddr, int id)
{
	struct pwmss *pwm = pwmss_lookup(id);
	assert(pwm);
	pwm->baseaddr = baseAddr;
	pwmss_disable(baseaddr);
}

pwm_init()
{
	pwmss_init(PWMSS0_MMAP_ADDR,BBBIO_PWMSS0);
	pwmss_init(PWMSS1_MMAP_ADDR,BBBIO_PWMSS1);
	pwmss_init(PWMSS2_MMAP_ADDR,BBBIO_PWMSS2);
}

/* ----------------------------------------------------------------------------------------------- */
/* PWMSS Timebase clock check
 *      check the timenase clock enable or not
 *
 *      @param PWMSS_ID :  PWM sumsystem ID (BBBIO_PWMSS0 ,BBBIO_PWMSS1, BBBIO_PWMSS2)
 *
 *      @return : 0 for disable timebase clock , 1 for enable for timebase clock
 */

int pwmss_tb_clock_check(unsigned int PWMSS_ID)
{
        unsigned int reg_value ;
	struct pwmss *pwm = pwmss_lookup(PWMSS_ID);
	if(!pwm) { return -RTEMS_INVALID_ID; }
        /* Control module check */
	reg_value = PWMREG(pwm, BBBIO_PWMSS_CTRL);

        return !!(reg_value & (1 << PWMSS_ID)) ;
}
 
/**
 * \brief   This function Enables pinmuxing for PWM module.
 *          
 *
 * \param   instance  It is the instance number of EPWM of pwmsubsystem.
 *
 **/

void EPWMPinMuxSetup(void)
{
 
  REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(9)) = BBB_MUXMODE(4);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(8)) = BBB_MUXMODE(4);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(0)) = BBB_MUXMODE(3);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(1)) = BBB_MUXMODE(3);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(11)) = BBB_MUXMODE(2);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_LCD_DATA(10)) = BBB_MUXMODE(2);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(2)) = BBB_MUXMODE(6);

  REG(AM335X_PADCONF_BASE + CONTROL_CONF_GPMC_AD(3)) = BBB_MUXMODE(6);

  REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_D0) = BBB_MUXMODE(3);

  REG(AM335X_PADCONF_BASE + AM335X_CONF_SPI0_SCLK) = BBB_MUXMODE(3);

  REG(AM335X_PADCONF_BASE + AM335X_CONF_MCASP0_FSX) = BBB_MUXMODE(1);

  REG(AM335X_PADCONF_BASE + AM335X_CONF_MCASP0_ACLKX) = BBB_MUXMODE(1);
}
/*
int pwmss_freq(struct pwmss *pwm)
{
  int clkdiv;
  clkdiv =am335x_ehrpwm_clkdiv[pwm->pwm_clkdiv];
  pwm->pwm_freq = PWM_CLOCK / clkdiv / pwm->pwm_period;

int pwmss_freq(float frequency,int id )
{
  int clkdiv,i,period;
	struct pwmss *pwm = pwmss_lookup(id);
	pwm->pwm_freq = frequency;
  if (pwm->pwm_freq > PWM_CLOCK)
	pwm->pwm_freq = PWM_CLOCK;
  for(i = nitems(am335x_ehrpwm_clkdiv) - 1; i >= 0; i--) {
	clkdiv = am335x_ehrpwm_clkdiv[i];
	period = PWM_CLOCK / clkdiv / freq;
	if(period >65535) // USHRT_MAX
	break;
	pwm->pwm_clkdiv = i;
	pwm->pwm_period = period;
   }
   if(PWM_COUNT_UP_DOWN == countDir) 
   {
	PWMREG(pwm,PWM_TBPRD) = pwm->pwm_period /2;
   }
   else 
   {
	PWMREG(pwm,PWM_TBPRD) = pwm->pwm_period;
   }
}	
*/

/* PWMSS setting
 *      set pulse argument of epwm module
 *
 *      @param PWMID    : EPWMSS number , 0~2
 *      @param HZ       : pulse HZ
 *      @param dutyA    : Duty Cycle in ePWM A
 *      @param dutyB    : Duty Cycle in ePWM B
 *
 *      @return         : 1 for success , 0 for failed
 *
 *      @example        :  PWMSS_Setting(0 , 50.0f , 50.0f , 25.0f);      // Generate 50HZ pwm in PWM0 ,
 *                                                                              // duty cycle is 50% for ePWM0A , 25% for ePWM0B
 *
 *      @Note :
 *              find an number nearst 65535 for TBPRD , to improve duty precision,
 *
 *              Using big TBPRD can increase the range of CMPA and CMPB ,
 *              and it means we can get better precision on duty cycle.
 *
 *              EX : 20.25% duty cycle
 *                  on TBPRD = 62500 , CMPA = 12656.25 ( .25 rejection) , real duty : 20.2496% (12656 /62500)
 *                  on TBPRD = 6250  , CMPA = 1265.625 ( .625 rejection), real duty : 20.24%   (1265 6250)
 *                  on TBPRD = 500   , CMPA = 101.25   ( .25 rejection) , real duty : 20.2%    (101/500)
 *
 *              Divisor = CLKDIV * HSPCLKDIV
 *                      1 TBPRD : 10 ns (default)
 *                      65535 TBPRD : 655350 ns
 *                      65535 TBPRD : 655350 * Divisor ns  = X TBPRD : Cyclens
 *
 *              according to that , we must find a Divisor value , let X nearest 65535 .
 *              so , Divisor must  Nearest Cyclens/655350
 */

int PWMSS_Setting(int id, float HZ, float dutyA, float dutyB)
{
	struct pwmss *pwm = pwmss_lookpup(id);
	pwm->pwm_freq = HZ;
	pwm->pwm_dutyA = dutyA;
	pwm->pwm_dutyB = dutyB;
        int param_error =1;
        if(HZ < 0)
                param_error =0;
        if(dutyA < 0.0f || dutyA > 100.0f || dutyB < 0.0f || dutyB > 100.0f)
                param_error = 0;
        if(param_error == 0) {
                printf("ERROR in parameter \n");
        }
        dutyA /= 100.0f;
        dutyB /= 100.0f;

        /*Compute necessary TBPRD*/
        float Cyclens = 0.0f;
        float Divisor =0;
        int i,j;
        const float CLKDIV_div[] = {1.0,2.0,4.0,8.0,16.0,32.0,64.0,128.0};
        const float HSPCLKDIV_div[] = {1.0, 2.0, 4.0, 6.0, 8.0, 10.0,12.0, 14.0};
        int NearCLKDIV =7;
        int NearHSPCLKDIV =7;
        int NearTBPRD =0;

        Cyclens = 1000000000.0f / HZ; /** 10^9 /Hz compute time per cycle (ns)
                                       */
        Divisor = (Cyclens / 655350.0f);  /** am335x provide (128* 14) divider,
                                           *  and per TBPRD means 10ns when divider 
                                           *  and max TBPRD is 65535 so max cycle 
                                           *  is 128 * 8 * 14 * 65535 * 10ns
                                           */
        if(Divisor > (128 * 14)) {
                printf("Can't generate %f HZ",HZ);
                return 0;
        }
	else {
                for (i=0;i<8;i++) {
                        for(j=0 ; j<8; j++) {
                                if((CLKDIV_div[i] * HSPCLKDIV_div[j]) < (CLKDIV_div[NearCLKDIV]
                                                        * HSPCLKDIV_div[NearHSPCLKDIV]) && (CLKDIV_div[i] * HSPCLKDIV_div[j] > Divisor)) {
                                        NearCLKDIV = i;
                                        NearHSPCLKDIV = j;
                                }
                        }
                }
		
                NearTBPRD = (Cyclens / (10.0 * CLKDIV_div[NearCLKDIV] * HSPCLKDIV_div[NearHSPCLKDIV]));
                /*setting clock divider and freeze time base*/
                PWMREG16(pwm, EPWM_TBCTL) = TBCTL_CTRMODE_FREEZE | (NearCLKDIV << 10) | (NearHSPCLKDIV << 7);
                PWMREG16(pwm, EPWM_CMPB) = (unsigned short)((float)(NearTBPRD) * dutyB);
                PWMREG16(pwm, EPWM_CMPA) = (unsigned short)((float)(NearTBPRD) * dutyA);
                PWMREG16(pwm, EPWM_TBPRD) = (unsigned short)NearTBPRD;
                PWMREG16(pwm, EPWM_TBCNT) = 0;
                printf("\nfinished setting \n");
        }
        return 1;
}

/* ----------------------------------------------------------------------------------------------- */
/* PWM subsystem system control
 *      enable or disable module clock
 *
 *      @param PWMSS_ID :  PWM sumsystem ID (BBBIO_PWMSS0 ,BBBIO_PWMSS1, BBBIO_PWMSS2).
 *      @param enable : 0 for disable , else for enable .
 *
 *      @return : 1 for success ,  0 for error
 */
int PWMSS_module_ctrl(unsigned int id, int enable)
{
        unsigned int module_set[] = {BBBIO_PWMSS0, BBBIO_PWMSS1, BBBIO_PWMSS2};
        unsigned int module_clk_set[] = {BBBIO_CM_PER_EPWMSS0_CLKCTRL, BBBIO_CM_PER_EPWMSS1_CLKCTRL, BBBIO_CM_PER_EPWMSS2_CLKCTRL};
        int ret = 1;
	struct pwmss *pwm = pwmss_lookup(id);

        if(enable) {
                if(PWMSS_TB_clock_check(module_set[id])) {
                        /* Enable module clock */
       //                 *reg = 0x2;     /* Module enable and fully functional */
                        REG(CM_PER_ADDR + module_clk_set[id]) = 0x2;
			return ret;
                }
                else {
                        printf("PWMSS_module_ctrl : PWMSS-%d timebase clock disable in Control Module\n", id);
                }
                ret = 0 ;
        }
        REG(CM_PER_ADDR + module_clk_set[id]) = 0x3 << 16;       /* Module is disabled and cannot be accessed */
        return ret;
}

/* ----------------------------------------------------------------------------------------------- */
/* PWMSS status (no effect now)
 *      set pulse argument of epwm module
 *
 *      @param PWMID    : EPWMSS number , 0~3
 *
 *      @return         : 1 for success , 0 for failed
 */
int BBBIO_PWMSS_Status(unsigned int PWMID)
{
unsigned int reg_value ;
struct pwmss *pwm = pwmss_lookup(PWMID);
	reg_value = REG(BBBIO_CONTROL_MODULE + BBBIO_PWMSS_CTRL) >>PWMID & 0x01;
	 if(reg_value == 0) {
                printf("PWMSS [%d] Timebase clock Disable , Control Module [pwmss_ctrl register]\n", PWMID);
        }
        else {
                // reg=(void *)pwmss_ptr[PWMID] + PWMSS_CLKSTATUS;
                reg_value = REG(PWMSS_AddressOffset[PWMID] + PWMSS_CLKSTATUS);

                printf("PWMSS [%d] :\tCLKSTOP_ACK %d , CLK_EN_ACK %d , CLKSTOP_ACK %d , CLK_EN_ACK %d , CLKSTOP_ACK %d , CLK_EN_ACK %d\n",
                        PWMID ,
                        reg_value >>9 & 0x1 ,
                        reg_value >>8 & 0x1 ,
                        reg_value >>5 & 0x1 ,
                        reg_value >>4 & 0x1 ,
                        reg_value >>1 & 0x1 ,
                        reg_value >>0 & 0x1 );
        }
        return 1 ;
}

