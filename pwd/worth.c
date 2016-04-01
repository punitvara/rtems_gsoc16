 
#define HWREG(x)                                                              \
        (*((volatile unsigned int *)(x)))
#define HWREGH(x)                                                             \
        (*((volatile unsigned short *)(x)))
#define HWREGB(x)                                                             \
        (*((volatile unsigned char *)(x)))
#define HWREGBITW(x, b)                                                       \
        HWREG(((unsigned int)(x) & 0xF0000000) | 0x02000000 |                \
              (((unsigned int)(x) & 0x000FFFFF) << 5) | ((b) << 2))
#define HWREGBITH(x, b)                                                       \
        HWREGH(((unsigned int)(x) & 0xF0000000) | 0x02000000 |               \
               (((unsigned int)(x) & 0x000FFFFF) << 5) | ((b) << 2))
#define HWREGBITB(x, b)                                                       \
        HWREGB(((unsigned int)(x) & 0xF0000000) | 0x02000000 |               \
               (((unsigned int)(x) & 0x000FFFFF) << 5) | ((b) << 2))

/* EPWM Registers */
#define EHRPWM_TBCTL            (0x0)
#define EHRPWM_TBSTS            (0x2)
#define EHRPWM_TBPHSHR          (0x4)
#define EHRPWM_TBPHS            (0x6)
#define EHRPWM_TBCTR            (0x8)
#define EHRPWM_TBPRD            (0xA)
#define EHRPWM_CMPCTL           (0xE)
#define EHRPWM_CMPAHR           (0x10)
#define EHRPWM_CMPA             (0x12)
#define EHRPWM_CMPB             (0x14)
#define EHRPWM_AQCTLA           (0x16)
#define EHRPWM_AQCTLB           (0x18)
#define EHRPWM_AQSFRC           (0x1A)
#define EHRPWM_AQCSFRC          (0x1C)
#define EHRPWM_DBCTL            (0x1E)
#define EHRPWM_DBRED            (0x20)
#define EHRPWM_DBFED            (0x22)
#define EHRPWM_TZSEL            (0x24)
#define EHRPWM_TZCTL            (0x28)
#define EHRPWM_TZEINT           (0x2A)
#define EHRPWM_TZFLG            (0x2C)
#define EHRPWM_TZCLR            (0x2E)
#define EHRPWM_TZFRC            (0x30)
#define EHRPWM_ETSEL            (0x32)
#define EHRPWM_ETPS             (0x34)
#define EHRPWM_ETFLG            (0x36)
#define EHRPWM_ETCLR            (0x38)
#define EHRPWM_ETFRC            (0x3A)
#define EHRPWM_PCCTL            (0x3C)
#define EHRPWM_HRCNFG           (0xC0)


#define BBBIO_PWMSS_COUNT       3
#define BBBIO_PWMSS0    0
#define BBBIO_PWMSS1    1
#define BBBIO_PWMSS2    2
volatile unsigned int *epwm_ptr[3]      ={NULL, NULL, NULL} ;

void init(int subSystemNumber,float Frequency,bool UseDeadBand,unsigned int DeadBandTime);
{
/*
on hold
*/

}
void PWMSubSystemEnable(int subSytemNumber,bool EnableOrDisable);
{
volatile unsigned short *reg16;

if (EnableOrDisable = 1)
{

reg16 = (void*)epwm_ptr[subSystemNumber] + EPWM_AQCTLA;
*reg16 = 0x2 | ( 0x3 << 4) ;    

/*
---------------------------------------------------------------------------------------------------------------------------------
|	0	|	0	|	0	|	0	|	0	|	0	|	0	|	0	|
---------------------------------------------------------------------------------------------------------------------------------
|	0	|	0	|	1	|	1	|	0	|	0	|	1	|	0	|
---------------------------------------------------------------------------------------------------------------------------------
AQCTA register
---------------------------------------------------------------------------------------------------------------------------------
|				Reserved			|		CBD		|		CBU		|
---------------------------------------------------------------------------------------------------------------------------------
|		CAD		|		CAU		|		PRD		|		ZRO		|
---------------------------------------------------------------------------------------------------------------------------------
*/
/* Action when the counter equals the period. = Clear - force EPWMxA output low. 
 * Action when the counter equals the active CMPA register and the counter is incrementing. = set - force EPWMxA output high.
 * Action when the counter equals the active CMPA register and the counter is decrementing.  = Clear - force EPWMxA output low. 
 * Action when counter equals zero. = 0h (R/W) = Do nothing (action disabled) 
 */
reg16 = (void *)epwm_ptr[subSystemNumber] + EPWM_AQCTLB;
*reg16 = 0x2 | (0x3 << 8);
/*
---------------------------------------------------------------------------------------------------------------------------------
|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	1	|
---------------------------------------------------------------------------------------------------------------------------------
|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	0	|
---------------------------------------------------------------------------------------------------------------------------------
AQCTB register
---------------------------------------------------------------------------------------------------------------------------------
|		Reserved					|		CBD		|		CBU		|
---------------------------------------------------------------------------------------------------------------------------------
|		CAD		|		CAU		|		PRD		|		ZRO		|
--------------------------------------------------------------------------------------------------------------------------------
* Action when counter equals zero. = 0h (R/W) = Do nothing (action disabled) 
* Action when the counter equals the period. = Clear - force EPWMxA output low.
* Action when the counter equals the active CMPA register and the
counter is incrementing. = 0h Do nothing
* Action when the counter equals the active CMPA register and the counter is decrementing. = Do nothing 
* Action when the counter equals the active CMPB register and the
counter is incrementing. = 3h (R/W) = Toggle EPWMxB output - low output signal will be forced
high, and a high signal will be forced low. */

reg16 = (void *)epwm_ptr[subSystemNumber] + EPWM_TBCNT;
*reg16 = 0;

reg16 = (void *)epwm_ptr[subSystemNumber] + EPWM_TBCTL;
*reg16 &= ~0x3;

/*
---------------------------------------------------------------------------------------------------------------------------------
|	1	|	1	|	1	|	1	|	1	|	1	|	1	|	1	|
---------------------------------------------------------------------------------------------------------------------------------
|	1	|	1	|	1	|	1	|	1	|	1	|	0	|	0	|
---------------------------------------------------------------------------------------------------------------------------------

---------------------------------------------------------------------------------------------------------------------------------
|		FREE_SOFT	|	PHSDIR	|			CLKDIV			|		HSPCLKDIV	|
---------------------------------------------------------------------------------------------------------------------------------
|HSPCLKDIV	|SWFSYNC	|         SYNCOSEL		|	PRDLD	|	PHSEN	|		CTRLMODE	|
---------------------------------------------------------------------------------------------------------------------------------

* Counter Mode. = 0h (R/W) = Up-count mode
* Counter Register Load From Phase Register Enable = 1h (R/W) = Load the time-base counter with the phase register when
an EPWMxSYNCI input signal occurs or when a software synchronization is forced by the SWFSYNC bit.
* Active Period Register Load From Shadow Register Select = 1h (R/W) = Load the TBPRD register immediately without using a
shadow register. A write or read to the TBPRD register directly accesses the active register.
* Synchronization Output Select. = 3h (R/W) = Disable EPWMxSYNCO signal
* Software Forced Synchronization Pulse. = 1h (R/W) = Writing a 1 forces a one-time synchronization pulse to be generated. This event is ORed with the EPWMxSYNCI input of the ePWM module. SWFSYNC is valid (operates) only when EPWMxSYNCI is selected by SYNCOSEL = 00.
* High-Speed Time-base Clock Prescale Bits. = 3h (R/W) = /6 
* Time-base Clock Prescale Bits. = 3h (R/W) = /8 
* Phase Direction Bit. = 1h (R/W) = Count up after the synchronization event
* Emulation Mode Bits. = 3h (R/W) = Free run 
*/
}
else 
{
 	reg16=(void *)epwm_ptr[subSystemNumber] + EPWM_TBCTL;
 	*reg16 |= 0x3;
/*

Counter Mode. = 3h (R/W) = Stop-freeze counter operation (default on reset)

*/

        reg16=(void*)epwm_ptr[subSystemNumber] +EPWM_AQCTLA;
        *reg16 = 0x1 | ( 0x3 << 4) ;
/*

Action when counter equals zero. = 1h (R/W) = Clear - force EPWMxA output low.
Action when the counter equals the active CMPA register and the
counter is incrementing. = 2h (R/W) = Set - force EPWMxA output high.
Action when the counter equals the active CMPA register and the
counter is decrementing. = 1h (R/W) = Clear - force EPWMxA output low.
*/
        reg16=(void*)epwm_ptr[subSystemNumber] +EPWM_AQCTLB;
        *reg16 = 0x1 | ( 0x3 << 8) ;
/*
Action when counter equals zero. = 1h (R/W) = Clear - force EPWMxB output low.
Action when the counter equals the active CMPB register and the
counter is incrementing. = 2h (R/W) = Set - force EPWMxB output high.
Action when the counter equals the active CMPB register and the
counter is decrementing. = 1h (R/W) = Clear - force EPWMxB output low.

*/
        reg16 = (void *)epwm_ptr[subSystemNumber] + EPWM_TBCNT;
        *reg16 = 0;
/*
0h = Reading these bits gives the current time-base counter value.
Writing to these bits sets the current time-base counter value.
The update happens as soon as the write occurs.
The write is NOT synchronized to the time-base clock (TBCLK) and
the register is not shadowed.
*/

}

}
/* 

*/
rtems_status_code SetDutyCycle(int subSystemNumber,float HZ, float DutyCycleA, float DutyCycleB);
{
DutyCycleA /= 100.0f;
DutyCycleB /= 100.0f;

float cyclens = 0.0f;
float divisor = 0;
const float CLKDIV_div[] = {1.0 ,2.0 ,4.0 ,8.0 ,16.0 ,32.0 , 64.0 , 128.0};
const float HSPCLKDIV_div[] ={1.0 ,2.0 ,4.0 ,6.0 ,8.0 ,10.0 , 12.0 , 14.0};
int NearTBPRD = 0;
int NearCLKDIV =7;
int NearHSPCLKDIV =7;

cyclens = 10000000000.0f / HZ; 

divisor = (cyclens / 655350.0f );
if(divisor > (128 * 14)) {
printf("Frequency can't generate ");
}
 else {
                /* using Exhaustive Attack metho */
                for(i = 0 ; i < 8 ; i ++) {
                        for(j = 0 ; j < 8 ; j ++) {
                                if((CLKDIV_div[i] * HSPCLKDIV_div[j]) < (CLKDIV_div[NearCLKDIV] * HSPCLKDIV_div[NearHSPCLKDIV]) &&
                                  ((CLKDIV_div[i] * HSPCLKDIV_div[j]) > Divisor)) {
                                        NearCLKDIV = i ;
                                        NearHSPCLKDIV = j ;
                                }
                        }
                }
}

 NearTBPRD = (Cyclens / (10.0 *CLKDIV_div[NearCLKDIV] *HSPCLKDIV_div[NearHSPCLKDIV])) ;
 /* setting clock diver and freeze time base */
                reg16=(void*)epwm_ptr[subSystemNumber] +EPWM_TBCTL;
                *reg16 = TBCTL_CTRMODE_FREEZE | (NearCLKDIV << 10) | (NearHSPCLKDIV << 7);

                /*  setting dutycycle A and dutycycle B */
                reg16=(void*)epwm_ptr[subSystemNumber] +EPWM_CMPB;
                *reg16 =(unsigned short)((float)NearTBPRD * dutyB);

                reg16=(void*)epwm_ptr[subSystemNumber] +EPWM_CMPA;
                *reg16 =(unsigned short)((float)NearTBPRD * dutyA);

                reg16=(void*)epwm_ptr[subSystemNumber] +EPWM_TBPRD;
                *reg16 =(unsigned short)NearTBPRD;

                /* reset time base counter */
                reg16 = (void *)epwm_ptr[subSystemNumber] + EPWM_TBCNT;
                *reg16 = 0;
        }
        return RTEMS_SUCCESSFUL;
}


