#ifndef LIBBSP_ARM_BEAGLE_BBB_PWM_H
#define LIBBSP_ARM_BEAGLE_BBB_PWM_H


void PWMSSTBClkEnable(unsigned int instance);
void PWMSSModuleClkConfig(unsigned int instanceNum);
void EHRPWMTimebaseClkConfig(unsigned int baseAddr,
                             unsigned int tbClk,
                             unsigned int moduleClk);
void EHRPWMPWMOpFreqSet(unsigned int baseAddr,
                        unsigned int tbClk,
                        unsigned int pwmFreq,
                        unsigned int counterDir,
                        bool enableShadowWrite);
void EHRPWMTimebaseSyncDisable(unsigned int baseAddr);
void EHRPWMSyncOutModeSet(unsigned int baseAddr, unsigned int syncOutMode);
void EHRPWMTBEmulationModeSet(unsigned int baseAddr, unsigned int mode);
bool EHRPWMLoadCMPA(unsigned int baseAddr,
                    unsigned int CMPAVal,
                    bool enableShadowWrite,
                    unsigned int ShadowToActiveLoadTrigger,
                    bool OverwriteShadowFull);
bool EHRPWMLoadCMPB(unsigned int baseAddr,
                    unsigned int CMPBVal,
                    bool enableShadowWrite,
                    unsigned int ShadowToActiveLoadTrigger,
                    bool OverwriteShadowFull);
void EHRPWMConfigureAQActionOnB(unsigned int baseAddr,
                                unsigned int zero,
                                unsigned int period,
                                unsigned int CAUp,
                                unsigned int CADown,
                                unsigned int CBUp,
                                unsigned int CBDown,
                                unsigned int SWForced);
void EHRPWMDBOutput(unsigned int baseAddr, unsigned int DBgenOpMode);
void EHRPWMChopperDisable(unsigned int baseAddr);
void EHRPWMTZTripEventDisable(unsigned int baseAddr, bool osht_CBC);
void EHRPWMETIntPrescale(unsigned int baseAddr, unsigned int prescale);
void EHRPWMETIntSourceSelect(unsigned int baseAddr, unsigned int selectInt);
void EHRPWMHRDisable(unsigned int baseAddr);


#endif
