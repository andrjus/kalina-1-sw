#ifndef fb3_burst_common_h
#define fb3_burst_common_h
#include "k1-burst-driver.names.h"
#include K1_PFR_CONFIG
#endif

#define K1PWM_MODULO ( K1CPU_FREQ_HZ* K1PWM_PERIOD_US / K1PWM_TIMER_PRESC / 2000000L  - 1L )
#define K1ADC_DELAY ( K1CPU_FREQ_HZ* K1ADC_CURRENT_SENCE_TIME_US / K1PWM_TIMER_PRESC / 4000000L  - 1L )

