#ifndef fb3_burst_common_h
#define fb3_burst_common_h
#include "k1-burst-driver.names.h"
#include K1_PFR_CONFIG
#endif

#define K1_PWM_MODULO ( K1_CPU_FREQ_HZ* K1_PWM_PERIOD_US / K1_PWM_TIMER_PRESC / 2000000L  - 1L )
#define K1_ADC_DELAY ( K1_CPU_FREQ_HZ* K1_ADC_CURRENT_SENCE_TIME_US / K1_PWM_TIMER_PRESC / 4000000L  - 1L )



