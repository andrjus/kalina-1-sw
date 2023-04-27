#if (!defined(fb3_burst_prf_config_hpp)) && defined(fb3_burst_common_h)
#define fb3_burst_prf_config_hpp
#else
#error error of using k1-burst-driver.prf.config.hpp
#endif

#define K1PWM_PERIOD_US 50L
#define K1PWM_TIMER_PRESC 1
#define K1CPU_FREQ_HZ 72000000LL
#define K1ADC_CURRENT_SENCE_TIME_US 8
#define K1ADC_CH_COUNT 6
#define K1CURRENT_SENCE_ENABLED 0
#define K1ROTOR_ENCO_TYPE K1ROTOR_ENCO_TYPE_NONE
#define K1ROTOR_POSITION_SENCE_TYPE K1ROTOR_POSITION_SENCE_TYPE_NONE
#define K1SERIAL_TYPE K1SERIAL_TYPE_NONE
#define BURST_ADC_CHANNEL_COUNT 5
#define FMSTR_REC_TIMEBASE (32768+K1PWM_PERIOD_US)
#define FMSTR_REC_BUFF_SIZE 4000
