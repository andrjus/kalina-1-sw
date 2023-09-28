#if (!defined(fb3_burst_prf_config_hpp)) && defined(fb3_burst_common_h)
#define fb3_burst_prf_config_hpp
#else
#error error of using k1-nh.prf.config.hpp
#endif

#define K1_CPU_FREQ_HZ 168000000LL
#define K1_ADC_CURRENT_SENCE_TIME_US 6
#define K1_DEAD_TIME 64
//#define K1_CURRENT_SENCE_ENABLED 0
//#define K1_ROTOR_ENCO_TYPE K1_ROTOR_ENCO_TYPE_NONE
//#define K1_ROTOR_POSITION_SENCE_TYPE K1_ROTOR_POSITION_SENCE_TYPE_NONE
//#define K1_SERIAL_TYPE K1_SERIAL_TYPE_NONE

//#define adc_INDEX 

#define FMSTR_REC_TIMEBASE (32768+K1_CONTROL_PERIOD_US)
#define FMSTR_REC_BUFF_SIZE 4000

#define motor_INV3PH_NATIVE_RANGE_LO						0
#define motor_INV3PH_NATIVE_RANGE_HI						(K1_PWM_MODULO-K1_ADC_DELAY)
#define motor_INV3PH_PWM_FORCE									600

#define motor_RANGE_VOLTAGE_LO									BURST_SIGNAL_MIN
#define motor_RANGE_VOLTAGE_HI									BURST_SIGNAL_MAX
#define motor_LATERAL_VOLTAGE_RANGE_LO					BURST_SIGNAL_MIN
#define motor_LATERAL_VOLTAGE_RANGE_HI					BURST_SIGNAL_MAX
#define BURST_ADC_CHANNEL_COUNT 3
#define adc_INDEX {0,1,2}
#define adc_SCALE {-1,-1,-1}

//#define motor_CURRENT3PH_DEFORM { 42234,   -36409,   -5825, -29127,    36409,   -7282,   -11651,   -18204,    29855}

