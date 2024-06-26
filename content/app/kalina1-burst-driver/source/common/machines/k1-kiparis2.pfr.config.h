#if (!defined(k1_kiparis_prf_config_hpp)) && defined(k1_burst_common_h)
#define k1_kiparis_prf_config_hpp
#else
#error error of using k1-kiparis2.prf.config.hpp
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
#define motor_INV3PH_NATIVE_RANGE_HI						K1_PWM_MODULO-1
//(K1_PWM_MODULO-K1_ADC_DELAY)
#define motor_INV3PH_PWM_FORCE									0

#define motor_RANGE_VOLTAGE_LO									BURST_SIGNAL_MIN
#define motor_RANGE_VOLTAGE_HI									BURST_SIGNAL_MAX
#define motor_LATERAL_VOLTAGE_RANGE_LO					BURST_SIGNAL_MIN
#define motor_LATERAL_VOLTAGE_RANGE_HI					BURST_SIGNAL_MAX
#define BURST_ADC_CHANNEL_COUNT 3

#define adc_INDEX {0,2,1}

#define adc_SCALE {1,1,1}

#define swt_CONFIG_PWM_MIN 0
#define swt_CONFIG_PWM_MAX (K1_PWM_MODULO-1)
#define swt_CONFIG_SECTOR_OFFSET 1


#if 0
#define motor_CURRENT3PH_DEFORM  	{\
     	26124, -55714 ,-28673, \
     	-56212, 63589, -19930, \
     	30088, -7875, 48603\
}
#endif

#if 0
#define motor_CURRENT3PH_DEFORM  	{\
     	65538, 0 ,0, \
     	0, 65538, 0, \
     	0, 0, 65538\
}
#endif

#define K1_BOARD_TEMPER_SENCE_REFER_VOLT 0.76
#define K1_BOARD_TEMPER_SENCE_REFER_TEMPER 25
#define K1_BOARD_TEMPER_SENCE_grad2mVolt 2.5



#define BURST_PROTECTION_ENABLED 1
#define BURST_PANICS_MASTER_LOST_ENABLED 0
#if BURST_PANICS_MASTER_LOST_ENABLED == 1
#define motor_REF_ALIVE_PERIOD_US 100000
#endif
		
#define BURST_PANICS_BOARD_TEMPER_ENABLED 1
#if BURST_PANICS_BOARD_TEMPER_ENABLED == 1
#define BURST_PANICS_BOARD_TEMPER_OVERHI_PP 70
#define BURST_PANICS_BOARD_TEMPER_HI_PP 60
#define BURST_PANICS_BOARD_TEMPER_LO_PP (-15)
#define BURST_PANICS_BOARD_TEMPER_ULTRALO_PP (-20)
#endif
		
#define K1_BOARD_MVOLT_TO_PP(x) ((burst_signal_t)((x)>>8))
		
#define BURST_PANICS_BOARD_VOLTAGE_ENABLED 0
#if BURST_PANICS_BOARD_VOLTAGE_ENABLED == 1
#define BURST_PANICS_BOARD_VOLTAGE_OVERHI_PP K1_BOARD_MVOLT_TO_PP(65000)
#define BURST_PANICS_BOARD_VOLTAGE_HI_PP K1_BOARD_MVOLT_TO_PP(60000)
#define BURST_PANICS_BOARD_VOLTAGE_LO_PP K1_BOARD_MVOLT_TO_PP(24000)
#define BURST_PANICS_BOARD_VOLTAGE_ULTRALO_PP K1_BOARD_MVOLT_TO_PP(20000)
#endif

//Ipp=200pp -> 2*Upp = 79.2 mV -> I = 2*Upp/20= 3.96F -> Ipp/I = 200/3.96
#define K1_MOTOR_CURRENT_A_TO_PP_GAIN_15 ((burst_long_signal_t)(32767.*200./3.96 + 16384.))
#define K1_MOTOR_CURRENT_PP_TO_A_GAIN_15 ((burst_long_signal_t)(32767.*3.96/200. + 16384.))
