#if (!defined(k1_burst_prf_config_hpp)) && defined(k1_burst_common_h)
#define k1_burst_prf_config_hpp
#else
#error error of using k1-nh.prf.config.hpp
#endif

#define K1_CPU_FREQ_HZ 168000000LL
#define K1_ADC_CURRENT_SENCE_TIME_US 0
//#define K1_DEAD_TIME 64
//#define K1_CURRENT_SENCE_ENABLED 0
//#define K1_ROTOR_ENCO_TYPE K1_ROTOR_ENCO_TYPE_NONE
//#define K1_ROTOR_POSITION_SENCE_TYPE K1_ROTOR_POSITION_SENCE_TYPE_NONE
//#define K1_SERIAL_TYPE K1_SERIAL_TYPE_NONE

//#define adc_INDEX 

#define FMSTR_REC_TIMEBASE (32768+K1_CONTROL_PERIOD_US)
#define FMSTR_REC_BUFF_SIZE 4000

#define motor_INV3PH_NATIVE_RANGE_LO						0
#define motor_INV3PH_NATIVE_RANGE_HI						K1_PWM_MODULO-1
#define motor_INV3PH_PWM_FORCE									0

#define motor_RANGE_VOLTAGE_LO									BURST_SIGNAL_MIN
#define motor_RANGE_VOLTAGE_HI									BURST_SIGNAL_MAX
#define motor_LATERAL_VOLTAGE_RANGE_LO					BURST_SIGNAL_MIN
#define motor_LATERAL_VOLTAGE_RANGE_HI					BURST_SIGNAL_MAX
#define BURST_ADC_CHANNEL_COUNT 3
#define adc_INDEX {0,1,2}
#define adc_SCALE {-1,-1,-1}

#define swt_CONFIG_GAIN_PROP 175
#define swt_CONFIG_GAIN_MODEL 3000
#define swt_CONFIG_PWM_RAMP32 2000
#define swt_CONFIG_PWM_MIN 0
#define swt_CONFIG_PWM_MAX K1_PWM_MODULO-1
#define swt_CONFIG_SECTOR_OFFSET 5
#define swt_CONFIG_CURRENT_RAMP32 200
#define swt_CONFIG_CURRENT_MIN 0
#define swt_CONFIG_CURRENT_MAX K1_MOTOR_CURRENT_A_TO_PP(50)

#if 0
#define motor_CURRENT3PH_DEFORM  	{\
     	26124, -55714 ,-28673, \
     	-56212, 63589, -19930, \
     	30088, -7875, 48603\
}
#endif



#define motor_CURRENT3PH_DEFORM  	{\
     	65538, 0 ,0, \
     	0, (burst_long_signal_t)(65538.*172/154*81/66), 0, \
     	0, 0, (burst_long_signal_t)(65538.)\
}


#if 0
#define motor_CURRENT3PH_DEFORM  	{\
     	65538, 0 ,0, \
     	0, 84811, 0, \
     	0, 0, 30424\
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
#define BURST_PANICS_BOARD_TEMPER_OVERHI_PP (50)
#define BURST_PANICS_BOARD_TEMPER_HI_PP (45)
#define BURST_PANICS_BOARD_TEMPER_LO_PP (-15)
#define BURST_PANICS_BOARD_TEMPER_ULTRALO_PP (-20)
#endif



//b = pinv([pp ones(size(pp))])*vv*32767
#define BURST_PANICS_BOARD_RESET_TIMEOUT_PP 0

#define K1_BOARD_VOLTAGE_PP_TO_MVOLT_GAIN_10 22432L
#define K1_BOARD_VOLTAGE_PP_TO_MVOLT_OFFSET_10  ( -282620L)

#define K1_BOARD_VOLTAGE_MVOLT_TO_PP_GAIN_10 47L
#define K1_BOARD_VOLTAGE_MVOLT_TO_PP_OFFSET_10    12902L

#define BURST_PANICS_BOARD_VOLTAGE_ENABLED 0
#if BURST_PANICS_BOARD_VOLTAGE_ENABLED == 1
#define BURST_PANICS_BOARD_VOLTAGE_OVERHI_PP K1_BOARD_VOLTAGE_MVOLT_TO_PP(65000)
#define BURST_PANICS_BOARD_VOLTAGE_HI_PP K1_BOARD_VOLTAGE_MVOLT_TO_PP(60000)
#define BURST_PANICS_BOARD_VOLTAGE_LO_PP K1_BOARD_VOLTAGE_MVOLT_TO_PP(24000)
#define BURST_PANICS_BOARD_VOLTAGE_ULTRALO_PP K1_BOARD_VOLTAGE_MVOLT_TO_PP(20000)
#endif

#define K1_MOTOR_CURRENT_A_TO_PP_GAIN_15 ((burst_long_signal_t)(32767.*15.1657))
#define K1_MOTOR_CURRENT_PP_TO_A_GAIN_15 ((burst_long_signal_t)(32767.*0.0658))


#define motor_INV3PH_DEFORM_ENABLED burst_true
#define motor_INV3PH_DEFORM_LEVEL 4312
#define motor_INV3PH_DEFORM_HI_GAIN_16 151359
#define motor_INV3PH_DEFORM_LO_GAIN_16 52469
#define motor_INV3PH_DEFORM_LO_BEVEL_16 428180201

#define  BURST_APP_PMSM_BLDC_ENABLED 1
