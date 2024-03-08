#ifndef k1_burst_common_h
#define k1_burst_common_h
#include "k1-burst-driver.names.h"

#include K1_PFR_CONFIG
#include K1_MOTOR_CONFIG

#define K1_PWM_MODULO ( K1_CPU_FREQ_HZ* K1_PWM_PERIOD_US / 2000000L  - 1L )
#define K1_ADC_DELAY ( K1_CPU_FREQ_HZ* K1_ADC_CURRENT_SENCE_TIME_US / 4000000L  - 1L )

#define K1_CONTROL_PERIOD_US (K1_PWM_PERIOD_US * K1_PWM_TIMER_PRESC)

#define K1_SERIAL_TYPE_NONE 0
#define K1_SERIAL_TYPE_OV_UART 1
#define K1_SERIAL_TYPE_OV_CAN 2
#define K1_SERIAL_TYPE_OV_USB 3



#ifndef K1_SERIAL_SIZE
#define K1_SERIAL_SIZE 255
#endif

#ifndef TMP423_ENABLED
#define TMP423_ENABLED 0
#endif


#define K1_MOTOR_CURRENT_PP_TO_A(pp) ((burst_signal_t)(((burst_long_signal_t)(K1_MOTOR_CURRENT_PP_TO_A_GAIN_15*pp))>>15))
#define K1_MOTOR_CURRENT_A_TO_PP(a) ((burst_signal_t)(((burst_long_signal_t)(K1_MOTOR_CURRENT_A_TO_PP_GAIN_15*a))>>15))

#define K1_MOTOR_CURRENT_PP_TO_MA(pp) ((burst_signal_t)(((burst_long_signal_t)(K1_MOTOR_CURRENT_PP_TO_MA_GAIN_10*pp))>>10))
#define K1_MOTOR_CURRENT_MA_TO_PP(ma) ((burst_signal_t)(((burst_long_signal_t)(K1_MOTOR_CURRENT_MA_TO_PP_GAIN_10*ma))>>10))

#ifndef K1_BOARD_TEMPER_PP_TO_GRAD
#define K1_BOARD_TEMPER_PP_TO_GRAD( pp )\
(\
	(burst_signal_t)((\
		(int) (K1_BOARD_TEMPER_SENCE_REFER_TEMPER*65536 - K1_BOARD_TEMPER_SENCE_REFER_VOLT*4096*1000*16/K1_BOARD_TEMPER_SENCE_grad2mVolt)\
		+(int) (3.3* 1000*16/K1_BOARD_TEMPER_SENCE_grad2mVolt)*pp +32767\
	)>>16)\
)
#endif
		

#ifndef K1_BOARD_TEMPER_GRAD_TO_PP
#define K1_BOARD_TEMPER_GRAD_TO_PP( gr )\
(\
	(burst_signal_t)((\
		(int)(K1_BOARD_TEMPER_SENCE_REFER_VOLT/3.3*4098*65536  - K1_BOARD_TEMPER_SENCE_REFER_TEMPER*K1_BOARD_TEMPER_SENCE_grad2mVolt/1000/3.3*4098*65536)\
		+ (int)(K1_BOARD_TEMPER_SENCE_grad2mVolt/1000/3.3*4098*65536) * gr\
	)>>16)\
)	
#endif
#endif


#define K1_BOARD_VOLTAGE_PP_TO_MVOLT(pp) ((burst_signal_t)(((burst_long_signal_t)(K1_BOARD_VOLTAGE_PP_TO_MVOLT_GAIN_10*pp+K1_BOARD_VOLTAGE_PP_TO_MVOLT_OFFSET_10))>>10))
#define K1_BOARD_VOLTAGE_MVOLT_TO_PP(ma) ((burst_signal_t)(((burst_long_signal_t)(K1_BOARD_VOLTAGE_MVOLT_TO_PP_GAIN_10*ma+K1_BOARD_VOLTAGE_MVOLT_TO_PP_OFFSET_10))>>10))

#define K1_BOARD_CURRENT_PP_TO_MAMPER(pp) ((burst_signal_t)(((burst_long_signal_t)(K1_BOARD_CURRENT_PP_TO_MAMPER_GAIN_10*pp+K1_BOARD_CURRENT_PP_TO_MAMPER_OFFSET_10))>>10))
#define K1_BOARD_CURRENT_MAMPER_TO_PP(ma) ((burst_signal_t)(((burst_long_signal_t)(K1_BOARD_CURRENT_MAMPER_TO_PP_GAIN_10*ma+K1_BOARD_CURRENT_MAMPER_TO_PP_OFFSET_10))>>10))

#ifndef K1_BOARD_SERIAL_TYPE
#define K1_BOARD_SERIAL_TYPE_NONE 0
#endif
		
#if K1_BOARD_SERIAL_TYPE==K1_BOARD_SERIAL_TYPE_NONE
#define K1_BOARD_SERIAL_ENABLED 0
#else
#define K1_BOARD_SERIAL_ENABLED 1
#endif

#ifndef K1_BOARD_COMMON_VER
#define K1_BOARD_COMMON_VER 0
#endif
