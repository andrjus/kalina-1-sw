#ifndef k1_burst_common_h
#define k1_burst_common_h
#include "k1-burst-driver.names.h"

#include K1_PFR_CONFIG
#include K1_MOTOR_CONFIG

#define K1_PWM_MODULO ( K1_CPU_FREQ_HZ* K1_PWM_PERIOD_US / 2000000L  - 1L )
#define K1_ADC_DELAY ( K1_CPU_FREQ_HZ* K1_ADC_CURRENT_SENCE_TIME_US / 4000000L  - 1L )

#define K1_CONTROL_PERIOD_US (K1_PWM_PERIOD_US * K1_PWM_TIMER_PRESC)

#ifndef K1_SERIAL_SIZE
#define K1_SERIAL_SIZE 255
#endif

#ifndef TMP423_ENABLED
#define TMP423_ENABLED 0
#endif


#define K1_MOTOR_CURRENT_PP_TO_A(pp) ((burst_signal_t)(((burst_long_signal_t)(K1_MOTOR_CURRENT_PP_TO_A_GAIN_15*pp))>>15))
#define K1_MOTOR_CURRENT_A_TO_PP(ma) ((burst_signal_t)(((burst_long_signal_t)(K1_MOTOR_CURRENT_A_TO_PP_GAIN_15*ma))>>15))

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




