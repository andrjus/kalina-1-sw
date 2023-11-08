#if (!defined(k1_ecmax_motor_config_h)) && defined(k1_burst_common_h)
#define fb3_ecmax_motor_config_h
#else
#error error of using k1-ecmax.motor.config.h
#endif

#define K1_PWM_PERIOD_US 50L
#define K1_PWM_TIMER_PRESC 1

#define c_cross_flt_SHIFT												2
#define c_cross_flt_PRESC_SHIFT									2
#define c_cross_flt_VALUE_SHIFT									2

#define c_lat_flt_SHIFT													2
#define c_lat_flt_PRESC_SHIFT										2
#define c_lat_flt_VALUE_SHIFT										2

#define motor_LATERAL_CURRENT_PI_PROP_GAIN			700
#define motor_LATERAL_CURRENT_PI_MODEL_GAIN			150
#define motor_LATERAL_CURRENT_PI_DIFF_GAIN			0
#define motor_LATERAL_CURRENT_PI_FORCE_GAIN			0
#define motor_LATERAL_CURRENT_PI_CONTROL_SHIFT	7
#define motor_LATERAL_CURRENT_PI_MODEL_SHIFT		10
#define motor_LATERAL_CURRENT_PI_RAMP						100
#define motor_LATERAL_CURRENT_RANGE_LO					-500
#define motor_LATERAL_CURRENT_RANGE_HI					500

#define motor_CROSS_CURRENT_PI_PROP_GAIN				700
#define motor_CROSS_CURRENT_PI_MODEL_GAIN				150
#define motor_CROSS_CURRENT_PI_DIFF_GAIN				0
#define motor_CROSS_CURRENT_PI_FORCE_GAIN				0
#define motor_CROSS_CURRENT_PI_CONTROL_SHIFT		7
#define motor_CROSS_CURRENT_PI_MODEL_SHIFT			10
#define motor_CROSS_CURRENT_PI_RAMP 						1000
#define motor_CROSS_CURRENT_RANGE_LO 						BURST_SIGNAL_MIN
#define motor_CROSS_CURRENT_RANGE_HI 						BURST_SIGNAL_MAX

#define hall_INV burst_false //burst_true
#define hall_OFFSET_NATIVE 0// (-BURST_SIGNAL_T(75./180))

#define PMSM_HALL_APP_EXTRA_TYPE PMSM_HALL_APP_EXTRA_TYPE_NONE
#define motor_CONTROL_PRESC 1

#define swt_CONFIG_GAIN_PROP 1
#define swt_CONFIG_GAIN_MODEL 1
#define swt_CONFIG_PWM_RAMP32 1
#define swt_CONFIG_PWM_MIN 0
#define swt_CONFIG_PWM_MAX K1_PWM_MODULO-1
#define swt_CONFIG_SECTOR_OFFSET 0

//Ipp=100pp -> 2*Upp = 178.0 mV -> I = 2*Upp/20= 8.9A -> Ipp/I = 100/8.9
#define K1_MOTOR_CURRENT_A_TO_PP_GAIN_15 ((burst_long_signal_t)(32767.*100./8.9 + 16384.))
#define K1_MOTOR_CURRENT_PP_TO_A_GAIN_15 ((burst_long_signal_t)(32767.*8.9/100. + 16384.))

#define BURST_PANICS_ACWC_OVERCURRENT_ENABLED 1
#if BURST_PANICS_ACWC_OVERCURRENT_ENABLED == 1
#define motor_PANICS_ACWC_OWERCURRENT_PP FB3_MOTOR_CURRENT_A_TO_PP(15)
#define motor_PANICS_ACWC_OWERPOWER_PP FB3_MOTOR_CURRENT_A_TO_PP(10)
#define motor_PANICS_ACWC_OWERPOWER_TM_US 3000000
#endif
