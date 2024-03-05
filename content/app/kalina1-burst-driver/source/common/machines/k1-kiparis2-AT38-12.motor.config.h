#if (!defined(k1_kiparis2_at38_12_config_h)) && defined(k1_burst_common_h)
#define k1_kiparis2_at38_12_config_h
#else
#error error of using k1-kiparis2-AT38-12.motor.config.h
#endif

#define K1_PWM_PERIOD_US 12L
#define K1_PWM_TIMER_PRESC 2

#define c_cross_flt_SHIFT												5
#define c_cross_flt_PRESC_SHIFT									5
#define c_cross_flt_VALUE_SHIFT									5

#define c_lat_flt_SHIFT													5
#define c_lat_flt_PRESC_SHIFT										5
#define c_lat_flt_VALUE_SHIFT										5

#define motor_LATERAL_CURRENT_PI_PROP_GAIN			1000
#define motor_LATERAL_CURRENT_PI_MODEL_GAIN			100
#define motor_LATERAL_CURRENT_PI_DIFF_GAIN			0
#define motor_LATERAL_CURRENT_PI_FORCE_GAIN			0
#define motor_LATERAL_CURRENT_PI_CONTROL_SHIFT	7
#define motor_LATERAL_CURRENT_PI_MODEL_SHIFT		10
#define motor_LATERAL_CURRENT_PI_RAMP						100
#define motor_LATERAL_CURRENT_RANGE_LO					-K1_MOTOR_CURRENT_A_TO_PP(5)
#define motor_LATERAL_CURRENT_RANGE_HI					K1_MOTOR_CURRENT_A_TO_PP(5)

#define motor_CROSS_CURRENT_PI_PROP_GAIN				1000
#define motor_CROSS_CURRENT_PI_MODEL_GAIN				100
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

#define swt_CONFIG_PWM_RAMP32 200

#define swt_CONFIG_GAIN_PROP 150
#define swt_CONFIG_GAIN_MODEL 50000
#define swt_CONFIG_CURRENT_RAMP32 6000000
#define swt_CONFIG_CURRENT_MIN motor_LATERAL_CURRENT_RANGE_LO
#define swt_CONFIG_CURRENT_MAX motor_LATERAL_CURRENT_RANGE_HI


#define BURST_PANICS_ACWC_OVERCURRENT_ENABLED 1
#if BURST_PANICS_ACWC_OVERCURRENT_ENABLED == 1
#define motor_PANICS_ACWC_OWERCURRENT_PP K1_MOTOR_CURRENT_A_TO_PP(6)
#define motor_PANICS_ACWC_OWERPOWER_PP K1_MOTOR_CURRENT_A_TO_PP(4)
#define motor_PANICS_ACWC_OWERPOWER_TM_US 3000000
#define motor_PANICS_ACWC_NORMPOWER_PP K1_MOTOR_CURRENT_A_TO_PP(3.5)
#endif
 
#define  BURST_APP_PMSM_BLDC_ENABLED 1

//#define  hall_EXTRA_MODE hall_extra_mode_regress
