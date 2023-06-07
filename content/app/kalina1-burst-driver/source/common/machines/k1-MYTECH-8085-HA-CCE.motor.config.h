#if (!defined(fb3_ecmax_motor_config_h)) && defined(fb3_burst_common_h)
#define fb3_ecmax_motor_config_h
#else
#error error of using fb3-ecmax.motor.config.h
#endif

#define K1_PWM_PERIOD_US 50L
#define K1_PWM_TIMER_PRESC 1

#define c_cross_flt_SHIFT												2
#define c_cross_flt_PRESC_SHIFT									2
#define c_cross_flt_VALUE_SHIFT									2

#define c_lat_flt_SHIFT													2
#define c_lat_flt_PRESC_SHIFT										2
#define c_lat_flt_VALUE_SHIFT										2

#define motor_LATERAL_CURRENT_PI_PROP_GAIN			250
#define motor_LATERAL_CURRENT_PI_MODEL_GAIN			70
#define motor_LATERAL_CURRENT_PI_DIFF_GAIN			0
#define motor_LATERAL_CURRENT_PI_FORCE_GAIN			0
#define motor_LATERAL_CURRENT_PI_CONTROL_SHIFT	7
#define motor_LATERAL_CURRENT_PI_MODEL_SHIFT		10
#define motor_LATERAL_CURRENT_PI_RAMP						100
#define motor_LATERAL_CURRENT_RANGE_LO					-500
#define motor_LATERAL_CURRENT_RANGE_HI					500

#define motor_CROSS_CURRENT_PI_PROP_GAIN				250
#define motor_CROSS_CURRENT_PI_MODEL_GAIN				70
#define motor_CROSS_CURRENT_PI_DIFF_GAIN				0
#define motor_CROSS_CURRENT_PI_FORCE_GAIN				0
#define motor_CROSS_CURRENT_PI_CONTROL_SHIFT		7
#define motor_CROSS_CURRENT_PI_MODEL_SHIFT			10
#define motor_CROSS_CURRENT_PI_RAMP 						1000
#define motor_CROSS_CURRENT_RANGE_LO 						BURST_SIGNAL_MIN
#define motor_CROSS_CURRENT_RANGE_HI 						BURST_SIGNAL_MAX

#define hall_INV burst_false
#define hall_OFFSET_NATIVE (-BURST_SIGNAL_T(30./180))

#define PMSM_HALL_APP_EXTRA_TYPE PMSM_HALL_APP_EXTRA_TYPE_REGRESS
#define motor_CONTROL_PRESC 2
