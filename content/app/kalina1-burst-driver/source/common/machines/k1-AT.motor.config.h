#if (!defined(k1_ecmax_motor_config_h)) && defined(k1_burst_common_h)
#define k1_ecmax_motor_config_h
#else
#error error of using k1-ecmax.motor.config.h
#endif

#define K1_PWM_PERIOD_US 12L
#define K1_PWM_TIMER_PRESC 2
//#define PMSM_HALL_RPM_FILTER_VALUE_PRESC 0
//#define PMSM_HALL_RPM_FILTER_GAIN_PRESC 5
//#define PMSM_HALL_RPM_VALUE_TOTAL_PRESC 5
#define c_cross_flt_SHIFT												3
#define c_cross_flt_PRESC_SHIFT									3
#define c_cross_flt_VALUE_SHIFT									3

#define c_lat_flt_SHIFT													3
#define c_lat_flt_PRESC_SHIFT										3
#define c_lat_flt_VALUE_SHIFT										3

#define motor_LATERAL_CURRENT_PI_PROP_GAIN			6000
#define motor_LATERAL_CURRENT_PI_MODEL_GAIN			150
#define motor_LATERAL_CURRENT_PI_DIFF_GAIN			0
#define motor_LATERAL_CURRENT_PI_FORCE_GAIN			0
#define motor_LATERAL_CURRENT_PI_CONTROL_SHIFT	7
#define motor_LATERAL_CURRENT_PI_MODEL_SHIFT		10
#define motor_LATERAL_CURRENT_PI_RAMP						100
#define motor_LATERAL_CURRENT_RANGE_LO					-500
#define motor_LATERAL_CURRENT_RANGE_HI					500

#define motor_CROSS_CURRENT_PI_PROP_GAIN				6000
#define motor_CROSS_CURRENT_PI_MODEL_GAIN				150
#define motor_CROSS_CURRENT_PI_DIFF_GAIN				0
#define motor_CROSS_CURRENT_PI_FORCE_GAIN				0
#define motor_CROSS_CURRENT_PI_CONTROL_SHIFT		7
#define motor_CROSS_CURRENT_PI_MODEL_SHIFT			10
#define motor_CROSS_CURRENT_PI_RAMP 						1000
#define motor_CROSS_CURRENT_RANGE_LO 						BURST_SIGNAL_MIN
#define motor_CROSS_CURRENT_RANGE_HI 						BURST_SIGNAL_MAX

#define hall_INV burst_false //burst_true
#define hall_OFFSET_NATIVE  (-BURST_SIGNAL_T(45./180))

#define PMSM_HALL_APP_EXTRA_TYPE PMSM_HALL_APP_EXTRA_TYPE_REGRESS
#define motor_CONTROL_PRESC 1

#define BURST_PANICS_ACTUATOR_TEMPER_ENABLED 0
#if BURST_PANICS_ACTUATOR_TEMPER_ENABLED == 1
#define K1_ACCTUATOR_TEMPER_GRAD_TO_PP(x) ((burst_signal_t)((x+100)>>4))
#define motor_PANICS_ACTUATOR_TEMPER_OVERHI_PP K1_ACCTUATOR_TEMPER_GRAD_TO_PP(110)
#define motor_PANICS_ACTUATOR_TEMPER_HI_PP K1_ACCTUATOR_TEMPER_GRAD_TO_PP(85)
#define motor_PANICS_ACTUATOR_TEMPER_LO_PP K1_ACCTUATOR_TEMPER_GRAD_TO_PP(-15)
#define motor_PANICS_ACTUATOR_TEMPER_ULTRALO_PP K1_ACCTUATOR_TEMPER_GRAD_TO_PP(-20)
#endif

#define BURST_PANICS_ACWC_OVERCURRENT_ENABLED 1
#if BURST_PANICS_ACWC_OVERCURRENT_ENABLED == 1
#define motor_PANICS_ACWC_OWERCURRENT_PP K1_MOTOR_CURRENT_A_TO_PP(20)
#define motor_PANICS_ACWC_OWERPOWER_PP  K1_MOTOR_CURRENT_A_TO_PP(12)
#define motor_PANICS_ACWC_NORMPOWER_PP  K1_MOTOR_CURRENT_A_TO_PP(10)
#define motor_PANICS_ACWC_OWERPOWER_TM_US 3000000
#endif

#define BURST_PANICS_PMSM_MISSALIGMENT_ENABLED 0
#define motor_PANICS_PMSM_CURRENT_MISSALIGMENT_PP FB3_MOTOR_CURRENT_A_TO_PP(2)

#define motor_RANGE_SPEED_LO -300
#define motor_RANGE_SPEED_HI 300
#define motor_MOTION_OV_CURRENT_PROP_GAIN 500
#define motor_MOTION_OV_CURRENT_DIFF_GAIN 0
#define motor_MOTION_OV_CURRENT_MODEL_GAIN 17
#define motor_MOTION_OV_CURRENT_CONTROL_SHIFT 7
#define motor_MOTION_OV_CURRENT_MODEL_SHIFT 10
