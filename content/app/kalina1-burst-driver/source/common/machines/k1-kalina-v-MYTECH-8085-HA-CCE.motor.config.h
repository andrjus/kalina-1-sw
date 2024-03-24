#if (!defined(k1_motor_config_h)) && defined(k1_burst_common_h)
#define k1_motor_config_h
#else
#error error of using k1-kalina-v-MYTECH-8085-HA-CCE.motor.config.h
#endif

#define K1_PWM_PERIOD_US 50L
#define K1_PWM_TIMER_PRESC 1

#define c_cross_flt_SHIFT												4
#define c_cross_flt_PRESC_SHIFT									4
#define c_cross_flt_VALUE_SHIFT									4

#define c_lat_flt_SHIFT													4
#define c_lat_flt_PRESC_SHIFT										4
#define c_lat_flt_VALUE_SHIFT										4

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
#define hall_OFFSET_NATIVE  (-BURST_SIGNAL_T(150./180))

#define PMSM_HALL_APP_EXTRA_TYPE PMSM_HALL_APP_EXTRA_TYPE_NONE
#define motor_CONTROL_PRESC 1


#define BURST_PANICS_ACWC_OVERCURRENT_ENABLED 1
#if BURST_PANICS_ACWC_OVERCURRENT_ENABLED == 1
#define motor_PANICS_ACWC_OWERCURRENT_PP K1_MOTOR_CURRENT_A_TO_PP(150)
#define motor_PANICS_ACWC_OWERPOWER_PP K1_MOTOR_CURRENT_A_TO_PP(120)
#define motor_PANICS_ACWC_NORMPOWER_PP K1_MOTOR_CURRENT_A_TO_PP(100)
#define motor_PANICS_ACWC_OWERPOWER_TM_US 3000000
#endif

#define motor_RANGE_SPEED_LO -300
#define motor_RANGE_SPEED_HI 300
#define motor_MOTION_OV_CURRENT_PROP_GAIN 500
#define motor_MOTION_OV_CURRENT_DIFF_GAIN 0
#define motor_MOTION_OV_CURRENT_MODEL_GAIN 17
#define motor_MOTION_OV_CURRENT_CONTROL_SHIFT 7
#define motor_MOTION_OV_CURRENT_MODEL_SHIFT 10



#define  BURST_APP_PMSM_BLDC_ENABLED 1
#if  BURST_APP_PMSM_BLDC_ENABLED
#define swt_CONFIG_GAIN_PROP 175
#define swt_CONFIG_GAIN_MODEL 300
#define swt_CONFIG_PWM_RAMP32 200
#define swt_CONFIG_CURRENT_RAMP32 200
#define swt_CONFIG_CURRENT_MIN -K1_MOTOR_CURRENT_A_TO_PP(100)
#define swt_CONFIG_CURRENT_MAX K1_MOTOR_CURRENT_A_TO_PP(100)
#endif
