#if (!defined(kalina1_drive_config_hpp)) && defined(kalina1_drive_common_hpp)
#define kalina1_drive_config_hpp
#else
#error error of using kalina1-drive-im.config.hpp
#endif

#define KALINA1_PWM_PERIOD_US 50L
#define KALINA1_ADC_SENCE_TOTAL_TIME_US 24

#define KALINA1_ACTUATOR_ENCO_MAX types::long_max
#define KALINA1_ACTUATOR_ENCO_MIN types::long_min

#define KALINA1_CPU_FREQ_HZ 72000000
#define KALINA1_ADC_SENCE_TIME_US 2
#define KALINA1_PWM_DEADTIME_US .5
#define KALINA1_BRK_SENCE_TIME_US 2
#define KALINA1_MAX_CURRENT_GAIN 66.
#define KALINA1_MAX_CURRENT_A 60
#define KALINA1_ROTOR_POLE_COUNT 7

#define KALINA1_MOTOR_ENCO_ABS_BITS 16

