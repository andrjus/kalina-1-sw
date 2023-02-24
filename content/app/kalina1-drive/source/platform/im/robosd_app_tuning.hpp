#if (!defined(__robosd_app_tunning_hpp)) && defined(__robosd_common_hpp)
#define __robosd_app_tunning_hpp
#else
#error error of using robosd_app_tunning.hpp
#endif

#include "mexo/mexo.env.common.hpp"

#define mexo_drive_types_t ::mexo::fixed_point<::mexo::int15>

#define ROBO_UNICODE_ENABLED 0

#define ROBO_APP_DEBUG_LOG_ENABLED 1

#include <stdint.h>

//#define ROBO_APP_MEXO_SIDE

#define ROBO_APP_MEXO_VAR_MODE full
#define ROBO_APP_SYSTEM_ENABLED 1
#define ROBO_APP_PRINT_TYPE ROBO_APP_TYPE_WIN
#define ROBO_APP_FORMATING_TYPE ROBO_APP_TYPE_STD
#define ROBO_LOG_APP_PRINT_TYPE ROBO_APP_TYPE_WIN
#define ROBO_APP_INI_TYPE ROBO_APP_TYPE_WIN

#define ROBO_APP_ENV_TYPE ROBO_APP_TYPE_WIN
#define ROBO_APP_LIB_TYPE ROBO_APP_TYPE_NONE
#define ROBO_APP_ALLOC_TYPE ROBO_APP_TYPE_STD

#define ROBO_APP_CRITICAL_TYPE ROBO_APP_TYPE_WIN
#define ROBO_APP_OS_TYPE ROBO_APP_TYPE_WIN

#define ROBO_APP_MEXO_SLOT_COUNT 4
#define ROBO_APP_MEXO_VAR_ENABLED 1

#define ROBO_APP_SHARED_TYPE ROBO_APP_TYPE_WIN

#define FMSTR_ADDRESS_OFFSET_TYPE uintptr_t
extern uintptr_t MODULE_ADDRESS;
#define FMSTR_ADDRESS_OFFSET MODULE_ADDRESS

#define  FMSTR_REC_BUFF_SIZE 100000

#ifndef MEXO_DEVICE_SIDE
#define MEXO_DEVICE_SIDE
#endif

#define ROBO_APP_MEXO_SAMPLE_US KALINA1_PWM_PERIOD_US

