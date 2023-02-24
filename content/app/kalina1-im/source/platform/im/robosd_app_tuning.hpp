#if (!defined(__robosd_app_tunning_hpp)) && defined(__robosd_common_hpp)
#define __robosd_app_tunning_hpp
#else
#error error of using robosd_app_tunning.hpp
#endif

#ifdef UNICODE
#define ROBO_UNICODE_ENABLED 1
#else
#define ROBO_UNICODE_ENABLED 0
#endif


#define ROBO_APP_DEBUG_LOG_ENABLED 1

#define ROBO_APP_SYSTEM_ENABLED 1

#define ROBO_APP_PRINT_TYPE ROBO_APP_TYPE_WIN
#define ROBO_APP_FORMATING_TYPE ROBO_APP_TYPE_STD
#define ROBO_LOG_APP_PRINT_TYPE ROBO_APP_TYPE_WIN

#define ROBO_APP_ENV_TYPE ROBO_APP_TYPE_WIN
#define ROBO_APP_INI_TYPE ROBO_APP_TYPE_WIN
#define ROBO_APP_LIB_TYPE ROBO_APP_TYPE_NONE
#define ROBO_APP_ALLOC_TYPE ROBO_APP_TYPE_STD
#define ROBO_APP_SHARED_TYPE ROBO_APP_TYPE_WIN
#define ROBO_APP_CONSOL_TYPE ROBO_APP_TYPE_NONE
#include <stdint.h>

#define ROBO_APP_FREEMASTER_SERIAL_ENABLED 1
#define ROBO_APP_NET_FLOW_ENABLED 1
#define APP_MEXO_SLOT_COUNT 4
#define FMSTR_REC_TIMEBASE (32768+50)

#define ROBO_APP_TERMINAL_ENABLED 1
#define ROBO_APP_PROTO_SWITCH_ENABLED 1
#define ROBO_APP_MEXO_VAR_ENABLED 1

