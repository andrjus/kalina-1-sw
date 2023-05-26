#if (!defined(burst_app_tuning_h)) && defined(burst_common_h)
#define burst_common_h
#else
#error error of using burst_app_tuning.h
#endif

#define BURST_DEV_COUNT 1

#include "k1-burst-driver.common.h"

#define BURST_DEBUG_TP_ENABLED 1

#define BURST_SLOT_COUNT 16

#define BURST_TIMER_ENABLED 1

#define BURST_TIMER_TICK_US K1_PWM_PERIOD_US

/*

#define HVD_TICK_PERIOD_US 50
#define HVD_POOL_PERIOD_US 10
#define HVD_PWM_MODULO (100000000ULL*HVD_TICK_PERIOD_US/1000000/2)
#define HVD_POLL_MODULO (100000000ULL*HVD_POOL_PERIOD_US/1000000/2)


#define HVD_SERIAL0 hvd_usart

#define BURST_APP_FREEMASTER_SERIAL_ENABLED 1

#define FMSTR_REC_TIMEBASE 32768 + HVD_TICK_PERIOD_US



#define BURST_TIMER_ENABLED 1
#define BURST_TIMER_TICK_US HVD_TICK_PERIOD_US


#define BURST_BUTTON_ENABLED 1
#define BURST_BTN_MAX_COUNT 4
*/
