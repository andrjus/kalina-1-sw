#include "k1-burst-driver.h"
//פנטלאסעונ
#define CLCH_NAME fm
#include "burst/cliche/fm.h"

#if TMP423_ENABLED == 1
#define CLCH_NAME TMP423
#include "burst/cliche/net_master.h"
#endif
#if K1_BOARD_COMMON_VER >=2
#if K1_BOARD_SERIAL_1_ENABLED
#define CLCH_NAME serial1
#include "burst/cliche/net_serial_servo.h"
#endif
#if K1_BOARD_SERIAL_2_ENABLED
#define CLCH_NAME serial2
#include "burst/cliche/net_serial_servo.h"
#endif
#if K1_BOARD_SERIAL_3_ENABLED
#define CLCH_NAME serial3
#include "burst/cliche/net_serial_servo.h"
#endif

#if BOARD_TEMPER_SENCE_ABC_ENABLED
#define CLCH_NAME temper_ABC
#include "burst/cliche/lookuptable.h"
#endif


#if BOARD_TEMPER_SENCE_MK_ENABLED
#define CLCH_NAME temper_MK
#include "burst/cliche/lookuptable.h"
#endif

#endif
