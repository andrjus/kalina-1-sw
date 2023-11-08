#if !  defined(k1_burst_common_h)  
#error error of using k1-burst-driver.names.h
#endif
#ifdef K1_TAG_A60_7M
#define K1_MOTOR_CONFIG "machines/k1-A60-7M.motor.config.h"
#endif

#ifdef K1_TAG_MYTECH_8085_HA_CCE
#define K1_MOTOR_CONFIG "machines/k1-MYTECH-8085-HA-CCE.motor.config.h"
#endif

#ifdef K1_TAG_NH_MYTECH_8085_HA_CCE
#define K1_MOTOR_CONFIG "machines/k1-nh-MYTECH-8085-HA-CCE.motor.config.h"
#endif



#ifdef K1_TAG_BOARD_ALFA
#define K1_PFR_CONFIG "machines/k1-alfa.pfr.config.h"
#endif

#ifdef K1_TAG_BOARD_BETA
#define K1_PFR_CONFIG "machines/k1-beta.pfr.config.h"
#endif

#ifdef K1_TAG_BOARD_NH
#define K1_PFR_CONFIG "machines/k1-nh.pfr.config.h"
#endif

#ifdef K1_TAG_BOARD_KIPARIS_2
#define K1_PFR_CONFIG "machines/k1-kiparis2.pfr.config.h"
#endif

#ifdef K1_TAG_KIPARIS_2_AT_38_12
#define K1_MOTOR_CONFIG "machines/k1-kiparis2-AT38-12.motor.config.h"
#endif
