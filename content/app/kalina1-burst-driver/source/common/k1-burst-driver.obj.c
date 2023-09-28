#include "k1-burst-driver.h"
//פנטלאסעונ
#define CLCH_NAME fm
#include "burst/cliche/fm.h"

#if TMP423_ENABLED == 1
#define CLCH_NAME TMP423
#include "burst/cliche/net_master.h"
#endif
