#include "k1-burst-driver.h"

//фримастер
#define CLCH_NAME fm
#include "burst/cliche/fm.h"

//силовой преобразователь
#define CLCH_NAME power
#include "burst/cliche/ps.h"

NIKITIN_CREATE(speedse)

BURST_PI_CREATE(lat_current_pi)
