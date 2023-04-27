#ifndef  fb3_burst_h
#define fb3_burst_h
#include "burst/burst.h"
#include "burst/burst_serial.h"
#include "burst/modules/burst_inv3ph.h"
#include "burst/burst_hall.h"
#include "burst/burst_adc.h"
#include "burst/burst_pi.h"
#include "burst/modules/nikitin.h"

//фримастер
#define CLCH_NAME fm
#define CLCH_HEADER 
#include "burst/cliche/fm.h"

//силовой преобразователь
#define CLCH_NAME power
#define CLCH_HEADER 
#include "burst/cliche/ps.h"

NIKITIN(speedse)

BURST_PI(lat_current_pi)

extern inv3ph_t inverter;
extern current3ph_t curse;
extern hall_t hall;
extern adc_t adc;
extern nikitin_t speedse;
void adc_start(void);


#endif
