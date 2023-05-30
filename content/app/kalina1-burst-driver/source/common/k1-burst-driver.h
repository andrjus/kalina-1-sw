#ifndef  k1_burst_driver_h
#define k1_burst_driver_h

#include "burst\modules\pmsm_hall_app.h"

//פנטלאסעונ
#define CLCH_NAME fm
#define CLCH_HEADER 
#include "burst/cliche/fm.h"

void adc_start(void);

void k1_serial_send_refuse(void);
void k1_serial_send_complete(void);
void k1_serial_receive_packet(const uint8_t * _data, uint8_t _sz);
void k1_serial_send_packet(uint8_t * _data, uint8_t _sz);
void k1_serial_aborttx(void);
void k1_serial_pool(void);
void k1_serial_start_receive(void);

burst_bool_t k1_serial_ready(void);

#endif
