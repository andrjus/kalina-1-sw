#ifndef  k1_burst_driver_h
#define k1_burst_driver_h

#include "burst\modules\pmsm_hall_app.h"

//פנטלאסעונ
#define CLCH_NAME fm
#define CLCH_HEADER 
#include "burst/cliche/fm.h"

//swt
#define CLCH_NAME swt
#define CLCH_HEADER 
#include "burst/cliche/swt.h"



#if TMP423_ENABLED == 1
#define CLCH_NAME TMP423
#define CLCH_HEADER 
#include "burst/cliche/net_master.h"
void TMP423_read(uint8_t _addr, uint8_t * data);
void TMP423_write(uint8_t _addr, uint8_t data);
#endif

void adc_start(void);

void k1_serial_send_refuse(void);
void k1_serial_send_complete(void);
void k1_serial_receive_packet(const uint8_t * _data, uint8_t _sz);
void k1_serial_send_packet(uint8_t * _data, uint8_t _sz);
void k1_serial_aborttx(void);
void k1_serial_pool(void);
void k1_serial_start_receive(void);

burst_bool_t k1_serial_ready(void);
typedef struct temper_raw_s{
	struct{
		int16_t motor;
		#if TMP423_ENABLED == 1
		struct{
			int16_t A;
			int16_t B;
			int16_t C;
			int16_t Z;
		} board;
		#else
		int16_t board;
		#endif
	} dg;
	struct{
		uint16_t motor;
		#if TMP423_ENABLED == 1
		struct{
			uint16_t A;
			uint16_t B;
			uint16_t C;
			uint16_t Z;
		} board;
		#else
		uint16_t board;
		#endif
	} raw;
} temper_t;
extern temper_t temper;

typedef  struct voltage_s{
	uint16_t raw;
	int16_t mVolt;
} voltage_t;

extern voltage_t voltage;


void delay_us(volatile uint32_t _us);

#endif
