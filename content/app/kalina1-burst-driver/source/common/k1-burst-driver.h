#ifndef  k1_burst_driver_h
#define k1_burst_driver_h

#include "burst/modules/pmsm_hall_app.h"
#include "burst/net/burst_net_flow.h"

//פנטלאסעונ
#define CLCH_NAME fm
#define CLCH_HEADER 
#include "burst/cliche/fm.h"

#if BOARD_TEMPER_SENCE_ABC_ENABLED
#define CLCH_NAME temper_ABC
#define CLCH_HEADER 
#include "burst/cliche/lookuptable.h"
#endif


#if BOARD_TEMPER_SENCE_MK_ENABLED
#define CLCH_NAME temper_MK
#define CLCH_HEADER 
#include "burst/cliche/lookuptable.h"
#endif

void adc_start(void);



#if K1_BOARD_COMMON_VER >= 2

#if K1_BOARD_SERIAL_1_ENABLED
#define CLCH_NAME serial1
#define CLCH_HEADER 
#include "burst/cliche/net_serial_servo.h"
#endif
#if K1_BOARD_SERIAL_2_ENABLED
#define CLCH_NAME serial2
#define CLCH_HEADER 
#include "burst/cliche/net_serial_servo.h"
#endif
#if K1_BOARD_SERIAL_3_ENABLED
#define CLCH_NAME serial3
#define CLCH_HEADER 
#include "burst/cliche/net_serial_servo.h"
#endif

#if K1_BOARD_NET_FLOW_ENABLED
#include "burst/net/flow_serial.h"
#endif

typedef  struct voltage_s{
		uint16_t raw;
		int16_t cVolt;
} voltage_t;
typedef  struct temper_s{
		uint16_t raw;
		int16_t dgC;
} temper_t;
typedef  struct current_s{
		uint16_t raw;
		int16_t dcA;
} current_t;
typedef  struct board_s{
	struct{
		voltage_t ref;
		voltage_t in;		
		struct {
			voltage_t A;
			voltage_t B;
			voltage_t C;
		} ps;
	} voltage;
	struct{
		voltage_t ref;
		voltage_t in;
	} curren;
	struct{
		#if BOARD_TEMPER_SENCE_MK_ENABLED		
		temper_t mk;	
		#endif
		struct {
			#if BOARD_TEMPER_SENCE_ABC_ENABLED		
			temper_t A;
			temper_t B;
			temper_t C;
			#endif
			#if BOARD_TEMPER_SENCE_Z_ENABLED
			temper_t Z;
			#endif
		} ps;
		burst_signal_t hi;
		burst_signal_t lo;
	} temper;
} board_t;

#if K1_BOARD_NET_FLOW_ENABLED
void k1_proto_servo_begin(void);
#endif

#else

#if K1_BOARD_SERIAL_1_ENABLED
void k1_serial_send_refuse(void);
void k1_serial_send_complete(void);
void k1_serial_receive_packet(const uint8_t * _data, uint8_t _sz);
burst_time_us_t k1_serial_send_packet(uint8_t * _data, uint8_t _sz);
void k1_serial_aborttx(void);
void k1_serial_pool(void);
void k1_serial_start_receive(void);
burst_bool_t k1_serial_ready(void);
#endif

#if TMP423_ENABLED == 1
#define CLCH_NAME TMP423
#define CLCH_HEADER 
#include "burst/cliche/net_master.h"
void TMP423_read(uint8_t _addr, uint8_t * data);
void TMP423_write(uint8_t _addr, uint8_t data);
#endif



typedef struct temper_raw_s{
	struct{
		int16_t motor;

	} dg;
	struct{
		uint16_t motor;
		#if TMP423_ENABLED == 0
		uint16_t board;
		#endif
	} raw;
} temper_t;
//extern temper_t temper;

typedef  struct board_s{
	struct {
		uint16_t raw;
		int16_t mVolt;
	} voltage;
	struct {
		uint16_t raw;
		int32_t mA;
	} current;
	#if TMP423_ENABLED == 1
	struct{
		burst_signal_t A;
		burst_signal_t B;
		burst_signal_t C;
		burst_signal_t Z;
		burst_signal_t mean;
	} temper;
	#else
	struct{
		uint16_t raw;
		burst_signal_t dg;
	} temper;
	
	#endif
} board_t;
#endif

extern board_t board;


void delay_us(volatile uint32_t _us);

extern uint32_t adc_tot_offset;
#endif
