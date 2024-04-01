#include "k1-burst-driver.h"
#include "burst/burst_timer.h"
#include "burst/burst_app.h"
#include "burst/burst_sqrt.h"
pmsm_hall_app_config_t  k1_config = PMSM_HALL_APP_CONFIG();
pmsm_action_t pmsma = {};	
pmsm_feedback_t feedback = {};
void burst_sw_begin(void){
	pmsm_hall_app_begin(&k1_config, &pmsma, &feedback);
}
board_t board = {};
#if K1_BOARD_COMMON_VER >=2

int burst_board_temper_get_hi_pp(void){
	return board.temper.hi;
}
int burst_board_temper_get_lo_pp(void){
	return board.temper.lo;
}

int burst_board_voltage_get_pp(void){
	return (655L * board.voltage.in.cVolt + 32767)>>16; //деление на 100 с окуруглением
}
#else
	int burst_board_temper_get_hi_pp(void){
	return board.temper.dg;
}
int burst_board_temper_get_lo_pp(void){
	return board.temper.dg;
}

int burst_board_voltage_get_pp(void){
	return (66L * board.voltage.mVolt + 32767)>>16; //деление на 1000 с окуруглением
}
#endif

#if K1_BOARD_COMMON_VER <2
#if TMP423_ENABLED == 1
int burst_board_temper_get_pp(void){
	burst_signal_t s =  board.temper.A +  board.temper.B +  board.temper.C;
	board.temper.mean = s_mult(s, BURST_SIGNAL_T(1.0/3));
	return board.temper.mean;
}
#else
int burst_board_temper_get_pp(void){
	return board.temper.raw;
}
#endif
	
#if TMP423_ENABLED == 1

uint8_t tables[13] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t addr[13] = {0x00,0x01,0x02,0x03,0x08,0x09,0xA,0x0B,0x10,0x11,0x12,0x13,0xFE};


int ix = 13;


void TMP423_confirm_callback(burst_bool_t _r){
	BURST_UNUSED(_r);
	switch(ix){
		case 1: board.temper.Z = (int8_t)tables[0]; break;
		case 2: board.temper.A = (int8_t)tables[1]; break;
		case 3: board.temper.B = (int8_t)tables[2]; break;
		case 4: board.temper.C = (int8_t)tables[3]; break;
	}
	if(ix <13){
		TMP423_read(addr[ix], tables+ix);
		ix++;
	}
}

burst_run_t temp_poll_(void){
	if(ix ==13){
		if(TMP423_ready()){
			ix = 0;
			TMP423_read(addr[ix], tables+ix);
			ix++;
		}		
	}
	return burst_continue;
}

burst_timer_t temp_poll ={
	0
	,10000
	,&temp_poll_
	,0
	,0
};
#endif
#endif

#if K1_BOARD_COMMON_VER >= 2
burst_serial_p fm_serial_ = 
#if K1_FREEMASTER_TYPE == K1_FREEMASTER_TYPE_SERIAL_1
&serial1.serial
#endif
#if K1_FREEMASTER_TYPE == K1_FREEMASTER_TYPE_SERIAL_2
&serial2.serial
#endif
#if K1_FREEMASTER_TYPE == K1_FREEMASTER_TYPE_SERIAL_3
&serial3.serial
#endif
#if K1_FREEMASTER_TYPE == K1_FREEMASTER_TYPE_OV_FLOW_PROTO
&burst_net_flow_serial
#endif
;
uint8_t	fm_available(void){
	return fm_serial_->available() ? 1:0; 
}
uint8_t	fm_space(void){
	return  fm_serial_->space()?1:0; 
}
uint8_t	fm_get(void){
	return  fm_serial_->get(); 
}
void		fm_put(uint8_t _data){
	fm_serial_->put(_data);
}
#endif

void burst_sw_start(void){	
	adc_start();
	while( adc.ready == burst_false ){
		BURST_NOP();
	}	
	adc_tot_offset = adc.offset[0]+adc.offset[1]+adc.offset[2];
	if( motor.cross.ac.ref.panic == 2){
		if( burst_core_panics() == 1<<4 ){
			burst_core_reset_panics();
		}
	}
	pmsm_hall_app_start();
	#if TMP423_ENABLED == 1
	TMP423_start();
	burst_timer_start(&temp_poll);
	#endif
	burst_sqrt_init();
	#if K1_BOARD_NET_FLOW_ENABLED
	k1_proto_servo_begin();
	#endif
}

void burst_sw_realtime_loop(void){	
	pmsm_hall_app_realtime_loop();
	enco.ref.run();
}
#if 0
static uint32_t sqrt_test_x = 0;
static volatile uint16_t sqrt_test_y = 0;
static uint16_t sqrt_test_y0 = 0;
#endif
#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) > (Y)) ? (X) : (Y))

void burst_sw_backend_loop(void){		
	pmsm_hall_app_backend_loop();
	fm_recorder();
	fm_poll();
//	machine();
	#if 0
	sqrt_test_y0++;
	if(sqrt_test_y0>4095){
		sqrt_test_y0 = 0;
	}
	debug_tp_on(112);
	sqrt_test_x = sqrt_test_y0*sqrt_test_y0;
	sqrt_test_y = burst_sqrt(sqrt_test_x);
	debug_tp_off(112);
	#endif
	#if K1_BOARD_COMMON_VER >= 2
	#if 1
	static uint16_t temper_test_pp = 0;
	static volatile burst_signal_t temper_test_dg = 0;
	temper_test_pp++;
	if(temper_test_pp>4100){
		temper_test_pp = 0;
	}
	temper_test_dg = temper_ABC_from_pp((burst_signal_t)temper_test_pp);
	#endif
	board.temper.lo = +32767;
	board.temper.hi = -32767;
	#if BOARD_TEMPER_SENCE_ABC_ENABLED		
	board.temper.ps.A.dgC = temper_ABC_from_pp((burst_signal_t)board.temper.ps.A.raw);
	board.temper.ps.B.dgC = temper_ABC_from_pp((burst_signal_t)board.temper.ps.B.raw);
	board.temper.ps.C.dgC = temper_ABC_from_pp((burst_signal_t)board.temper.ps.C.raw);
	board.temper.hi = MAX(
		MAX(board.temper.ps.A.dgC,board.temper.ps.B.dgC)
		,MAX(board.temper.ps.C.dgC,board.temper.hi)
	);
	board.temper.lo = MIN(
		MIN(board.temper.ps.A.dgC,board.temper.ps.B.dgC)
		,MIN(board.temper.ps.C.dgC,board.temper.lo)
	);
	#endif
	#if BOARD_TEMPER_SENCE_MK_ENABLED		
	board.temper.mk.dgC = temper_MK_from_pp((burst_signal_t)board.temper.mk.raw);
	if(board.temper.hi<board.temper.mk.dgC){
		board.temper.hi = board.temper.mk.dgC;
	}
	if(board.temper.lo > board.temper.mk.dgC){
		board.temper.lo = board.temper.mk.dgC;
	}
	#endif
	#if BOARD_TEMPER_SENCE_Z_ENABLED		
	if(board.temper.max<board.temper.Z.dgC){
		board.temper.max = board.temper.Z.dgC;
	}
	if(board.temper.min > board.temper.Z.dgC){
		board.temper.min = board.temper.Z.dgC;
	}
	#endif
	#if BOARD_VOLTAGE_SENCE_ENABLED
	 board.voltage.in.cVolt = K1_BOARD_VOLTAGE_PP_TO_CVOLT( board.voltage.in.raw);
	#endif  
	#endif
}

int synchro_test_enable = 0;
int synchro_test_level = 10000000;
void burst_sw_frontend_loop(void){
	pmsm_hall_app_frontend_loop();
	#if K1_BOARD_COMMON_VER >= 2
	#if K1_BOARD_SERIAL_1_ENABLED
	serial1.pool();
	#endif
	#else
	k1_serial_pool();
	#endif
	static burst_time_us_t us = 0;
	burst_time_us_t now = burst_time_us();
	if(synchro_test_enable){
		if( now - us >10000 ){
			if(motor.synchro.freq < synchro_test_level){
				if( motor.cross.ac.voltage.req < 30000){
					motor.synchro.freq += 50000;
					us = now;
				} else {
					synchro_test_enable = 0;
				}
			} else{
				//synchro_test_enable = 0;
			}
		}
	}
	
	#if K1_BOARD_COMMON_VER <2
	#if BURST_PANICS_BOARD_TEMPER_ENABLED == 1
	#if TMP423_ENABLED  == 0
	board.temper.dg= K1_BOARD_TEMPER_PP_TO_GRAD(board.temper.raw);
	#endif
	#endif

	#if BURST_PANICS_BOARD_VOLTAGE_ENABLED == 1
	board.voltage.mVolt = K1_BOARD_VOLTAGE_PP_TO_CVOLT(board.voltage.raw);
	#endif

	#if BURST_PANICS_BOARD_CURRENT_ENABLED == 1
	board.current.mA = K1_BOARD_CURRENT_PP_TO_MAMPER(board.current.raw);
	#endif
	#endif

}

#if K1_BOARD_NET_FLOW_ENABLED
#include "k1-burst-servo.proto.h"
#include "burst/net/flow_serial.h"

void burst_net_flow_sended(uint8_t _suba){
	switch(_suba){
		case k1_proto_cmd_serial:
		{
			uint8_t * outcom = burst_net_flow_outcom_get(k1_proto_cmd_serial,8);
			if(outcom){
				uint8_t tmp = burst_net_flow_serial_execute(0,0, outcom,8);
				if(tmp>0){
					burst_net_flow_outcom_post(k1_proto_cmd_serial,tmp);
				}
			}			
		} break;	
	}
}

void burst_net_flow_perform(uint8_t _suba,const uint8_t * _data, uint8_t _sz){
	switch(_suba){		
		case k1_proto_cmd_serial:
		{
			uint8_t * outcom = burst_net_flow_outcom_get(k1_proto_cmd_serial,8);
			if(outcom){
				uint8_t tmp = burst_net_flow_serial_execute(_data,_sz, outcom,8);
				if(tmp>0){
					burst_net_flow_outcom_post(k1_proto_cmd_serial,tmp);
				}
			}			
		} break;	
	}
}

#endif

void burst_sw_slot_0(void){
	pmsm_hall_app_control_step_1();
}

void burst_sw_slot_1(void){
	pmsm_hall_app_control_step_2();
}
void burst_sw_slot_2(void){
	pmsm_hall_app_control_step_3();
}

uint8_t swt_phy_sector_get(void){	
	/*
	burst_signal_t angle ;
	if(angle_forcer.ref.electro.speed>1 || angle_forcer.ref.electro.speed<-1){
		angle = angle_forcer.ref.electro.angle;
	} else{
		angle = hall.angle;		
	}
	if(angle>0){
		return ((6L * angle+32768)>>16) ;
	} else {
		return 6-((-6L * angle+32768)>>16) ;
	}
	*/
	return hall.sector;
}


#if K1_BOARD_COMMON_VER >=2

#else
#if K1_BOARD_SERIAL_1_ENABLED
#define RING_PREFIX_NAME fmincom
#define RING_SIZE_BITS 7
#define RING_LOCK() uint32_t context = burst_guard_enter();
#define RING_UNLOCK() burst_guard_leave(context);
#include "burst/burst_ring.inc.h"

#define RING_PREFIX_NAME fmoutcom
#define RING_SIZE_BITS 7
#define RING_LOCK() uint32_t context = burst_guard_enter();
#define RING_UNLOCK() burst_guard_leave(context);
#include "burst/burst_ring.inc.h"

uint8_t  fm_available(void){
	return fmincom_count();
}

uint8_t  fm_space(void){
	return fmoutcom_size_ - fmoutcom_count();
}

uint8_t fm_get(void){ 
	return fmincom_get_();
}


void fm_put(uint8_t _data){
	fmoutcom_put_(_data);
}

burst_size_t fm_getb(uint8_t* _buf, burst_size_t _max_sz){
	return fmincom_buf_get(_buf,_max_sz);
}

burst_bool_t fm_putb(const uint8_t* _buf, burst_size_t _sz){
	return  fmoutcom_buf_put(_buf, _sz);
}

uint8_t irga1_serial_tx_size = 0;
burst_bool_t feedback_send_need = burst_false;

void k1_serial_receive_packet(const uint8_t * _data, uint8_t _sz){
	fmincom_buf_put(_data, _sz);
}

uint8_t k1_serial_tx_buffer[K1_SERIAL_SIZE];
burst_bool_t sending=burst_false;
uint32_t k1_serial_tx_sz;
burst_time_us_t k1_serial_timeout_tx;

void fmserial_try_send(void){
	k1_serial_tx_sz = fmoutcom_buf_get(k1_serial_tx_buffer,K1_SERIAL_SIZE);
	if(k1_serial_tx_sz){
		sending=burst_true;
		k1_serial_timeout_tx = k1_serial_send_packet( k1_serial_tx_buffer , k1_serial_tx_sz );			
	} 
}

void k1_serial_send_complete(void){
}

void k1_serial_send_refuse(void){
	fmserial_try_send();	
}

void k1_serial_pool(void){
	static burst_time_us_t tm=0;
	burst_time_us_t now = burst_time_us();
	burst_time_us_t d = now - tm;
	if( k1_serial_ready() ){
		fmserial_try_send();
		
		//k1_serial_tx_buffer[0] = 'H';
		//k1_serial_tx_buffer[1] = 'E';
		//k1_serial_tx_buffer[2] = 'l';
		//serial_tx_sz = 3;
		//k1_serial_send_packet( k1_serial_tx_buffer , serial_tx_sz );
		

		tm = now;
	}else{
		if( d> k1_serial_timeout_tx ){
			k1_serial_aborttx();
			tm = now;
		}
	}
}
#endif

#if TMP423_ENABLED == 1

void TMP423_read(uint8_t _addr, uint8_t * data){
	static uint8_t addr;
	static TMP423_packet_t op = {&addr,1};
	static TMP423_packet_t ip = {0,1};
	
	addr = _addr;
	ip.data = data;
	
	TMP423_exchange(&op,&ip);

}
void TMP423_write(uint8_t _addr, uint8_t data){
	static uint8_t outcom[2];
	static TMP423_packet_t op = {outcom,2};
	outcom[0] = _addr;
	outcom[0] = data;

	TMP423_exchange(&op,0);

}
#endif

#if BURST_PANICS_BOARD_VOLTAGE_ENABLED == 1
int burst_board_voltage_get_pp(void){
	return board.voltage.raw;
}
#endif



#if BURST_PANICS_BOARD_CURRENT_ENABLED == 1
int burst_board_current_get_pp(void){
	return board.current.raw;
}
#endif
#endif

#if 0
void machine(void){
	if(requried.mode != actual.mode){
		actual.mode = requried.mode;
		switch(actual.mode){
			case 0:
				power_command = clch_ps_command_off;
				break;
			case 1:
				power_command = clch_ps_command_on;
				actual.angle32 = 0;
				actual.angle = 0;
				break;
			case 2:
				power_command = clch_ps_command_on;
				break;
			case 3:
				power_command = clch_ps_command_on;
				break;
		}		
	}
	
	switch(actual.mode){
			case 1:
			actual.angle32 += requried.freq;
			actual.angle =  actual.angle32 >> 16;
			inv3ph_run(&inverter,requried.voltage.cross, requried.voltage.lateral, actual.angle);
			break;
			case 2:
			{
				actual.angle =  hall.angle;
				if(inverter.dq.cross<requried.voltage.cross){
					inverter.dq.cross++;
				} else if(inverter.dq.cross>requried.voltage.cross){
					inverter.dq.cross--;
				}
				burst_signal_t v = BURST_ABS(250 * speedse.ref.value);
				if(inverter.dq.cross>0){
					if(v>inverter.dq.cross){
						v=inverter.dq.cross;
					}
				} else if(inverter.dq.cross<0) {
					if(v<inverter.dq.cross){
						v=inverter.dq.cross;
					}				
				} else v = 0;
				inverter.dq.lateral = -v;
				//inv3ph_run(&inverter,hall_extra.angle32>>16);
			}
			break;
			case 3:
			{
				actual.angle =  hall.angle;
				eds = 1200L*speedse.ref.value;				
				voltage_min = eds-6000;
				voltage_max = eds+6000;
				if(requried.voltage.cross>voltage_max){
					inverter.dq.cross = voltage_max;
				} else if(requried.voltage.cross<voltage_min){
					inverter.dq.cross = voltage_min;
				}  else{
					if(inverter.dq.cross<requried.voltage.cross){
						inverter.dq.cross++;
					} else if(inverter.dq.cross>requried.voltage.cross){
						inverter.dq.cross--;
					}
				}
				lat_current_pi.run();				
				force_angle = (3823L* speedse.ref.value)>>5;
				//inv3ph_run(&inverter,(hall_extra.angle32>>16) + force_angle);
			}
			break;
	}

	current3ph_run(&curse);
	hall_qubic_interp(&hall_extra);
}


void burst_sw_begin(void){
	hall_begin(&hall, &burst_config.hall);
	adc_begin(&adc, &burst_config.adc);
	inv3ph_begin(&inverter, &burst_config.inv3ph);
	current3ph_begin(&curse, &burst_config.current3ph, &inverter, adc.values );
	nikitin_begin(&speedse,&burst_config.speedse);
	power_status = clch_ps_status_off;
	hall_extra.hall.pactual = &hall.angle;
	hall_extra.lost = burst_true;
	lat_current_pi.setup(
	&burst_config.lat_current_pi 				//burst_pi_config_p _config
	,	&requried.current.lateral 	//burst_signal_p				_signal_req
	, &curse.dq.lateral 					//burst_signal_p				_signal
	, &inverter.dq.lateral				//burst_signal_p			  _control
	, 0														//burst_signal_t 				_start_control
	, &power_satstate							//burst_satstate_t *		_master_sut_flag
	, 0														//burst_signal_p				_controlMax
	, 0														//burst_signal_p				_controlMin
	, 0														//burst_signal_p				_signal_diff
	, 0														//burst_signal_p				_signal_force
	);
}
void burst_sw_start(void){
	adc_start();
	while( adc.ready == burst_false ){
		BURST_NOP();
	}

}

void burst_sw_realtime_loop(void){	
	power_run();
	fm_recorder();
}

void burst_sw_backend_loop(void){	
	power_run();
	fm_recorder();
	/*if(power_command == clch_ps_command_on){
		power_command = clch_ps_command_off;
	}*/
	//inv3ph_test();
	machine();
	fm_poll();
}
void burst_sw_frontend_loop(void){	
	hall_regres_poll(&hall_extra);
}
void burst_sw_slot_0(void){
	speedse.ref.run();
	hall.delta_acc = 0;
}


#endif

