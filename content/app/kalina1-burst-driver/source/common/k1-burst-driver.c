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
temper_t temper;
voltage_t voltage;
#if TMP423_ENABLED == 1

uint8_t tables[13] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
uint8_t addr[13] = {0x00,0x01,0x02,0x03,0x08,0x09,0xA,0x0B,0x10,0x11,0x12,0x13,0xFE};


int ix = 13;


void TMP423_confirm_callback(burst_bool_t _r){
	BURST_UNUSED(_r);
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

void burst_sw_start(void){	
	adc_start();
	while( adc.ready == burst_false ){
		BURST_NOP();
	}	
	pmsm_hall_app_start();
	#if TMP423_ENABLED == 1
	TMP423_start();
	burst_timer_start(&temp_poll);
	#endif
	burst_sqrt_init();

}

void burst_sw_realtime_loop(void){	
	pmsm_hall_app_realtime_loop();
	enco.ref.run();
}
static uint32_t sqrt_test_x = 0;
static volatile uint16_t sqrt_test_y = 0;
static uint16_t sqrt_test_y0 = 0;
void burst_sw_backend_loop(void){		
	pmsm_hall_app_backend_loop();
	fm_recorder();
	fm_poll();
//	machine();
	sqrt_test_y0++;
	if(sqrt_test_y0>4095){
		sqrt_test_y0 = 0;
	}
	debug_tp_on(112);
	sqrt_test_x = sqrt_test_y0*sqrt_test_y0;
	sqrt_test_y = burst_sqrt(sqrt_test_x);
	debug_tp_off(112);
	
}

int synchro_test_enable = 0;
int synchro_test_level = 10000000;
void burst_sw_frontend_loop(void){
	pmsm_hall_app_frontend_loop();
	k1_serial_pool();
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
	
}

void burst_sw_slot_0(void){
	pmsm_hall_app_control_step_1();
}

void burst_sw_slot_1(void){
	pmsm_hall_app_control_step_2();
}
void burst_sw_slot_2(void){
	pmsm_hall_app_control_step_3();
}

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
	return fmincom_get();
}


void fm_put(uint8_t _data){
	fmoutcom_put(_data);
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
uint32_t serial_tx_sz=0;
void fmserial_try_send(void){
	serial_tx_sz = fmoutcom_buf_get(k1_serial_tx_buffer,K1_SERIAL_SIZE);
	if(serial_tx_sz){
		sending=burst_true;
		k1_serial_send_packet( k1_serial_tx_buffer , serial_tx_sz );			
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
		if( d> serial_tx_sz*3400 ){
			k1_serial_aborttx();
			tm = now;
		}
	}
}

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

