#include "k1-burst-driver.h"

pmsm_hall_app_config_t  k1_config = PMSM_HALL_APP_CONFIG();

void burst_sw_begin(void){
	pmsm_hall_app_begin(&k1_config);
}



void burst_sw_start(void){
	adc_start();
	while( adc.ready == burst_false ){
		BURST_NOP();
	}	
	pmsm_hall_app_start();
}

void burst_sw_realtime_loop(void){	
	pmsm_hall_app_realtime_loop();
}

void burst_sw_backend_loop(void){		
	pmsm_hall_app_backend_loop();
	fm_recorder();
	fm_poll();
//	machine();
}
void burst_sw_frontend_loop(void){
	pmsm_hall_app_frontend_loop();
}
void burst_sw_slot_0(void){
	pmsm_hall_app_control_step_1();
}
static int presc = 0;
void burst_sw_slot_1(void){
	pmsm_hall_app_control_step_2();
}
void burst_sw_slot_2(void){
	pmsm_hall_app_control_step_3();
}



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
