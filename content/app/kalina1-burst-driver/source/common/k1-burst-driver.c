#include "k1-burst-driver.h"

typedef struct config_s {
	inv3ph_config_t inv3ph;
	current3ph_config_t current3ph;
	hall_config_t hall;
	adc_config_t adc;
	nikitin_config_t speedse;
	burst_pi_config_t lat_current_pi;
} config_t;
config_t burst_config = {
	{K1PWM_MODULO-K1ADC_DELAY,0} //inv3ph_config_t
	,{ // current3ph_config_t
		{0,1,4} //int adc_index[3];
		,{ 
			75358, -30028 ,-10646, 
			-33930, 71247, -11748, 
			-41428, -41219, 22393
		}	//burst_signal_t deform[9];
	}
	,{ //hall_config_s
		{
			BURST_SIGNAL_T(175./180)//burst_signal_t native;
			,0// -BURST_SIGNAL_T(30./180.)//burst_signal_t dynamic;
		} //offset;
		, burst_true//burst_bool_t inv;
	}
	,{ //adc_config_s
		{0,1,2,3,4} // index[BURST_ADC_CHANNEL_COUNT ];
		,{1,1,1,1,1} //burst_signal_t scale[BURST_ADC_CHANNEL_COUNT ];
		,10 //unsigned init_count_shift;
	}
	,{ //nikitin_config_t
		{}
		, 5//int8_t shift;
		, 5//int8_t presc_shift;
		, 0//int8_t value_shift;
	}
	,{ //burst_filter_config_t
		250//burst_parametr_t	propGain;
		,650//burst_parametr_t	modelGain;
		,0//burst_parametr_t	diffGain;
		,0//burst_parametr_t	forceGain;
		,7//uint8_t						controlShift;
		,10//uint8_t						modelShift;
	}
};


inv3ph_t inverter={};
hall_t hall={};
adc_t adc={};
current3ph_t curse={};
static volatile burst_signal_t  force_angle = 0;
static volatile burst_long_signal_t  voltage_max = 0;
static volatile burst_long_signal_t  voltage_min = 0;
static volatile burst_long_signal_t  eds = 0;

struct{
	int mode;
	dq_t voltage;
	dq_t current;
	burst_long_signal_t freq;
} requried = {};

struct{
	int mode;
	burst_signal_t angle;
	burst_long_signal_t angle32;
} actual = {};

hall_extra_t hall_extra ={};
	
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
			inverter.dq.lateral = requried.voltage.lateral;
			inverter.dq.cross = requried.voltage.cross;
			inv3ph_run(&inverter,actual.angle);
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
				inv3ph_run(&inverter,hall_extra.angle32>>16);
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
				inv3ph_run(&inverter,(hall_extra.angle32>>16) + force_angle);
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
	lat_current_pi.begin(
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

