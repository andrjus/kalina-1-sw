#include "k1-burst-driver.h"
#ifdef K1_TAG_BOARD_BETA
//#include "k1-burst-driver.h"
#include "tim.h"
#include "i2c.h"
#include "usart.h"
#include "burst/burst.h"
#include "adc.h"
#include "burst/burst_app.h"
#include "burst/burst_timer.h"



#define K1_TICK_PER_MKS ( K1_CPU_FREQ_HZ / 1000000 )
volatile unsigned int *DWT_CYCCNT = (volatile unsigned int *)0xE0001004; //address of the register
volatile unsigned int *DWT_CONTROL = (volatile unsigned int *)0xE0001000; //address of the register
volatile unsigned int *SCB_DEMCR = (volatile unsigned int *)0xE000EDFC; //address of the register 
void delay_us(volatile uint32_t _us){
	volatile unsigned int start, current; 
	start = *DWT_CYCCNT;
	_us *=K1_TICK_PER_MKS;
	do{
		current = *DWT_CYCCNT;
	} while((current - start) < _us);
}
void delay_ns(volatile uint64_t _ns){
	volatile uint64_t start, current; 
	start = *DWT_CYCCNT;
	do{
		current = *DWT_CYCCNT;
	} while((current - start)*1000 < _ns*K1_TICK_PER_MKS);
}

void burst_hw_begin(void){
	HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	
	debug_set_verb(VERB_LOOP);
	
	*SCB_DEMCR = *SCB_DEMCR | 0x01000000; 
	*DWT_CYCCNT = 0; // reset the counter 
	*DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter		
	delay_us(500000);
	
}

void adc_start(void){
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

burst_bool_t temp_alarm(uint8_t _addr, uint8_t _value){
	uint8_t tmp;
	for( int i=0;i<6;++i){
		//SW_I2C_Read_8addr( &k1_sw_i2c, K1_TM423ADDR, _addr, &tmp, 1);
		if(tmp == _value){
			return burst_true;
		} 		
		delay_us(1000);
	}
	return burst_false;
}

void burst_hw_start(void){
	TIM1->ARR = K1_PWM_MODULO;
	TIM1->CCR4 = K1_PWM_MODULO-K1_ADC_DELAY;
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);		
	
//	HAL_TIMEx_HallSensor_Start_IT(&htim3);
	
	__enable_irq();	
	k1_serial_start_receive();	
	#if TMP423_ENABLED == 1
	TMP423_start();
	#endif
	//burst_alarm( temp_alarm(0x0A,0x7C));
	//burst_alarm( temp_alarm(0xFE,0x55));

}

burst_bool_t serialComDone = burst_true;
void burst_hw_frontend_loop(void){
/*	if(serialComDone == burst_true){
		static const char tmp[] = "Hello world!\n\r";
		k1_serial_send_packet((uint8_t *)tmp,14);
	}
	*/
}

hall_pins_t hall_pins;

uint8_t swt_enable_prev = 0;
uint8_t swt_enable = 0;
uint16_t swt_value = 0;

typedef struct {
	struct{
		int prop;
		int model;
	} gain;	
}swt_config_t;

typedef struct {
	int error;
	int model;
	int model32;
	int pwm;
}swt_t;

int swt_pwm = 899;
int swt_cur_req = 100;

//int swt_model_reset2(swt_config_t * _conf,  int _pwm1, int _pwm2 ){
	//return (((_pwm1 + _pwm2) >> 1) *_conf->gain.iprop10)>>10 ;
//}

//int swt_model_reset(swt_config_t * _conf,  int _pwm){
	//return ( _pwm *_conf->gain.iprop10 ) >> 10 ;
//}

void swt_reset(swt_t * _swt, int _pwm){
	_swt->pwm = _pwm;
	_swt->model = _pwm<<7;
	_swt->model32 = _swt->model << 10;
}

void swt_run(swt_t * _swt, swt_config_t * _conf,int _requried,  int _actual){
	int error = _requried - _actual;
	int model;
	if ( !( (error>0 && _swt->pwm == swt_pwm ) || (error<0 && _swt->pwm == 0))){
		int model32 = _swt->model32 + error * _conf->gain.model;
		model = model32 >> 10;
		_swt->model = model;
		_swt->model32 = model32;
	} else{
		model = _swt->model32 >> 10;
	}
	int pwm = ((error  - _actual) * _conf->gain.prop + model ) >> 7;
	if( pwm<0) {
		pwm = 0;
	} if ( pwm > swt_pwm) {
		pwm = swt_pwm;
	}
	_swt->pwm = pwm;	
	_swt->error = error;
}
swt_t swtA={};
swt_t swtB={};
swt_t swtC={};
swt_config_t swt_cfg={
	{
		15
		,2000
	}
};
//motor.sensor.abc.A
int swt_pwmA = 0;
int swt_pwmB = 0;
int swt_pwmC = 0;

int swm_sector_offset = 0;
void swt_loop(void){
	if(motor.cross.ac.ref.panic){
		swt_enable = 0;
	}
	
	if(swt_enable){
		int sector = hall.sector+swm_sector_offset;
		if (sector >=6){
			sector = sector-6;
		}
		static int  sector_prev = -1;
		int sectror_change = sector != sector_prev ;
		sector_prev = sector;
		
		if(swt_enable == 1 ){
			switch(sector){
				case 0: //A
					TIM1->CCER = 0x1055;
					swt_pwmA = 0;
					if(sectror_change){
						swt_reset(&swtB,swtA.pwm>>1);
					}
					swt_run(&swtB,&swt_cfg, swt_cur_req, motor.sensor.abc.B);
					swt_pwmB = swtB.pwm;
					swt_pwmC = 0;
					break;
				case 1://-C
					TIM1->CCER = 0x1550;
					if(sectror_change){
						swt_reset(&swtB,swtB.pwm>>1);
					}
					swt_pwmA = 0;
					swt_run(&swtB,&swt_cfg, swt_cur_req, motor.sensor.abc.B);
					swt_pwmB = swtB.pwm;
					swt_pwmC = 0;
					break;
				case 2://B
					TIM1->CCER = 0x1505;
					swt_pwmA = 0;
					swt_pwmB = 0;
					if(sectror_change){
						swt_reset(&swtC,swtB.pwm>>1);
					}
					swt_run(&swtC,&swt_cfg, swt_cur_req, motor.sensor.abc.C);
					swt_pwmC = swtC.pwm;
					break;
				case 3://-A
					TIM1->CCER = 0x1055;
					swt_pwmA = 0;
					swt_pwmB = 0;
					if(sectror_change){
						swt_reset(&swtC,swtC.pwm>>1);
					}
					swt_run(&swtC,&swt_cfg, swt_cur_req, motor.sensor.abc.C);
					swt_pwmC = swtC.pwm;
					break;
				case 4://C
					TIM1->CCER = 0x1550;
					if(sectror_change){
						swt_reset(&swtA,swtC.pwm>>1);
					}
					swt_run(&swtA,&swt_cfg, swt_cur_req, motor.sensor.abc.A);
					swt_pwmA = swtA.pwm;
					swt_pwmB = 0;
					swt_pwmC = 0;
					break;
				case 5: //-B
					TIM1->CCER = 0x1505;
					if(sectror_change){
						swt_reset(&swtA,swtA.pwm>>1);
					}
					swt_run(&swtA,&swt_cfg, swt_cur_req, motor.sensor.abc.A);
					swt_pwmA = swtA.pwm;
					//swtB.pwm = swtC.pwm 
					swt_pwmB = 0;
					swt_pwmC = 0;

					break;
			}
		}
		
	if(swt_enable == 2 ){
			switch(sector){
				case 0: //A
					TIM1->CCER = 0x1055;
					swt_pwmA = swt_pwm>>1;
					swt_pwmB = swt_pwm;
					swt_pwmC = 0;
					break;
				case 1://-C
					TIM1->CCER = 0x1550;
					swt_pwmA = 0;
					swt_pwmB = swt_pwm;
					swt_pwmC = swt_pwm>>1;
					break;
				case 2://B
					TIM1->CCER = 0x1505;
					swt_pwmA = 0;
					swt_pwmB = swt_pwm>>1;
					swt_pwmC = swt_pwm;
					break;
				case 3://-A
					TIM1->CCER = 0x1055;
					swt_pwmA = swt_pwm>>1;
					swt_pwmB = 0;
					swt_pwmC =  swt_pwm;
					break;
				case 4://C
					TIM1->CCER = 0x1550;
					swt_pwmA = swt_pwm;
					swt_pwmB = 0;
					swt_pwmC = swt_pwm>>1;
					break;
				case 5: //-B
					TIM1->CCER = 0x1505;
					swt_pwmA = swt_pwm;
					swt_pwmB = swt_pwm>>1;
					swt_pwmC = 0;

					break;
			}
		}
		
		TIM1->CCR3 = swt_pwmA;
		TIM1->CCR2 = swt_pwmB;
		TIM1->CCR1 = swt_pwmC;
		if(swt_enable_prev == 0){
			//TIM1->CCER = 0x1555;
			//SD_GPIO_Port->BRR = SD_Pin;
		}

	} else{
		if(swt_enable_prev){
			//SD_GPIO_Port->BSRR = SD_Pin;
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
			TIM1->CCER = 0x1000;
			swt_reset(&swtA,0);
			swt_reset(&swtB,0);
			swt_reset(&swtC,0);
		}
	}
	swt_enable_prev = swt_enable;
}


void burst_hw_realtime_loop(void){
}



void TIM1_CC_IRQHandler(void)
{
	hall_pins.C = (HALL1_GPIO_Port->IDR & HALL1_Pin) != 0;
	hall_pins.B = (HALL2_GPIO_Port->IDR & HALL2_Pin) != 0;
	hall_pins.A = (HALL3_GPIO_Port->IDR & HALL3_Pin) != 0;
	hall_update(&hall,&hall_pins);
	swt_loop();

	static int presk_tick=0;
	presk_tick++;
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
	if( presk_tick == K1_PWM_TIMER_PRESC ){
		HAL_ADCEx_InjectedStart_IT(&hadc1);
		HAL_ADCEx_InjectedStart(&hadc2);
		presk_tick = 0;
	}
}
uint32_t adc_raw[BURST_ADC_CHANNEL_COUNT]={};
void ADC1_2_IRQHandler(void)
{
	__HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);
	//__HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOS);
	 adc_raw[0] = hadc1.Instance->JDR1;
	 adc_raw[1] = hadc1.Instance->JDR2;
	 adc_raw[2] = hadc1.Instance->JDR3;
	 adc_raw[3] = hadc1.Instance->JDR4;
	 adc_raw[4] = hadc2.Instance->JDR1;
	 adc_raw[5] = hadc2.Instance->JDR2;
	 adc_raw[6] = hadc2.Instance->JDR3;

	adc_update(&adc,adc_raw);
	burst_realtime_loop();
	burst_backend_loop();
}

//==============TP==================================
void burst_tp_phy_on(void){
	TEST0_GPIO_Port->BSRR = (uint32_t)TEST0_Pin;
}
void burst_tp_phy_off(void){
	TEST0_GPIO_Port->BRR = TEST0_Pin;
}


uint8_t k1_serial_rx_buffer[K1_SERIAL_SIZE];


extern DMA_HandleTypeDef hdma_usart1_rx;

burst_bool_t k1_serial_ready(void){
	return serialComDone;
}
void k1_serial_start_receive(void){
	HAL_UART_DMAStop(&huart1);
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, k1_serial_rx_buffer, K1_SERIAL_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	//HAL_NVIC_DisableIRQ(USART1_IRQn);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
	if (huart->Instance == USART1) {
		serialComDone = burst_true;
		k1_serial_send_complete();
		k1_serial_start_receive();
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart){
	if (huart->Instance == USART1)
	{
		if(serialComDone == burst_false) {
			serialComDone = burst_true;
			k1_serial_send_refuse();
		}
		k1_serial_start_receive();
	}
}
int abort_count = 0;
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART1)
	{
		abort_count++;
		serialComDone = burst_true;
		k1_serial_send_refuse();
	}
}
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART1)
	{
		abort_count++;
		serialComDone = burst_true;
		k1_serial_send_refuse();
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART1)
	{
		k1_serial_receive_packet(k1_serial_rx_buffer,Size);
		k1_serial_start_receive();
	}
}

void k1_serial_send_packet(uint8_t * _data, uint8_t _sz){
	//HAL_NVIC_EnableIRQ(USART1_IRQn);
	HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_SET);
	if( HAL_UART_Transmit_DMA(&huart1,_data,_sz) != HAL_OK ) {
		HAL_GPIO_WritePin(DE_GPIO_Port, DE_Pin, GPIO_PIN_RESET);
		serialComDone = burst_true;
		k1_serial_send_refuse();
	} else {
		serialComDone = burst_false;
	}
}

void k1_serial_aborttx(void){
	HAL_UART_AbortTransmit(&huart1);
	serialComDone = burst_true;
	k1_serial_send_refuse();
}


void power_boot_begin(void){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCER = 0x1555;
}
burst_satstate_t power_do_invert(void){
	TIM1->CCR1 = motor.inverter.duty.C;
	TIM1->CCR2 = motor.inverter.duty.B;
	TIM1->CCR3 = motor.inverter.duty.A;
	return burst_satstate_none;
}

void power_shutdown_begin(void){
	TIM1->CCER = 0x1000;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;	
}

void burst_hw_on_crash(void){
	__disable_irq();
	power_shutdown_begin();	
}

int critical_ = 0;	
burst_guard_op_t burst_hw_critical_enter(void){
	critical_++;
	return  burst_guard_op_run;
}

void burst_hw_critical_leave(void){
	critical_ --;
	if(critical_<=0){
	}
}

burst_guard_op_t burst_hw_guard_enter(void){
		uint32_t prim = __get_PRIMASK();
	__disable_irq();
	return prim == 0? burst_guard_op_run : burst_guard_op_skip;	
}

void burst_hw_guard_leave(void){
	__enable_irq();
}

burst_guard_op_t burst_hw_guard_lock(void){
		uint32_t prim = __get_PRIMASK();
	__disable_irq();
	return prim == 0? burst_guard_op_run : burst_guard_op_skip;	
}
void burst_hw_guard_unlock(void){
	__enable_irq();
}


void k1_update_temp(void){
	
}
/*
uint32_t  hall_period1 = 0;
uint32_t  hall_period2 = 0;
uint32_t  hall_period3 = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3){
		hall_period1 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_1);
		hall_period2 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_2);
		hall_period3 = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);
	}
}
*/
#if TMP423_ENABLED == 1
void TMP423_prf_send_cancel(void){
	HAL_I2C_Master_Abort_IT(&hi2c1,0);
}
void TMP423_prf_receive_cancel(void){
	HAL_I2C_Master_Abort_IT(&hi2c1,0);
}
void TMP423_prf_start_send(TMP423_packet_p _p){
	HAL_I2C_Master_Transmit_IT  (&hi2c1,K1_TM423ADDR,_p->data,_p->size); 
}

void TMP423_prf_start_receive(TMP423_packet_p _p){
	HAL_I2C_Master_Receive_IT (&hi2c1,K1_TM423ADDR,_p->data,_p->size); 
}

burst_time_us_t TMP423_prf_wd_us(TMP423_packet_p _p){
	BURST_UNUSED(_p);
	return 100;
}

burst_bool_t TMP423_prf_ready(void){
	return hi2c1.State == HAL_I2C_STATE_READY ? burst_true : burst_false;
}

void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	TMP423_confirm();
}
void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
	TMP423_confirm();
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef * hi2c){
	 TMP423_refuse();
}
#endif

#endif
