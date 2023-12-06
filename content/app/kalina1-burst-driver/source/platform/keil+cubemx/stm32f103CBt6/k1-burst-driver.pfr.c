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



void burst_hw_realtime_loop(void){
}



void TIM1_CC_IRQHandler(void)
{
	hall_pins.C = (HALL1_GPIO_Port->IDR & HALL1_Pin) != 0;
	hall_pins.B = (HALL2_GPIO_Port->IDR & HALL2_Pin) != 0;
	hall_pins.A = (HALL3_GPIO_Port->IDR & HALL3_Pin) != 0;
	hall_update(&hall,&hall_pins);
	//swt_pwm_run_forward();

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
	#if BURST_PANICS_BOARD_VOLTAGE_ENABLED == 1
	board.voltage.raw = 
	#endif 
	adc_raw[5] = hadc2.Instance->JDR2;
	#if BURST_PANICS_BOARD_CURRENT_ENABLED == 1
	board.current.raw = 
	#endif 
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
#define swm_pwm_Z -10
int16_t swt_pwm_A = swm_pwm_Z;
int16_t swt_pwm_B = swm_pwm_Z;
int16_t swt_pwm_C = swm_pwm_Z;

void swt_phy_A_set_pwm(uint16_t _pwm){
	swt_pwm_A = (int16_t)_pwm;
	TIM1->CCR3 = _pwm;
}
void swt_phy_B_set_pwm(uint16_t _pwm){
	swt_pwm_B = (int16_t)_pwm;
	TIM1->CCR2 = _pwm;
}
void swt_phy_C_set_pwm(uint16_t _pwm){
	swt_pwm_C = (int16_t)_pwm;
	TIM1->CCR1 = _pwm;	
}
void swt_phy_A_set_lo(void){
	swt_pwm_A = 0;
	TIM1->CCR3 = 0;
	TIM1->CCER |= 0x1500;
}
void swt_phy_B_set_lo(void){
	swt_pwm_B = 0;
	TIM1->CCR2 = 0;
	TIM1->CCER |= 0x1050;
}
void swt_phy_C_set_lo(void){
	swt_pwm_C = 0;
	TIM1->CCR1 = 0;	
	TIM1->CCER |= 0x1005;
}

void swt_phy_A_off(void){
	swt_pwm_A = swm_pwm_Z;
	TIM1->CCER &= ~0x500;
}
void swt_phy_B_off(void){
	swt_pwm_B = swm_pwm_Z;
	TIM1->CCER &= ~0x050;
}
void swt_phy_C_off(void){
	swt_pwm_C = swm_pwm_Z;
	TIM1->CCER &= ~0x005;
}

void swt_phy_A_on(uint16_t _pwm){
	swt_pwm_A = (int16_t)_pwm;
	TIM1->CCR3 = _pwm;
	TIM1->CCER |= 0x1500;
}
void swt_phy_B_on(uint16_t _pwm){
	swt_pwm_B = (int16_t)_pwm;
	TIM1->CCR2 = _pwm;
	TIM1->CCER |= 0x1050;
}
void swt_phy_C_on(uint16_t _pwm){
	swt_pwm_C = (int16_t)_pwm;
	TIM1->CCR1 = _pwm;
	TIM1->CCER |= 0x1005;
}
/*
uint8_t swt_phy_sector_get(void){
	return hall.sector;
}
*/


#endif
