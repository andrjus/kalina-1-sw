#include "k1-burst-driver.h"
#ifdef K1_TAG_BOARD_KIPARIS_2
//#include "k1-burst-driver.h"
#include "tim.h"
#include "usart.h"
#include "burst/burst.h"
#include "adc.h"
#include "burst/burst_app.h"
#include "burst/burst_timer.h"
#define ON(x) x##_GPIO_Port->BSRR = (uint32_t)x##_Pin;
#define OFF(x) x##_GPIO_Port->BSRR = (uint32_t)x##_Pin << 16u;

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
	
	debug_set_verb(111);	
	//debug_set_verb(VERB_REALTIME);
	*SCB_DEMCR = *SCB_DEMCR | 0x01000000; 
	*DWT_CYCCNT = 0; // reset the counter 
	*DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter		
	delay_us(500000);
	
}

void adc_start(void){
	HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
}

void burst_hw_start(void){
	TIM1->ARR = K1_PWM_MODULO;
	TIM1->CCR4 = K1_PWM_MODULO-K1_ADC_DELAY;
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);		
//	HAL_TIMEx_HallSensor_Start_IT(&htim3);
	
	__enable_irq();	
	k1_serial_start_receive();	

}

burst_bool_t serialComDone = burst_true;
void burst_hw_frontend_loop(void){
	temper.dg.board = K1_BOARD_TEMPER_PP_TO_GRAD(temper.raw.board);
}

hall_pins_t hall_pins;

void burst_hw_realtime_loop(void){
}

void TIM1_CC_IRQHandler(void)
{
	static int presk_tick=0;
	presk_tick++;
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
	if( presk_tick == K1_PWM_TIMER_PRESC ){
		debug_tp_on(111);
		//HAL_ADCEx_InjectedStart_IT(&hadc1);
		hadc3.Instance->CR2=(0x401 | ADC_CR2_JSWSTART);
		hadc2.Instance->CR2=(0x401 | ADC_CR2_JSWSTART);
		hadc1.Instance->CR1=0x180;
		hadc1.Instance->CR2=(0x401 | ADC_CR2_JSWSTART);
		//HAL_ADCEx_InjectedStart(&hadc2);
		//HAL_ADCEx_InjectedStart(&hadc3);
		presk_tick = 0;
	}
	hall_pins.A = (HALLA_GPIO_Port->IDR & HALLA_Pin) != 0;
	hall_pins.B = (HALLB_GPIO_Port->IDR & HALLB_Pin) != 0;
	hall_pins.C = (HALLC_GPIO_Port->IDR & HALLC_Pin) != 0;
	hall_update(&hall,&hall_pins);
}
uint32_t adc_raw[BURST_ADC_CHANNEL_COUNT]={};

uint32_t vref_raw = 0;
void ADC_IRQHandler(void)
{
	debug_tp_off(111);
	__HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);
	//__HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOS);
	 adc_raw[1] = hadc1.Instance->JDR1;//B
	 temper.raw.board = hadc1.Instance->JDR2;
	 vref_raw = hadc1.Instance->JDR3;
	 //adc_raw[2] = hadc2.Instance->JDR1;//C
	 voltage.raw = hadc2.Instance->JDR2;
	 adc_raw[0] = hadc3.Instance->JDR1;//A
	 adc_raw[2] = 2048*3 - adc_raw[1] - adc_raw[0];
	 

	adc_update(&adc,adc_raw);
	burst_realtime_loop();
	burst_backend_loop();
	
}


//==============TP==================================
void burst_tp_phy_on(void){
	ON(TEST1);
}
void burst_tp_phy_off(void){
	OFF(TEST1);
}


uint8_t k1_serial_rx_buffer[K1_SERIAL_SIZE];


extern DMA_HandleTypeDef hdma_usart3_rx;

burst_bool_t k1_serial_ready(void){
	return serialComDone;
}
void k1_serial_start_receive(void){
	HAL_UART_DMAStop(&huart3);
	OFF(DE);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, k1_serial_rx_buffer, K1_SERIAL_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
	if (huart->Instance == USART3) {
		serialComDone = burst_true;
		k1_serial_send_complete();
		k1_serial_start_receive();
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart){
	if (huart->Instance == USART3)
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
	if (huart->Instance == USART3)
	{
		abort_count++;
		serialComDone = burst_true;
		k1_serial_send_refuse();
	}
}
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart){
	if (huart->Instance == USART3)
	{
		abort_count++;
		serialComDone = burst_true;
		k1_serial_send_refuse();
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == USART3)
	{
		k1_serial_receive_packet(k1_serial_rx_buffer,Size);
		k1_serial_start_receive();
		#if BURST_PANICS_MASTER_LOST_ENABLED == 1
		burst_master_alive(&motor.cross.ac.ref);
		#endif
	}
}

void k1_serial_send_packet(uint8_t * _data, uint8_t _sz){
	ON(DE);
	if( HAL_UART_Transmit_DMA(&huart3,_data,_sz) != HAL_OK ) {
		OFF(DE);
		serialComDone = burst_true;
		k1_serial_send_refuse();
	} else {
		serialComDone = burst_false;
	}
}

void k1_serial_aborttx(void){
	HAL_UART_AbortTransmit(&huart3);
	serialComDone = burst_true;
	k1_serial_send_refuse();
}


void power_boot_begin(void){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	
	TIM1->CCER = 0x1555;
						/*HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
					HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
					HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
					HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
					HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
					HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);*/
}
burst_satstate_t power_do_invert(void){
	TIM1->CCR2 = motor.inverter.duty.A;
	TIM1->CCR1 = motor.inverter.duty.B;
	TIM1->CCR3 = motor.inverter.duty.C;
	return burst_satstate_none;
}

void power_shutdown_begin(void){
	TIM1->CCER = 0x1000;
	//					HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
		//			HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
			//		HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
				//	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2);
					//HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
					//HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_3);

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


void swt_phy_A_set_pwm(uint16_t _pwm){
	TIM1->CCR2 = _pwm;
}
void swt_phy_B_set_pwm(uint16_t _pwm){
	TIM1->CCR1 = _pwm;
}
void swt_phy_C_set_pwm(uint16_t _pwm){
	TIM1->CCR3 = _pwm;	
}
void swt_phy_A_set_lo(void){
	TIM1->CCR2 = 0;
	TIM1->CCER |= 0x1050;
}
void swt_phy_B_set_lo(void){
	TIM1->CCR1 = 0;
	TIM1->CCER |= 0x1005;
}
void swt_phy_C_set_lo(void){
	TIM1->CCR3 = 0;	
	TIM1->CCER |= 0x1500;
}

void swt_phy_A_off(void){
	TIM1->CCR2 = 0;	
	TIM1->CCER &= ~0x050;
}
void swt_phy_B_off(void){
	TIM1->CCR1 = 0;	
	TIM1->CCER &= ~0x005;
}
void swt_phy_C_off(void){
	TIM1->CCR3 = 0;	
	TIM1->CCER &= ~0x500;
}

void swt_phy_A_on(uint16_t _pwm){
	TIM1->CCR2 = _pwm;
	TIM1->CCER |= 0x1050;
}
void swt_phy_B_on(uint16_t _pwm){
	TIM1->CCR1 = _pwm;
	TIM1->CCER |= 0x1005;
}
void swt_phy_C_on(uint16_t _pwm){
	TIM1->CCR3 = _pwm;
	TIM1->CCER |= 0x1500;
}

uint8_t swt_phy_sector_get(void){
	return hall.sector;
}

int burst_board_temper_get_pp(void){
	return temper.raw.board;
}

int burst_board_voltage_get_pp(void){
	return voltage.raw;
}
#endif
