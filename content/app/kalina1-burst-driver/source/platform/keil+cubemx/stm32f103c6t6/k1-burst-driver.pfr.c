#include "k1-burst-driver.h"
//#include "k1-burst-driver.h"
#include "tim.h"
#include "usart.h"
#include "burst/burst.h"
#include "adc.h"
#include "burst/burst_app.h"
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
	SD_GPIO_Port->BSRR = SD_Pin;
	
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

void burst_hw_start(void){
	TIM1->ARR = K1_PWM_MODULO;
	TIM1->CCR4 = K1_PWM_MODULO-K1_ADC_DELAY;
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);		
	__enable_irq();	
}
hall_pins_t hall_pins;

void burst_hw_realtime_loop(void){
	hall_pins.A = (HALL1_GPIO_Port->IDR & HALL1_Pin) != 0;
	hall_pins.B = (HALL2_GPIO_Port->IDR & HALL2_Pin) != 0;
	hall_pins.C = (HALL3_GPIO_Port->IDR & HALL3_Pin) != 0;
	hall_update(&hall,&hall_pins);
}

void TIM1_CC_IRQHandler(void)
{
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	HAL_ADCEx_InjectedStart(&hadc2);
}
uint32_t adc_raw[BURST_ADC_CHANNEL_COUNT]={};
void ADC1_2_IRQHandler(void)
{
	__HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);
	//__HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOS);
	 adc_raw[0] = hadc1.Instance->JDR1;
	 adc_raw[1] = hadc1.Instance->JDR2;
	 adc_raw[2] = hadc1.Instance->JDR3;
	 adc_raw[3] = hadc2.Instance->JDR1;
	 adc_raw[4] = hadc2.Instance->JDR2;

	adc_update(&adc,adc_raw);
	burst_realtime_loop();
	burst_backend_loop();
}

//==============TP==================================
void burst_tp_phy_on(void){
	TP1_GPIO_Port->BSRR = (uint32_t)TP1_Pin;
}
void burst_tp_phy_off(void){
	TP1_GPIO_Port->BRR = TP1_Pin;
}

uint8_t	fm_available(void){
	//USART1->SR =(UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF );
	return (USART1->SR & UART_FLAG_RXNE) == UART_FLAG_RXNE ? 1:0; 
}
uint8_t	fm_space(void){
	return (USART1->SR & UART_FLAG_TXE) == UART_FLAG_TXE?1:0; 
}
uint8_t	fm_get(void){
	return USART1->DR; 
}
void		fm_put(uint8_t _data){
	USART1->DR = _data;
}

void power_boot_begin(void){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCER = 0x1555;
	SD_GPIO_Port->BRR = SD_Pin;
}
burst_satstate_t power_do_invert(void){
	TIM1->CCR1 = motor.inverter.duty.C;
	TIM1->CCR2 = motor.inverter.duty.B;
	TIM1->CCR3 = motor.inverter.duty.A;
	return burst_satstate_none;
}

void power_shutdown_begin(void){
	TIM1->CCER = 0x1000;
	SD_GPIO_Port->BSRR = SD_Pin;
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
