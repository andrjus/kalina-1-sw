#include "k1-burst-driver.h"
//#include "k1-burst-driver.h"
#include "tim.h"
#include "usart.h"
#include "burst/burst.h"
#include "adc.h"
#include "burst/burst_app.h"
#include "sw_i2c.h"

extern sw_i2c_t  k1_sw_i2c;

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

burst_bool_t temp_alarm(uint8_t _addr, uint8_t _value){
	uint8_t tmp;
	for( int i=0;i<6;++i){
		SW_I2C_Read_8addr( &k1_sw_i2c, K1_TM423ADDR, _addr, &tmp, 1);
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
	__enable_irq();	
	k1_serial_start_receive();	

	burst_alarm( temp_alarm(0x0A,0x7C));
	burst_alarm( temp_alarm(0xFE,0x55));

}

uint8_t tables[8];
uint8_t addr[8] = {0x00,0x01,0x02,0x03,0x08,0x09,0x0B,0x10};

void burst_hw_frontend_loop(void){
	/*static uint8_t dummy = 0;
	static uint8_t ix = 0;
	SW_I2C_Read_8addr( &k1_sw_i2c, K1_TM423ADDR, addr[ix], tables+ix, 1);
	ix++;
	if(ix==8){
		ix = 0;
	}*/
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


uint8_t k1_serial_rx_buffer[K1_SERIAL_SIZE];

burst_bool_t serialComDone = burst_true;
extern DMA_HandleTypeDef hdma_usart1_rx;

burst_bool_t k1_serial_ready(void){
	return serialComDone;
}
void k1_serial_start_receive(void){
	HAL_UART_DMAStop(&huart1);
	HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
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
	HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
	if( HAL_UART_Transmit_DMA(&huart1,_data,_sz) != HAL_OK ) {
		HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
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
	


static void sda_in_mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Pin = SDA_Pin;
    HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);
}


static void sda_out_mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Pin = SDA_Pin;
    HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);
}


static void scl_in_mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Pin = SCL_Pin;
    HAL_GPIO_Init(SCL_GPIO_Port, &GPIO_InitStruct);
}


static void scl_out_mode(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    GPIO_InitStruct.Pin = SCL_Pin;
    HAL_GPIO_Init(SCL_GPIO_Port, &GPIO_InitStruct);
}

static int sw_i2c_port_initial(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    // i2c_sw SCL
    GPIO_InitStruct.Pin = SCL_Pin;
    HAL_GPIO_Init(SDA_GPIO_Port, &GPIO_InitStruct);
    // i2c_sw SDA
    GPIO_InitStruct.Pin = SDA_Pin;
    HAL_GPIO_Init(SCL_GPIO_Port, &GPIO_InitStruct);
    return 0;
}


static int sw_i2c_port_io_ctl(hal_io_opt_e opt, void *param)
{
    int ret = -1;
    switch (opt)
    {
    case HAL_IO_OPT_SET_SDA_HIGH:
        HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin,GPIO_PIN_SET);
        break;
    case HAL_IO_OPT_SET_SDA_LOW:
        HAL_GPIO_WritePin(SDA_GPIO_Port, SDA_Pin,GPIO_PIN_RESET);
        break;
    case HAL_IO_OPT_GET_SDA_LEVEL:
        ret = HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin);
        break;
    case HAL_IO_OPT_SET_SDA_INPUT:
        sda_in_mode();
        break;
    case HAL_IO_OPT_SET_SDA_OUTPUT:
        sda_out_mode();
        break;
    case HAL_IO_OPT_SET_SCL_HIGH:
        HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin,GPIO_PIN_SET);
        break;
    case HAL_IO_OPT_SET_SCL_LOW:
        HAL_GPIO_WritePin(SCL_GPIO_Port, SCL_Pin,GPIO_PIN_RESET);
        break;
    case HAL_IO_OPT_GET_SCL_LEVEL:
        ret = HAL_GPIO_ReadPin(SCL_GPIO_Port, SCL_Pin);
        break;
    case HAL_IO_OPT_SET_SCL_INPUT:
        scl_in_mode();
        break;
    case HAL_IO_OPT_SET_SCL_OUTPUT:
        scl_out_mode();
        break;
    default:
        break;
    }
    return ret;
}




sw_i2c_t  k1_sw_i2c= {
	.hal_init = sw_i2c_port_initial,
	.hal_io_ctl = sw_i2c_port_io_ctl,
	.hal_delay_us = delay_us
};

void k1_update_temp(void){
	
}
