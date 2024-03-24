#include "k1-burst-driver.h"
#ifdef K1_TAG_BOARD_KALINA_V
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "can.h"
#include "burst/burst.h"
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
	debug_set_verb(VERB_REALTIME);
	*SCB_DEMCR = *SCB_DEMCR | 0x01000000; 
	*DWT_CYCCNT = 0; // reset the counter 
	*DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter		
	delay_us(500000);
	//ON(PHASE_CUR_FILT);
	
}
void adc2_query(void);
void adc_start(void){
	HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
}
uint32_t adc_tot_offset = 0;
void burst_hw_start(void){
	HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

	TIM1->ARR = K1_PWM_MODULO;
	TIM1->CCR4 = K1_PWM_MODULO-((K1_ADC_DELAY*5)>>3);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);		
	
	__enable_irq();	
	#if K1_BOARD_COMMON_VER <2
	#if K1_BOARD_SERIAL_1_ENABLED
	k1_serial_start_receive();	
	#endif
	#endif
	HAL_TIMEx_HallSensor_Start(&htim3);

}

//==============TP==================================
void burst_tp_phy_on(void){
	ON(TEST1);
}
void burst_tp_phy_off(void){
	OFF(TEST1);
}
hall_pins_t hall_pins;
void TIM1_CC_IRQHandler(void)
{
	static int presk_tick=0;
	presk_tick++;
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
	//OFF(TEST1);
	
	if( presk_tick == K1_PWM_TIMER_PRESC ){
		debug_tp_on(111);
		ON(TEST2);
		hadc3.Instance->CR2=(0x401 | ADC_CR2_JSWSTART);
		hadc2.Instance->CR2=(0x401 | ADC_CR2_JSWSTART);
		hadc1.Instance->CR1=0x180;
		hadc1.Instance->CR2=(0x401 | ADC_CR2_JSWSTART);
		//HAL_ADCEx_InjectedStart(&hadc2);
		//HAL_ADCEx_InjectedStart(&hadc3);
		//HAL_ADCEx_InjectedStart_IT(&hadc1);
		presk_tick = 0;
	}
	uint32_t tmp = GPIOC->IDR >> 6;
	hall_pins.index = tmp & 0x7;
	hall_update(&hall,&hall_pins);
	
	
}

uint32_t adc_raw[BURST_ADC_CHANNEL_COUNT]={};

burst_bool_t adc2_dma_start = burst_false;
void ADC_IRQHandler(void)
{
	OFF(TEST2);
	__HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);
	//adc2_query();
	//adc2_dma_start = burst_true;
	//__HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOS);
	//if( motor.cross.ac.ref.mode != 14 && motor.cross.ac.ref.mode != 15 ){
		adc_raw[0] = hadc1.Instance->JDR1;//A
		adc_raw[1] = hadc2.Instance->JDR1;//B
		adc_raw[2] = hadc3.Instance->JDR1;//C	
	/*} else {
		switch(swt_phy_sector_get()){
			case 1:
				adc_raw[1] = hadc1.Instance->JDR1;//B
				adc_raw[2] = hadc3.Instance->JDR1;//C
				adc_raw[0] = adc_tot_offset - hadc1.Instance->JDR1 - hadc3.Instance->JDR1;
				break;
			case 3:
				adc_raw[0] = hadc1.Instance->JDR1;//A
				adc_raw[1] = hadc2.Instance->JDR1;//B
				adc_raw[2] = adc_tot_offset - hadc1.Instance->JDR1 - hadc2.Instance->JDR1;
				break;
			case 5:
				adc_raw[0] = hadc1.Instance->JDR1;//A
				adc_raw[2] = hadc3.Instance->JDR1;//C
				adc_raw[1] = adc_tot_offset - hadc1.Instance->JDR1 - hadc3.Instance->JDR1;
				break;
			default:
			adc_raw[0] = hadc1.Instance->JDR1;//A
			adc_raw[1] = hadc2.Instance->JDR1;//B
			adc_raw[2] = hadc3.Instance->JDR1;//C
		}
	}*/

	board.voltage.ps.A.raw = hadc1.Instance->JDR2;//A
	board.voltage.ps.B.raw = hadc3.Instance->JDR2;//B
	board.voltage.ps.C.raw = hadc3.Instance->JDR3;//C
	board.voltage.in.raw = hadc3.Instance->JDR4;
	board.voltage.ref.raw = hadc1.Instance->JDR3;
	board.temper.mk.raw = hadc1.Instance->JDR4;
	board.temper.ps.A.raw = hadc2.Instance->JDR2;
	board.temper.ps.B.raw = hadc2.Instance->JDR3;
	board.temper.ps.C.raw = hadc2.Instance->JDR4;
	
	adc_update(&adc,adc_raw);
	burst_realtime_loop();
	burst_backend_loop();
}

burst_bool_t critical_ = burst_false;	
burst_guard_op_t burst_hw_critical_enter(void){
	__disable_irq();
	if(critical_){
		__enable_irq();
		return  burst_guard_op_skip;
	} else{
		//отключаем прерывание от CAN
		NVIC->ICER[(((uint32_t)CAN1_RX0_IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)CAN1_RX0_IRQn) & 0x1FUL));
		NVIC->ICER[(((uint32_t)CAN1_TX_IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)CAN1_TX_IRQn) & 0x1FUL));
		__DSB();
		__ISB();
		critical_ = burst_true;
		__enable_irq();		
		return  burst_guard_op_run;
	}
}

void burst_hw_critical_leave(void){
	critical_ = burst_false;
  NVIC->ISER[(((uint32_t)CAN1_RX0_IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)CAN1_RX0_IRQn) & 0x1FUL));
  NVIC->ISER[(((uint32_t)CAN1_TX_IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)CAN1_TX_IRQn) & 0x1FUL));
}

burst_guard_op_t burst_hw_guard_enter(void){
		uint32_t prim = __get_PRIMASK();
	__disable_irq();
	return prim == 0? burst_guard_op_run : burst_guard_op_skip;	
}

void burst_hw_guard_leave(void){
	__enable_irq();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
	uint8_t data[8];
	CAN_RxHeaderTypeDef rcvHeader;      /* identifier of the received message */
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rcvHeader,data);
	burst_net_flow_receive(rcvHeader.StdId, data, rcvHeader.DLC);
}

void burst_net_flow_prf_sent(uint16_t _id, const uint8_t * _data, uint8_t _sz) {
	static CAN_TxHeaderTypeDef tx_hdr={};
	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1 )> 0) {
		tx_hdr.DLC = _sz;
		tx_hdr.StdId = _id;

		uint32_t TxMailboxNum; // Transmission MailBox number

		HAL_CAN_AddTxMessage(&hcan1, &tx_hdr, (uint8_t *)_data, &TxMailboxNum);
	}
}
void k1_proto_servo_begin(void){
		CAN_FilterTypeDef FilterConfig;
	FilterConfig.FilterBank = 0;

	FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	FilterConfig.FilterIdHigh = K1_BOARD_ADDRESS<<4<<5;
	FilterConfig.FilterIdLow = 0x0;
	FilterConfig.FilterMaskIdHigh = 0xDF0<<5;
	FilterConfig.FilterMaskIdLow = 0x0;
	FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

	FilterConfig.FilterActivation = ENABLE;
	FilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &FilterConfig);
	FilterConfig.FilterBank = 1;
	FilterConfig.FilterIdHigh = 0xF<<4<<5;
	HAL_CAN_ConfigFilter(&hcan1, &FilterConfig);

	HAL_CAN_ActivateNotification(&hcan1, 
			CAN_IT_RX_FIFO0_MSG_PENDING
		);
	HAL_CAN_Start(&hcan1);
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
	TIM1->CCR1 = motor.inverter.duty.A;
	TIM1->CCR2 = motor.inverter.duty.B;
	TIM1->CCR3 = motor.inverter.duty.C;
	return burst_satstate_none;
}

void power_shutdown_begin(void){
	TIM1->CCER = 0x1000;
	/*HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1,TIM_CHANNEL_3);*/
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;	
}

void burst_hw_on_crash(void){
	__disable_irq();
	power_shutdown_begin();	
	while(1){
	}
}

extern DMA_HandleTypeDef hdma_adc2;
typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;
uint32_t adc2_raw[1]={};
void adc2_query(void)
{
	ADC2->CR2 &=  ~(ADC_CR2_ADON | ADC_CR2_DMA );
	DMA_Base_Registers *regs = (DMA_Base_Registers *)hdma_adc2.StreamBaseAddress;
  regs->IFCR = 0x3FU << hdma_adc2.StreamIndex;
	DMA2_Stream2->CR &= (uint32_t)(~DMA_SxCR_DBM);
	
  ADC2->SR &= ~(ADC_FLAG_EOC | ADC_FLAG_OVR);
  /* Configure DMA Stream data length */
  DMA2_Stream2->NDTR = 1;


	/* Configure DMA Stream source address */
	DMA2_Stream2->PAR = (uint32_t)&ADC2->DR;

	/* Configure DMA Stream destination address */
	DMA2_Stream2->M0AR = (uint32_t)adc2_raw;
  
	DMA2_Stream2->CR |=  DMA_SxCR_EN;
  ADC2->CR2 |= (ADC_CR2_DMA|ADC_CR2_SWSTART);
   
}


void burst_hw_frontend_loop(void){
	static int counter;
	
	if(adc2_dma_start == burst_true){
		counter++;
		if(counter>1){			
			counter = 0;
			board.voltage.in.raw = adc2_raw[0];
			//board.temper.ps.A.raw =adc2_raw[1];
			//board.temper.ps.B.raw =adc2_raw[3];
			//board.temper.ps.C.raw =adc2_raw[4];
			adc2_dma_start = burst_false;
			adc2_query();
		}
		
	}
}

void swt_phy_A_set_pwm(uint16_t _pwm){
	TIM1->CCR1 = _pwm;
}
void swt_phy_B_set_pwm(uint16_t _pwm){
	TIM1->CCR2 = _pwm;
}
void swt_phy_C_set_pwm(uint16_t _pwm){
	TIM1->CCR3 = _pwm;	
}
void swt_phy_A_set_lo(void){
	TIM1->CCR1 = 0;
	TIM1->CCER |= 0x1005;
}
void swt_phy_B_set_lo(void){
	TIM1->CCR2 = 0;
	TIM1->CCER |= 0x1050;
}
void swt_phy_C_set_lo(void){
	TIM1->CCR3 = 0;	
	TIM1->CCER |= 0x1500;
}

void swt_phy_A_off(void){
	TIM1->CCR1 = 0;	
	TIM1->CCER &= ~0x005;
}
void swt_phy_B_off(void){
	TIM1->CCR2 = 0;	
	TIM1->CCER &= ~0x050;
}
void swt_phy_C_off(void){
	TIM1->CCR3 = 0;	
	TIM1->CCER &= ~0x500;
}

void swt_phy_A_on(uint16_t _pwm){
	TIM1->CCR1 = _pwm;
	TIM1->CCER |= 0x1005;
}
void swt_phy_B_on(uint16_t _pwm){
	TIM1->CCR2 = _pwm;
	TIM1->CCER |= 0x1050;
}
void swt_phy_C_on(uint16_t _pwm){
	TIM1->CCR3 = _pwm;
	TIM1->CCER |= 0x1500;
}

/*
burst_guard_op_t burst_hw_guard_lock(void){
		uint32_t prim = __get_PRIMASK();
	__disable_irq();
	return prim == 0? burst_guard_op_run : burst_guard_op_skip;	
}
void burst_hw_guard_unlock(void){
	__enable_irq();
}
*/


#if 0
burst_bool_t serialComDone = burst_true;
void burst_hw_frontend_loop(void){
	board.temper.dg = K1_BOARD_TEMPER_PP_TO_GRAD(board.temper.raw);
}


void burst_hw_realtime_loop(void){
}



uint32_t vref_raw = 0;





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





#endif

#endif
