/*
 * author: Eric Tang 2020
 */
#include "stm32f4xx_hal.h"
#include "pwm.h"

//TO DO: Put all timer sources in class initialization. Add private variables where needed.
PWM::PWM(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim3):m_htim1(htim1), m_htim3(htim3){
}

//TO DO: Change channels and timer ports for HAL_TIM_PWM_Start
void PWM::start(){
	HAL_TIMEx_PWMN_Start(m_htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(m_htim1, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(m_htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(m_htim3, TIM_CHANNEL_2);
}

//TO DO: Change which timer port you use and also which channel to use. CCRx is for channel x.
void PWM::setduty(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4){
	//M1
	(*m_htim1).Instance->CCR2 = duty1;

	//M2
	(*m_htim1).Instance->CCR3 = duty2;

	//M3
	(*m_htim3).Instance->CCR1 = duty3;

	//M4
	(*m_htim3).Instance->CCR2 = duty4;

	//note: timer 1 uses N timer channels (inverted)
}
