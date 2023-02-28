/*
 * @author: Eric Tang, 2020
 */
#include "stm32f4xx_hal.h"

#ifndef __PWM_H
#define __PWM_H

class PWM {
public:
	PWM(TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim3);
/*
 * initializes PWM timer for all four channels
 */
	void start();

	void setduty(uint8_t duty1, uint8_t duty2, uint8_t duty3, uint8_t duty4);

private:
	TIM_HandleTypeDef* m_htim1;
	TIM_HandleTypeDef* m_htim3;
};

#endif
