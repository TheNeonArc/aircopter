/*
 * Euler.h
 *
 *  Created on: Sep 7, 2021
 *      Author: Alexiy Samoylov
 */

#ifndef INC_EULER_H_
#define INC_EULER_H_

#include "stm32f4xx_hal.h"

struct Orientation {
		float alpha;

		float roll_acc;
		float pitch_acc;

		float roll;
		float pitch;
		float yaw;
	};

class Euler {


public:


//	Euler(float alpha, float roll_acc, float pitch_acc, float roll, float pitch, float yaw);

	void Euler_intialize(Orientation* orientation, float alpha);

	void Euler_convert_raw(Orientation* orientation, int16_t IMU_acc[], int16_t IMU_gyro[], float* elapsed_time);

};



//typedef struct {
//	float alpha;
//
//	float roll_acc;
//	float pitch_acc;
//
//	float roll;
//	float pitch;
//	float yaw;
//} Orientation;
//
//void Euler_intialize(Orientation* orientation, float alpha);
//
//void Euler_convert_raw(Orientation* orientation, int16_t IMU_acc[], int16_t IMU_gyro[], float* elapsed_time);

#endif /* INC_EULER_H_ */
