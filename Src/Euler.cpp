/*
 * Euler.c
 *
 *  Created on: Sep 7, 2021
 *      Author: Alexiy Samoylov
 */

#include "Euler.h"
#include <math.h>

//Euler::Euler(float alpha, float roll_acc, float pitch_acc, float roll, float pitch, float yaw)
//{
//	this->alpha = alpha;
//
//	this->roll_acc = roll_acc;
//	this->pitch_acc = pitch_acc;
//
//	this->roll = roll;
//	this->pitch = pitch;
//	this->yaw = yaw;
//}

void Euler::Euler_intialize(Orientation* orientation, float alpha) {
	orientation->alpha = alpha;

	orientation->roll_acc = 0;
	orientation->pitch_acc = 0;

	orientation->roll = 0;
	orientation->pitch = 0;
	orientation->yaw = 0;
}

void Euler::Euler_convert_raw(Orientation* orientation, int16_t IMU_acc[], int16_t IMU_gyro[], float* elapsed_time) {
	// Assign accelerometer measurements (get rid of this in the future)
	int16_t x_acc = IMU_acc[1];
	int16_t y_acc = IMU_acc[0];
	int16_t z_acc = IMU_acc[2];

	// Assign gyroscope measurements (get rid of this in the future)
	float degree_to_radian = 0.017453;
	float roll_gyro = IMU_gyro[1] * 0.061 * degree_to_radian;
	float pitch_gyro = IMU_gyro[0] * 0.061 * degree_to_radian;
	float yaw_gyro = IMU_gyro[2] * 0.061 * degree_to_radian;

	// Compute roll and pitch from accelerometer
	float roll_acc_new = atan2(y_acc, z_acc);
	float pitch_acc_new = -atan2(x_acc, z_acc);

	// Low pass filter accelerometer values
	orientation->roll_acc = 0.0 * orientation->roll_acc + 1.0 * roll_acc_new;
	orientation->pitch_acc = 0.0 * orientation->pitch_acc + 1.0 * pitch_acc_new;

	// Obtain elapsed time
	//*elapsed_time = __HAL_TIM_GET_COUNTER(timer) / 1000000.0f;
	// Reset timer
	//TIM16->CNT = 0;

	// Computer roll, pitch, and yaw using a complementary filter
	orientation->roll = orientation->alpha * (orientation->roll - (roll_gyro * *elapsed_time)) + (1 - orientation->alpha) * orientation->roll_acc;
	orientation->pitch = orientation->alpha * (orientation->pitch - (pitch_gyro * *elapsed_time)) + (1 - orientation->alpha) * orientation->pitch_acc;
	orientation->yaw = orientation->yaw - (yaw_gyro * *elapsed_time);
}
