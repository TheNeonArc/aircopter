#include "DPID.h"

void DPIDController_Init(DPIDController *pid) {

    /* Clear controller variables */
    pid->integrator = 0.0f;
    pid->prevError  = 0.0f;

    pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->out = 0.0f;

    pid->run = 0;
    pid->diffSum = 0;
}

float DPIDController_Update(DPIDController *pid, float setpoint, float measurement) {

    /*
    * Error signal
    */
    float error = setpoint - measurement;


    /*
    * Proportional
    */
    float proportional = pid->Kp * error;


    /*
    * Integral
    */
    pid->integrator = pid->integrator + pid->Ki * pid->T * (error + pid->prevError);

    /* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }


    /*
    * Derivative (band-limited differentiator)
    */

//    if (pid->run >= 10)
//    {
//    	pid->differentiator = pid->diffSum / 10;//pid->Kd/pid->T * -(measurement - pid->prevMeasurement);
//    	pid->run = 1;
//    	pid->diffSum = -2.0f * pid->Kd * (measurement - pid->prevMeasurement);
//    }
//    else
//    {
//    	pid->run++;
//    	pid->diffSum += -2.0f * pid->Kd * (measurement - pid->prevMeasurement);
//    }
    pid->differentiator = -2.0f * pid->Kd * (measurement - pid->prevMeasurement);

    /*
    * Compute output and apply limits
    */
    pid->out = proportional + pid->integrator + pid->differentiator;



    /* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

    /* Return controller output */
    return pid->out;

}
