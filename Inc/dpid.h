#ifndef DPID_H
#define DPID_H

typedef struct{
    float Kp;
    float Ki;
    float Kd;

    //Integrator limits
    float limMinInt;
    float limMaxInt;

    //Sample time (s)
    float T;

    //average out d
    float run;
    float diffSum;

    float integrator;
    float prevError;
    float differentiator;
    float prevMeasurement;

    //Controller output
    float out;
} DPIDController;
void DPIDController_Init(DPIDController *pid);
float DPIDController_Update(DPIDController *pid, float setpoint, float measurement);



#endif
