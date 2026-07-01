#include "PID.h"
#include <zephyr/sys/printk.h>


void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
    PID_Reset(pid);
}

void PID_Calc(PID *pid, float reference, float feedback)
{
    pid->lastError = pid->error;
    pid->error = reference - feedback;
    
    float pout = pid->error * pid->kp;
    pid->integral += pid->error * pid->ki;
    float dout = (pid->error - pid->lastError) * pid->kd;

    /* 积分限幅 */
    if(pid->integral > pid->maxIntegral) {
        pid->integral = pid->maxIntegral;
    } else if(pid->integral < -pid->maxIntegral) {
        pid->integral = -pid->maxIntegral;
    }

    /* 计算输出 */
    pid->output = pout + pid->integral + dout;
    //printk("CAL = %f\n", (double)pid->output); 

    /* 输出限幅 */
    if(pid->output > pid->maxOutput) {
        pid->output = pid->maxOutput;
    } else if(pid->output < -pid->maxOutput) {
        pid->output = -pid->maxOutput;
    }
}

void PID_Reset(PID *pid)
{
    pid->error = 0;
    pid->lastError = 0;
    pid->integral = 0;
    pid->output = 0;
}