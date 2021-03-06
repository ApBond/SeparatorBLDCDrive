#ifndef __CURRENT_LOOP_H
#define __CURRENT_LOOP_H

#include "main.h"

#define MEASURMENT_COUNT 50
#define CURRENT_BREAK_LIMIT 700//AMP*0.07/0.00081
#define CURRENT_BREAK_LIMIT_AMP 25
#define CURRENT_SENSOR_OFFSET 1895
#define CURRENT_SENSOR_SENSETIVITY 0.07
#define TIM_PWM_LIMIT 10000
#define CURRENT_REFERENCE 10

typedef struct
{
    float kp;
    float ki;
    float kd;
    float prevError;
    float integralTerm;
}PIDHandle_t;


void adcInit(uint8_t channel);
void Tim5InitTrigger(void);
void setTriggerPetiod(uint16_t period_us);
float PIDController(PIDHandle_t * PID,float error);


#endif