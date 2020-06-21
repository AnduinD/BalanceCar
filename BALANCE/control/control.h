#ifndef __CONTROL_H
#define __CONTROL_H
#include <math.h>
#include "sys.h"

#define PI 3.14159265
#define midVal 0

int EXTI15_10_IRQHandler(void);
int balance(float angle,float gyro);
void Set_Pwm(int PWMA_pulse,int PWMB_pulse);
void Limit_Pwm(void);
void Get_Angle(void);
int myabs(int a);
#endif
