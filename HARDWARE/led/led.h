#ifndef __LED_H
#define __LED_H
#include "sys.h"

#define LED PAout(4) // LED �˿ڶ��� PA4
void LED_Init(void);  //��ʼ��
void Led_Flash(u16 time);
#endif
