#ifndef __LED_H
#define __LED_H
#include "sys.h"

#define LED PAout(4) // LED 端口定义 PA4
void LED_Init(void);  //初始化
void Led_Flash(u16 time);
#endif
