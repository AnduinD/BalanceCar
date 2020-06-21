#include "motor.h"

/**************************************************************************
函数功能：初始化控制电机转向的IO接口和输出PWM信号的接口
入口参数：重装载值arr和预分频值psc
返回  值：无
**************************************************************************/
void Motor_PWM_Init(u16 arr,u16 psc)
{		 					 
	////初始化控制电机转向的IO口
	RCC->APB2ENR|=1<<3;       //PORTB时钟使能   
	GPIOB->CRH&=0X0000FFFF;   //PORTB12 13 14 15推挽输出
	GPIOB->CRH|=0X22220000;   //PORTB12 13 14 15推挽输出
	
	//初始化电机的PWM接口
	RCC->APB2ENR|=1<<11;       //使能TIM1时钟    
	RCC->APB2ENR|=1<<2;        //PORTA时钟使能     
	GPIOA->CRH&=0XFFFF0FF0;    //PORTA8 11复用输出
	GPIOA->CRH|=0X0000B00B;    //PORTA8 11复用输出
	TIM1->ARR=arr;             //设定计数器自动重装值 
	TIM1->PSC=psc;             //预分频器不分频
	TIM1->CCMR2|=6<<12;        //CH4 PWM1模式	
	TIM1->CCMR1|=6<<4;         //CH1 PWM1模式	
	TIM1->CCMR2|=1<<11;        //CH4预装载使能	 
	TIM1->CCMR1|=1<<3;         //CH1预装载使能	  
	TIM1->CCER|=1<<12;         //CH4输出使能	   
	TIM1->CCER|=1<<0;          //CH1输出使能	
	TIM1->BDTR |= 1<<15;       //TIM1必须要这句话才能输出PWM
	TIM1->CR1=0x8000;          //ARPE使能 
	TIM1->CR1|=0x01;          //使能定时器1 			
} 

