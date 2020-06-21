#include "sys.h"

int Encoder_L,Encoder_R;             																		//左右编码器的脉冲计数
int PWMA_pulse,PWMB_pulse;                           										//电机PWM变量
float Angle_Balance,Gyro_Balance; 																			//平衡倾角 平衡陀螺仪
u8 delay_50,delay_flag;																									//延时变量
float Acceleration_Z;                       														//Z轴加速度计  
float Balance_Kp=495,Balance_Kd= 0.48;																	//PID参数    /*Velocity_Kp=50,Velocity_Ki=0.25*/
u16 PID_Parameter[10],Flash_Parameter[10];  														//Flash相关数组

int main(void)
{ 
	Stm32_Clock_Init(9);            //=====系统时钟设置(8.000M的9倍频)
	delay_init(72);                 //=====延时初始化(72M的时钟)
	LED_Init();                     //=====初始化LED
	USART1_Init(72,115600);         //=====初始化串口1
  Motor_PWM_Init(7199,0);   			//=====初始化PWM 10KHZ，用于驱动电机
	Encoder_Init();									//=====初始化编码器
	IIC_Init();                     //=====模拟IIC初始化
  MPU6050_initialize();           //=====MPU6050初始化	
  DMP_Init();                     //=====初始化DMP     
 	EXTI_Init();                    //=====MPU6050 5ms定时中断初始化
	while(1)
		{     
				  delay_flag=1;	
					delay_50=0;
					while(delay_flag);	     //通过MPU6050的INT中断实现的50ms精准延时				
		} 
}
