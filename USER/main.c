#include "sys.h"

int Encoder_L,Encoder_R;             																		//���ұ��������������
int PWMA_pulse,PWMB_pulse;                           										//���PWM����
float Angle_Balance,Gyro_Balance; 																			//ƽ����� ƽ��������
u8 delay_50,delay_flag;																									//��ʱ����
float Acceleration_Z;                       														//Z����ٶȼ�  
float Balance_Kp=495,Balance_Kd= 0.48;																	//PID����    /*Velocity_Kp=50,Velocity_Ki=0.25*/
u16 PID_Parameter[10],Flash_Parameter[10];  														//Flash�������

int main(void)
{ 
	Stm32_Clock_Init(9);            //=====ϵͳʱ������(8.000M��9��Ƶ)
	delay_init(72);                 //=====��ʱ��ʼ��(72M��ʱ��)
	LED_Init();                     //=====��ʼ��LED
	USART1_Init(72,115600);         //=====��ʼ������1
  Motor_PWM_Init(7199,0);   			//=====��ʼ��PWM 10KHZ�������������
	Encoder_Init();									//=====��ʼ��������
	IIC_Init();                     //=====ģ��IIC��ʼ��
  MPU6050_initialize();           //=====MPU6050��ʼ��	
  DMP_Init();                     //=====��ʼ��DMP     
 	EXTI_Init();                    //=====MPU6050 5ms��ʱ�жϳ�ʼ��
	while(1)
		{     
				  delay_flag=1;	
					delay_50=0;
					while(delay_flag);	     //ͨ��MPU6050��INT�ж�ʵ�ֵ�50ms��׼��ʱ				
		} 
}
