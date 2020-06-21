#include "control.h"	
#include "filter.h"	

int Balance_Pwm;
u8 Flag_Target;
u32 Flash_R_Count;
extern int Encoder_L,Encoder_R;                     //���ұ��������������
extern int PWMA_pulse,PWMB_pulse;                   //���PWM����
extern float angle;																	//�˲���õ������
extern float Angle_Balance,Gyro_Balance;            //ƽ����� ƽ��������
extern u8 delay_50,delay_flag;
extern float Acceleration_Z;
extern float Balance_Kp,Balance_Kd/*,Velocity_Kp,Velocity_Ki*/;

/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) //PA12���Ŵ����ⲿ�ж�ʱ�ķ�����������PA12����mpu��INT�ϣ�
{    
	 if(PAin(12)==0)		
	{   
		   EXTI->PR=1<<12;                                                      //���LINE12�ϵ��жϱ�־λ   
		   Flag_Target=!Flag_Target;
		   if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //���������ṩ��׼��ʱ
			 }
		  if(Flag_Target==1)                                                  //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ�����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲���Ч��
			{
				Get_Angle();                                             					//===������̬		
				if(++Flash_R_Count==150&&Angle_Balance>30)Flash_Read();           //=====��ȡFlash��PID����		
			  return 0;	                                               
			 }                                                                  //10ms����һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
			Encoder_L= Read_Encoder(2);                                         //===��ȡ��������ֵ����Ϊ�����������ת��180�ȵģ����ǣ������Զ�����һ��ȡ������֤�������һ��
			Encoder_R= Read_Encoder(4);                                         //===��ȡ��������ֵ
	  	Get_Angle();                                                        //===������̬	
			Led_Flash(100);                                      								//===LED��˸,1s�ı�һ��ָʾ�Ƶ�״̬	
 			Balance_Pwm =balance(Angle_Balance,Gyro_Balance);                   //===ƽ��PID����	
 		  PWMA_pulse=Balance_Pwm;                            								  //===�������ֵ������PWM
 	  	PWMB_pulse=Balance_Pwm;                           									//===�������ֵ������PWM
   		Limit_Pwm();                                                       	//===PWM�޷�
 			Set_Pwm(PWMA_pulse,PWMB_pulse);                                     //===��ֵ��PWM�Ĵ���  
	}       	
	 return 0;	  
} 

/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
��    �ߣ�ƽ��С��֮��
**************************************************************************/
int balance(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-midVal;       										//���ƽ��ĽǶ���ֵ �ͻ�е���
	 balance=Balance_Kp*Bias+Balance_Kd*Gyro;   	//����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}


/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int PWMA_pulse,int PWMB_pulse)
{
	    int Dead_time=400;//����
			if(PWMA_pulse>0)			AIN1=1,			AIN2=0;
			else 	          			AIN1=0,			AIN2=1;
			PWMA=myabs(PWMA_pulse)+Dead_time;
		  if(PWMB_pulse>0)			BIN1=0,			BIN2=1;
			else        					BIN1=1,			BIN2=0;
			PWMB=myabs(PWMB_pulse)+Dead_time;	
}


/**************************************************************************
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Limit_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM������7200 ������6900
    if(PWMA_pulse<-Amplitude) PWMA_pulse=-Amplitude;	
		if(PWMA_pulse>Amplitude)  PWMA_pulse=Amplitude;	
	  if(PWMB_pulse<-Amplitude) PWMB_pulse=-Amplitude;	
		if(PWMB_pulse>Amplitude)  PWMB_pulse=Amplitude;		
	
}

	
/**************************************************************************
�������ܣ���ȡ�Ƕ�
��ڲ�������ȡ�Ƕȵ��㷨 Kalman�˲�
����  ֵ����
**************************************************************************/
void Get_Angle(void)
{ 
	    float Accel_Y,Accel_X,Accel_Z,Gyro_X,Gyro_Z;

			Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //��ȡX��������
			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
		  Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //��ȡY����ٶȼ�
	  	Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
		  if(Gyro_X>32768)  Gyro_X-=65536;                       //��������ת��  Ҳ��ͨ��shortǿ������ת��
			if(Gyro_Z>32768)  Gyro_Z-=65536;                       //��������ת��
	  	if(Accel_Y>32768) Accel_Y-=65536;                      //��������ת��
		  if(Accel_Z>32768) Accel_Z-=65536;                      //��������ת��
			Gyro_Balance=Gyro_X;                                  //����ƽ����ٶ�
	   	Accel_X=atan2(Accel_Y,Accel_Z)*180/PI;                 //�������	
		  Gyro_X=Gyro_X/16.4;                                    //����������ת��	
      Kalman_Filter(Accel_X,Gyro_X);//�������˲�	

	    Angle_Balance=angle;                                   //����ƽ�����
			Acceleration_Z=Accel_Z;                                //===����Z����ٶȼ�	
}


/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
