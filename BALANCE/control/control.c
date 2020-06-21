#include "control.h"	
#include "filter.h"	

int Balance_Pwm;
u8 Flag_Target;
u32 Flash_R_Count;
extern int Encoder_L,Encoder_R;                     //左右编码器的脉冲计数
extern int PWMA_pulse,PWMB_pulse;                   //电机PWM变量
extern float angle;																	//滤波后得到的倾角
extern float Angle_Balance,Gyro_Balance;            //平衡倾角 平衡陀螺仪
extern u8 delay_50,delay_flag;
extern float Acceleration_Z;
extern float Balance_Kp,Balance_Kd/*,Velocity_Kp,Velocity_Ki*/;

/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
int EXTI15_10_IRQHandler(void) //PA12引脚触发外部中断时的服务函数（这里PA12接在mpu的INT上）
{    
	 if(PAin(12)==0)		
	{   
		   EXTI->PR=1<<12;                                                      //清除LINE12上的中断标志位   
		   Flag_Target=!Flag_Target;
		   if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //给主函数提供精准延时
			 }
		  if(Flag_Target==1)                                                  //5ms读取一次陀螺仪和加速度计的值，更高的采样频率可以改善卡尔曼滤波的效果
			{
				Get_Angle();                                             					//===更新姿态		
				if(++Flash_R_Count==150&&Angle_Balance>30)Flash_Read();           //=====读取Flash的PID参数		
			  return 0;	                                               
			 }                                                                  //10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据
			Encoder_L= Read_Encoder(2);                                         //===读取编码器的值，因为两个电机的旋转了180度的（不是），所以对其中一个取反，保证输出极性一致
			Encoder_R= Read_Encoder(4);                                         //===读取编码器的值
	  	Get_Angle();                                                        //===更新姿态	
			Led_Flash(100);                                      								//===LED闪烁,1s改变一次指示灯的状态	
 			Balance_Pwm =balance(Angle_Balance,Gyro_Balance);                   //===平衡PID控制	
 		  PWMA_pulse=Balance_Pwm;                            								  //===计算左轮电机最终PWM
 	  	PWMB_pulse=Balance_Pwm;                           									//===计算右轮电机最终PWM
   		Limit_Pwm();                                                       	//===PWM限幅
 			Set_Pwm(PWMA_pulse,PWMB_pulse);                                     //===赋值给PWM寄存器  
	}       	
	 return 0;	  
} 

/**************************************************************************
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
int balance(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-midVal;       										//求出平衡的角度中值 和机械相关
	 balance=Balance_Kp*Bias+Balance_Kd*Gyro;   	//计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}


/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int PWMA_pulse,int PWMB_pulse)
{
	    int Dead_time=400;//死区
			if(PWMA_pulse>0)			AIN1=1,			AIN2=0;
			else 	          			AIN1=0,			AIN2=1;
			PWMA=myabs(PWMA_pulse)+Dead_time;
		  if(PWMB_pulse>0)			BIN1=0,			BIN2=1;
			else        					BIN1=1,			BIN2=0;
			PWMB=myabs(PWMB_pulse)+Dead_time;	
}


/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Limit_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM满幅是7200 限制在6900
    if(PWMA_pulse<-Amplitude) PWMA_pulse=-Amplitude;	
		if(PWMA_pulse>Amplitude)  PWMA_pulse=Amplitude;	
	  if(PWMB_pulse<-Amplitude) PWMB_pulse=-Amplitude;	
		if(PWMB_pulse>Amplitude)  PWMB_pulse=Amplitude;		
	
}

	
/**************************************************************************
函数功能：获取角度
入口参数：获取角度的算法 Kalman滤波
返回  值：无
**************************************************************************/
void Get_Angle(void)
{ 
	    float Accel_Y,Accel_X,Accel_Z,Gyro_X,Gyro_Z;

			Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		  Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取Y轴加速度计
	  	Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
		  if(Gyro_X>32768)  Gyro_X-=65536;                       //数据类型转换  也可通过short强制类型转换
			if(Gyro_Z>32768)  Gyro_Z-=65536;                       //数据类型转换
	  	if(Accel_Y>32768) Accel_Y-=65536;                      //数据类型转换
		  if(Accel_Z>32768) Accel_Z-=65536;                      //数据类型转换
			Gyro_Balance=Gyro_X;                                  //更新平衡角速度
	   	Accel_X=atan2(Accel_Y,Accel_Z)*180/PI;                 //计算倾角	
		  Gyro_X=Gyro_X/16.4;                                    //陀螺仪量程转换	
      Kalman_Filter(Accel_X,Gyro_X);//卡尔曼滤波	

	    Angle_Balance=angle;                                   //更新平衡倾角
			Acceleration_Z=Accel_Z;                                //===更新Z轴加速度计	
}


/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
