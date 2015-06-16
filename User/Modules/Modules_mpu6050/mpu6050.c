/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：mpu6050.c
 * 描述    ：mpu6050配置         
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 淘宝    ：http://byd2.taobao.com
**********************************************************************************/
#include "include.h"

u8		 mpu6050_buffer[14];					//iic读取后存放数据 	
struct _sensor sensor;



 
 
/*====================================================================================================*/
/*====================================================================================================*
**函数 : InitMPU6050
**功能 : 初始化MPU6050
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
u8 InitMPU6050(void)
{
	u8 ack;
  u8 sig;

	ack = i2cRead(MPU6050_ADDRESS, WHO_AM_I, 1, &sig);
	if (!ack)
     return FALSE;
	
	i2cWrite(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);  	//解除休眠状态
	i2cWrite(MPU6050_ADDRESS, SMPLRT_DIV, 0x07);     
	i2cWrite(MPU6050_ADDRESS, CONFIGL, MPU6050_DLPF);              //低通滤波
	i2cWrite(MPU6050_ADDRESS, GYRO_CONFIG, MPU6050_GYRO_FS_1000);  //陀螺仪量程 +-1000
	i2cWrite(MPU6050_ADDRESS, ACCEL_CONFIG, MPU6050_ACCEL_FS_4);   //加速度量程 +-4G
	return TRUE;
}

//**************************实现函数********************************************
//将iic读取到得数据分拆,放入相应寄存器,更新MPU6050_Last
//******************************************************************************
void MPU6050_Read(void)
{
	i2cRead(MPU6050_ADDRESS,0x3B,14,mpu6050_buffer);
	
}
/**************************实现函数********************************************
//将iic读取到得数据分拆,放入相应寄存器
*******************************************************************************/
void MPU6050_Dataanl(void)
{
	MPU6050_Read();
	
	sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) - sensor.acc.quiet.x;
	sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) - sensor.acc.quiet.y;
	sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);

	sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
  
	sensor.gyro.radian.x = sensor.gyro.origin.x - sensor.gyro.quiet.x;
	sensor.gyro.radian.y = sensor.gyro.origin.y - sensor.gyro.quiet.y;
	sensor.gyro.radian.z = sensor.gyro.origin.z - sensor.gyro.quiet.z;

////////////////////////////////////////////////////
//    	The calibration  of  acc        //
////////////////////////////////////////////////////	
	 if(flag.calibratingA)
	 {
		 static int32_t	tempax=0,tempay=0,tempaz=0;
		 static uint8_t cnt_a=0;
		 if(cnt_a==0)
		 {
				sensor.acc.quiet.x = 0;
				sensor.acc.quiet.y = 0;
				sensor.acc.quiet.z = 0;
				tempax = 0;
				tempay = 0;
				tempaz = 0;
				cnt_a = 1;
		 }
				tempax+= sensor.acc.origin.x;
				tempay+= sensor.acc.origin.y;
				tempaz+= sensor.acc.origin.z;
				if(cnt_a==200)
				{
					sensor.acc.quiet.x = tempax/cnt_a;
					sensor.acc.quiet.y = tempay/cnt_a;
					sensor.acc.quiet.z = tempaz/cnt_a;
					cnt_a = 0;
					flag.calibratingA = 0;
					EE_SAVE_ACC_OFFSET();//保存数据
					return;
				}
				cnt_a++;		
			}	
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Gyro_OFFSET
**功能 : 陀螺仪静态采集
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Gyro_OFFSET(void)
{
   uint16_t cnt_g = 0;
	 int32_t tempgx = GYRO_GATHER;
	 int32_t tempgy = GYRO_GATHER;
	 int32_t tempgz = GYRO_GATHER;
	 int16_t gx_last=0,gy_last=0,gz_last=0;
	 sensor.gyro.quiet.x=0;
	 sensor.gyro.quiet.y=0;
	 sensor.gyro.quiet.z=0;

	 while(tempgx>=GYRO_GATHER || tempgy>=GYRO_GATHER || tempgz>= GYRO_GATHER)	//此循环是确保四轴处于完全静止状态
	 {
		 tempgx=0;tempgy=0;tempgz=0;cnt_g=30;
		 while(cnt_g--)
		 {
			 MPU6050_Read();
			 
			 sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
		   sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
		   sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
		
	     tempgx += absu16(sensor.gyro.origin.x - gx_last);
			 tempgy += absu16(sensor.gyro.origin.y - gy_last);
			 tempgz += absu16(sensor.gyro.origin.z - gz_last);
	
			 gx_last = sensor.gyro.origin.x;
			 gy_last = sensor.gyro.origin.y;
			 gz_last =	sensor.gyro.origin.z;
	  }	 
	}
	tempgx=0;tempgy=0;tempgz=0;cnt_g=2000;

	while(cnt_g--)	 //此循环进行陀螺仪标定，前提是四轴已经处于完全静止状态
	{
		MPU6050_Read();
		 
		sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	  sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	  sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
	
    tempgx += sensor.gyro.origin.x;
		tempgy += sensor.gyro.origin.y;
		tempgz += sensor.gyro.origin.z;
	}

	 sensor.gyro.quiet.x = tempgx/2000;
	 sensor.gyro.quiet.y = tempgy/2000;
	 sensor.gyro.quiet.z = tempgz/2000;	
}



