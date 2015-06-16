/*
******************* (C) COPYRIGHT 2015 Air Nano Team ***************************
 * ģ������ : ���Ƴ���
 * �ļ���   ��
 * ����     ��    
 * ʵ��ƽ̨ ��Air Nano���������
 * ��汾   ��ST3.5.0
 * ����     ��Air Nano Team 
 * �Ա�     ��http://byd2.taobao.com   
 *            http://hengtuo.taobao.com   
*********************************************************************************
*/
#include "include.h"
#include "MultiRotor_control.h"


struct _ctrl ctrl;
struct _target Target;

int16_t Moto_duty[4];
s16 *motor_array = Moto_duty;

/*====================================================================================================*/
/*====================================================================================================*
**���� : Calculate_target
**���� : ����Ŀ����
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Calculate_Target(void) 
{
	int16_t ftemp=0;
	
	Target.Pitch = (1500-RC_Data.PITCH)/(20 + 7*RC_Data.SENSITIVITY);
	Target.Roll = (RC_Data.ROLL-1500)/(20 + 7*RC_Data.SENSITIVITY);

  //Ŀ�꺽����ơ������Ŵ�����С���ֵʱ����Ϊ�û�ϣ����ɡ���ô��ʱ�ĺ�����ΪĿ�꺽��
   if(RC_Data.THROTTLE > RC_MINCHECK ) {
      if(flag.LockYaw != 1){  
				 flag.LockYaw = 1;
	       Target.Yaw = AngE.Yaw; //����ǰ�ĺ�����ΪĿ�꺽��
      }
   }
   else {
		 flag.LockYaw = 0;	
		 Target.Yaw = AngE.Yaw;
	 } 
	//�������е�����һ������
	if((RC_Data.YAW > 1600)||(RC_Data.YAW < 1400)){
		ftemp = 1500 - RC_Data.YAW; 
	  Target.Yaw += (ftemp / 200.0f)*0.1f; 
		
		//ת[-180.0,+180.0]
	  if(Target.Yaw >180.0f) Target.Yaw -= 360.0f;	
	  else if(Target.Yaw <-180.0f)Target.Yaw += 360.0f;
	}
}

/***************************************************/
/*void CONTROL(float rol, float pit, float yaw)    */
/*���룺rol   �����                               */
/*      pit   ������                               */
/*			yaw   ����                                 */
/*�����                                           */
/*��ע������PID ����   �⻷���ǶȻ�������PID����    */
/*                     �ڻ������ٶȻ�������PD����  */
/***************************************************/
void CONTROL(struct _target Goal)   
{
	float  deviation_pitch,deviation_roll,deviation_yaw;
	
	if(ctrl.ctrlRate >= 2)
	{
		//*****************�⻷(�ǶȻ�)PID**************************//
		//�������///////////////
	  deviation_pitch = Goal.Pitch - AngE.Pitch;
		ctrl.pitch.shell.increment += deviation_pitch;
		
		//limit for the max increment
		if(ctrl.pitch.shell.increment > ctrl.pitch.shell.increment_max)  	ctrl.pitch.shell.increment = ctrl.pitch.shell.increment_max;
		else if(ctrl.pitch.shell.increment < -ctrl.pitch.shell.increment_max)		ctrl.pitch.shell.increment = -ctrl.pitch.shell.increment_max;
		
		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * deviation_pitch + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment;
		
		//��������//////////////
		deviation_roll = Goal.Roll - AngE.Roll;
		ctrl.roll.shell.increment += deviation_roll;
		
		//limit for the max increment
		if(ctrl.roll.shell.increment > ctrl.roll.shell.increment_max)  	ctrl.roll.shell.increment = ctrl.roll.shell.increment_max;
		else if(ctrl.roll.shell.increment < -ctrl.roll.shell.increment_max)		ctrl.roll.shell.increment = -ctrl.roll.shell.increment_max;

		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * deviation_roll + ctrl.roll.shell.ki * ctrl.roll.shell.increment;
		
		//�������////////////
    if((Goal.Yaw - AngE.Yaw)>180 || (Goal.Yaw - AngE.Yaw)<-180){
       if(Goal.Yaw>0 && AngE.Yaw<0)  deviation_yaw= (-180 - AngE.Yaw) +(Goal.Yaw - 180);
       if(Goal.Yaw<0 && AngE.Yaw>0)  deviation_yaw= (180 - AngE.Yaw) +(Goal.Yaw + 180);
    }
    else  deviation_yaw = Goal.Yaw - AngE.Yaw;
		
	  ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * deviation_yaw;
    ctrl.ctrlRate = 0; 
	}
	ctrl.ctrlRate ++;
  Attitude_RatePID();
	
	Motor_Conter();
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : Attitude_RatePID
**���� : �����ʿ���PID
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Attitude_RatePID(void)
{
  fp32 E_pitch,E_roll,E_yaw;
	
	// ���㸩��ƫ��  
	E_pitch = ctrl.pitch.shell.pid_out - sensor.gyro.averag.y * Gyro_G;
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * E_pitch;
	
	// ����
	ctrl.pitch.core.increment += E_pitch;
	data_limit(ctrl.pitch.core.increment,10,-10);
	ctrl.pitch.core.ki_out = ctrl.pitch.core.ki/10 * ctrl.pitch.core.increment;

	// ΢��
	ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (sensor.gyro.histor.y - sensor.gyro.averag.y);
	
	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.ki_out + ctrl.pitch.core.kd_out;
	sensor.gyro.histor.y = sensor.gyro.averag.y;
	
	// ������ƫ��
	E_roll = ctrl.roll.shell.pid_out - sensor.gyro.averag.x * Gyro_G;
	ctrl.roll.core.kp_out = ctrl.roll.core.kp * E_roll;
	
	// ����
	ctrl.roll.core.increment += E_roll;
	data_limit(ctrl.roll.core.increment,10,-10);
	ctrl.roll.core.ki_out = ctrl.roll.core.ki/10 * ctrl.roll.core.increment;
	
	// ΢��
	ctrl.roll.core.kd_out = ctrl.roll.core.kd * (sensor.gyro.histor.x - sensor.gyro.averag.x);
	
	ctrl.roll.core.pid_out = ctrl.roll.core.kp_out + ctrl.roll.core.ki_out + ctrl.roll.core.kd_out;
	sensor.gyro.histor.x = sensor.gyro.averag.x;   
	
	// ���㺽��ƫ��
	E_yaw = ctrl.yaw.shell.pid_out - sensor.gyro.averag.z * Gyro_G;
	ctrl.yaw.core.kp_out = ctrl.yaw.core.kp * E_yaw;
	
	// ����
	ctrl.yaw.core.increment += E_yaw;
	data_limit(ctrl.yaw.core.increment,10,-10);
	ctrl.yaw.core.ki_out = ctrl.yaw.core.ki * ctrl.yaw.core.increment;
	
	// ΢��
	ctrl.yaw.core.kd_out = ctrl.yaw.core.kd * (sensor.gyro.histor.z - sensor.gyro.averag.z);
	
	ctrl.yaw.core.pid_out = ctrl.yaw.core.kp_out + ctrl.yaw.core.kd_out;
  sensor.gyro.histor.z = sensor.gyro.averag.z;
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : Motor_Conter(void)
**���� : �������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Motor_Conter(void)
{
	s16 pitch,roll,yaw;
	
	pitch = ctrl.pitch.core.pid_out;
  roll  = ctrl.roll.core.pid_out;    
 	yaw   = -ctrl.yaw.core.pid_out;
	
  if(RC_Data.THROTTLE > RC_MINCHECK) {
		int date_throttle	= (RC_Data.THROTTLE-1000)/cos(angle.roll/RtA)/cos(angle.pitch/RtA);
		
		Moto_duty[0] = date_throttle - pitch - roll + yaw;
	  Moto_duty[1] = date_throttle - pitch + roll - yaw;
		Moto_duty[2] = date_throttle + pitch + roll + yaw;
	  Moto_duty[3] = date_throttle + pitch - roll - yaw;
	}
	else
	{	
    Moto_duty[0] = Moto_duty[1] = Moto_duty[2] = Moto_duty[3] = IDLING;	
		Reset_Integral();		
	}
	if(flag.ARMED)  moto_PwmRflash(&Moto_duty[0]);		
	else            moto_STOP();	
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : Reset_Integral
**���� : ��������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Reset_Integral(void)
{
	ctrl.pitch.shell.increment = 0;
	ctrl.roll.shell.increment= 0;	
  ctrl.pitch.core.increment = 0;		
  ctrl.roll.core.increment = 0;		
	ctrl.yaw.core.increment = 0;
}

