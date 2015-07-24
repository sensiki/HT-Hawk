/*
******************* (C) COPYRIGHT 2015 Air Nano Team ***************************
 * ģ������ : ���Ƴ���
 * �ļ���   ��
 * ����     ��    
 * ʵ��ƽ̨ ��HT_Hawk
 * ��汾   ��ST3.5.0
 * ����     ��Air Nano Team 
 * ��̳    ��http://www.airnano.cn 
 * �Ա�     ��http://byd2.taobao.com   
 *            http://hengtuo.taobao.com   
*********************************************************************************
*/
#include "include.h"
#include "MultiRotor_control.h"


struct _ctrl ctrl;
struct _target Target;

s16 Moto_duty[MOTOR_NUM];
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

/*====================================================================================================*/
/*====================================================================================================*
**���� : CONTROL(struct _target Goal) 
**���� : ����PID ��?
**���� : Goal
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
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
		ctrl.pitch.shell.increment = data_limit(ctrl.pitch.shell.increment,ctrl.pitch.shell.increment_max,-ctrl.pitch.shell.increment_max);

		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * deviation_pitch + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment;
		
		//��������//////////////
		deviation_roll = Goal.Roll - AngE.Roll;
		ctrl.roll.shell.increment += deviation_roll;
		
		//limit for the max increment
		ctrl.roll.shell.increment = data_limit(ctrl.roll.shell.increment,ctrl.roll.shell.increment_max,-ctrl.roll.shell.increment_max);

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
	
	// ����ƫ��  
	E_pitch = ctrl.pitch.shell.pid_out - sensor.gyro.averag.y;
	E_roll  = ctrl.roll.shell.pid_out  - sensor.gyro.averag.x;
	E_yaw   = ctrl.yaw.shell.pid_out   - sensor.gyro.averag.z;
	
	// ����
	ctrl.pitch.core.increment += E_pitch;
	ctrl.roll.core.increment  += E_roll;
	ctrl.yaw.core.increment   += E_yaw;
	
	// �����޷�
	ctrl.pitch.core.increment = data_limit(ctrl.pitch.core.increment,20,-20);
	ctrl.roll.core.increment  = data_limit(ctrl.roll.core.increment,20,-20);		
	ctrl.yaw.core.increment   = data_limit(ctrl.yaw.core.increment,20,-20);
	
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * E_pitch;
	ctrl.roll.core.kp_out  = ctrl.roll.core.kp  * E_roll;
	ctrl.yaw.core.kp_out   = ctrl.yaw.core.kp   * E_yaw;
	
	ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment;
  ctrl.roll.core.ki_out  = ctrl.roll.core.ki  * ctrl.roll.core.increment;
	ctrl.yaw.core.ki_out   = ctrl.yaw.core.ki   * ctrl.yaw.core.increment;
	
	// ΢��
	ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (sensor.gyro.histor.y - sensor.gyro.averag.y)*33;
	ctrl.roll.core.kd_out  = ctrl.roll.core.kd  * (sensor.gyro.histor.x - sensor.gyro.averag.x)*33;
	ctrl.yaw.core.kd_out   = ctrl.yaw.core.kd   * (sensor.gyro.histor.z - sensor.gyro.averag.z)*33;	
	
	sensor.gyro.histor.y = sensor.gyro.averag.y;
	sensor.gyro.histor.x = sensor.gyro.averag.x; 
  sensor.gyro.histor.z = sensor.gyro.averag.z;	
	
	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.ki_out + ctrl.pitch.core.kd_out;
	ctrl.roll.core.pid_out  = ctrl.roll.core.kp_out  + ctrl.roll.core.ki_out  + ctrl.roll.core.kd_out;
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.kp_out   + ctrl.yaw.core.kd_out;
	
	ctrl.pitch.core.pid_out = ctrl.pitch.core.pid_out*0.8 + ctrl.pitch.shell.pid_out/2;
	ctrl.roll.core.pid_out  = ctrl.roll.core.pid_out *0.8 + ctrl.roll.shell.pid_out/2; 
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.pid_out;

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
		int date_throttle	= (RC_Data.THROTTLE-1000)/cos(AngE.Roll/RtA)/cos(AngE.Pitch/RtA);
		
		#ifdef QUADROTOR 
			Moto_duty[0] = date_throttle - pitch - roll + yaw;
			Moto_duty[1] = date_throttle - pitch + roll - yaw;
			Moto_duty[2] = date_throttle + pitch + roll + yaw;
			Moto_duty[3] = date_throttle + pitch - roll - yaw;
		#elif defined HEXRCOPTER
			Moto_duty[0] = date_throttle - pitch + 0.5*roll - yaw;
			Moto_duty[1] = date_throttle         +     roll + yaw;
			Moto_duty[2] = date_throttle + pitch + 0.5*roll - yaw;
			Moto_duty[3] = date_throttle + pitch - 0.5*roll + yaw;	
			Moto_duty[4] = date_throttle         -     roll - yaw;
			Moto_duty[5] = date_throttle - pitch - 0.5*roll + yaw;	
		#endif 	
	}
	else
	{	
		array_assign(&Moto_duty[0],IDLING,MOTOR_NUM);
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

