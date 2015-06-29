#ifndef _MultiRotor_app_H_
#define _MultiRotor_app_H_
/* Includes ------------------------------------------------------------------*/
#include "include.h"


typedef struct {
	      u8 MpuExist;      // MPU����
	      u8 MagExist;      // MAG����
	      u8 NrfExist;      // NRF����
	      u8 MagIssue;      // MAG������
        u8 ARMED;         // �������
	      u8 LockYaw;       // ��������       
        u8 calibratingA;  // ���ٶȲɼ�
	      u8 calibratingM;  // �����Ʋɼ�
	      u8 calibratingM_pre; //������Ԥ�ɼ�
	      
	      u8 ParamSave;     // ���������־
	
	      u8 Loop_250Hz;
	      u8 Loop_100Hz;
	      u8 Loop_10Hz;
         }Flag_t;

extern Flag_t flag;

void loop(void);
void Bootloader_Set(void);
void InitBoard(void);
void Sensor_Init(void);
void Screen_Update(void);
void Time_slice(void);
#endif /* __MultiRotor_app_H */



