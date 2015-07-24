#ifndef __BOARD_CONFIG_H
#define	__BOARD_CONFIG_H

#include "include.h"


#define QUADROTOR 
//#define HEXRCOPTER 

#ifdef QUADROTOR 
		#define MOTOR_NUM 4
#elif defined HEXRCOPTER
		#define MOTOR_NUM 6
#endif 



/*-------------������ƫ����----------------------*/
/*-------------��Ҫ  ��Ҫ��----------------------*/
#define FLASH_EXCURSION  0x20000
#define pro_FALG_ADD 0x0801FFF0

/*----------------�������----------------------*/
#define IDLING 210

/*--------------ң�ؿ��Ʒ�ʽѡ��----------------*/
#define RC_CONTROL_USE_NRF24l01

/*---------------�����ǲɼ�---------------------*/
#define GYRO_GATHER   100 

/*----------------���ż��----------------------*/
#define RC_MINCHECK   1200
#define RC_MAXCHECK   1800

/*-------------�Զ������װʱ��-----------------*/
#define AUTODISARMDE_TIME 2500  



typedef void (*rcReadRawData)(void);        

#endif /* __BOARD_CONFIG_H */
