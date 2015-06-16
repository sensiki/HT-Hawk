#ifndef __BOARD_CONFIG_H
#define	__BOARD_CONFIG_H

#include "include.h"

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
