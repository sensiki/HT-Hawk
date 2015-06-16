#ifndef __BOARD_CONFIG_H
#define	__BOARD_CONFIG_H

#include "include.h"

/*----------------电机怠速----------------------*/
#define IDLING 210

/*--------------遥控控制方式选择----------------*/
#define RC_CONTROL_USE_NRF24l01

/*---------------陀螺仪采集---------------------*/
#define GYRO_GATHER   100 

/*----------------油门检查----------------------*/
#define RC_MINCHECK   1200
#define RC_MAXCHECK   1800

/*-------------自动解除武装时间-----------------*/
#define AUTODISARMDE_TIME 2500  



typedef void (*rcReadRawData)(void);        

#endif /* __BOARD_CONFIG_H */
