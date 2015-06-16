#ifndef _PWM_IN_H_
#define _PWM_IN_H_
#include "stm32f10x.h"
#include "board_config.h"


#define GPIO_TIM4	     GPIOA
#define TIM4_CH1       GPIO_Pin_0
#define TIM4_CH2       GPIO_Pin_1
#define TIM4_CH3       GPIO_Pin_2
#define TIM4_CH4       GPIO_Pin_3
#define RCC_GPIO_TIM4  RCC_APB2Periph_GPIOA

extern u16  RC_Pwm_In[8];
extern u16  RC_Pwm_In_his[8];

void PWM_IN_Config(void);

#endif
