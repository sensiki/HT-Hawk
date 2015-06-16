/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：PWM_IN.c
 * 描述    ：PWM输入捕获      
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 淘宝    ：http://byd2.taobao.com
**********************************************************************************/
#include "pwm_in.h"


u16  Rise[4],Drop[4];
u16  RC_Pwm_In[8];
u16  RC_Pwm_In_his[8];
/*====================================================================================================*/
/*====================================================================================================*
**函数 : PWM_IN_Config
**功能 : 配置PWM输入捕获
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void PWM_IN_Config(void)
{
	  GPIO_InitTypeDef         GPIO_InitStructure;
	  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	  TIM_ICInitTypeDef  TIM4_ICInitStructure;

	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);	 //使能TIM4时钟
 	  RCC_APB2PeriphClockCmd(RCC_GPIO_TIM4, ENABLE);  

	  GPIO_InitStructure.GPIO_Pin  = TIM4_CH1 | TIM4_CH2 | TIM4_CH3 | TIM4_CH4;             
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;            
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_ResetBits(GPIO_TIM4,TIM4_CH1 | TIM4_CH2 | TIM4_CH3 | TIM4_CH4);		

	  //初始化定时器4 TIM4	 
	  TIM_TimeBaseStructure.TIM_Period = 0XFFFF;                   //设定计数器自动重装值 
	  TIM_TimeBaseStructure.TIM_Prescaler =71; 	                   //预分频器   
	  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //设置时钟分割:TDTS = Tck_tim
	  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);              //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  
	  //初始化TIM4输入捕获参数
	  TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1;                //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPolarity =TIM_ICPolarity_Rising;	   //上升沿捕获
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM2, &TIM4_ICInitStructure);
	
	  TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2;                //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM2, &TIM4_ICInitStructure);
		
		TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3;                //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM2, &TIM4_ICInitStructure);
		
		TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4;                //CC1S=01 	选择输入端 IC1映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	   //上升沿捕获
  	TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  	TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	         //配置输入分频,不分频 
  	TIM4_ICInitStructure.TIM_ICFilter = 0x00;                        //IC1F=0000 配置输入滤波器 不滤波
  	TIM_ICInit(TIM2, &TIM4_ICInitStructure);
	
	  TIM_Cmd(TIM2,ENABLE ); 
		
	  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);        //允许更新中断 ,允许CC1IE捕获中断	
	  TIM_ITConfig(TIM2, TIM_IT_CC2, ENABLE);
	  TIM_ITConfig(TIM2, TIM_IT_CC3, ENABLE);
	  TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
			
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : TIM4_IRQHandler
**功能 : TIM4中断服务
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void TIM2_IRQHandler(void)
{ 
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)   //捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC1); //清除中断标志位
			if(GPIO_ReadInputDataBit(GPIO_TIM4,TIM4_CH1) == 1) 
			{
				  TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
          Rise[0]=TIM_GetCapture1(TIM2);
      }
			else 
			{
				  TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
          Drop[0]=TIM_GetCapture1(TIM2);
				  if(Rise[0]>Drop[0])  RC_Pwm_In[0] = 65535-Rise[0] + Drop[0];
					else 	               RC_Pwm_In[0] = Drop[0] - Rise[0];
      }			
		}	
	  
		if (TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)   //捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC2); //清除中断标志位
			if(GPIO_ReadInputDataBit(GPIO_TIM4,TIM4_CH2) == 1) 
			{
				  TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
          Rise[1]=TIM_GetCapture2(TIM2);
      }
			else 
			{
				  TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
          Drop[1]=TIM_GetCapture2(TIM2);
				  if(Rise[1]>Drop[1])  RC_Pwm_In[1] = 65535-Rise[1] + Drop[1];
					else 	               RC_Pwm_In[1] = Drop[1] - Rise[1];
      }			
		}	
		
    if (TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)            //捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC3); //清除中断标志位
			if(GPIO_ReadInputDataBit(GPIO_TIM4,TIM4_CH3) == 1) 
			{
				  TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
          Rise[2]=TIM_GetCapture3(TIM2);
      }
			else 
			{
				  TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
          Drop[2]=TIM_GetCapture3(TIM2);
				  if(Rise[2]>Drop[2]) RC_Pwm_In[2] = 65535-Rise[2] + Drop[2];
					else 	              RC_Pwm_In[2] = Drop[2] - Rise[2];
      }	 
		}	

    if (TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)            //捕获1发生捕获事件
		{	
			TIM_ClearITPendingBit(TIM2, TIM_IT_CC4); //清除中断标志位
		  if(GPIO_ReadInputDataBit(GPIO_TIM4,TIM4_CH4) == 1) 
			{
				  TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
          Rise[3]=TIM_GetCapture4(TIM2);
      }
			else 
			{
				  TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
          Drop[3]=TIM_GetCapture4(TIM2);
				  if(Rise[3]>Drop[3])  RC_Pwm_In[3] = 65535-Rise[3] + Drop[3];
					else 	               RC_Pwm_In[3] = Drop[3] - Rise[3];
      }	  
		}		
}



