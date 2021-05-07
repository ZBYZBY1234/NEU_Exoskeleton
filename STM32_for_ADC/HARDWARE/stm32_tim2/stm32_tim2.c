#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32_tim2.h"
#include "misc.h"

/* TIM2中断优先级配置 */
void TIM2_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure; 
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;//中断种类  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//定义此中断的抢占优先级别
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//定义此中断的响应优先级别
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);	
}


//初始化TIM2定时器及中断
//每次进入中断服务程序间隔时间为
//((1+arr )/72M)*(1+psc )=((1+1999)/72M)*(1+35999)=1秒
void TIM2_Config(u16 arr,u16 psc)
{
    //定时器时间是1s

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    //清除中断标志位
    TIM_ClearITPendingBit(TIM2,TIM_IT_Update);//TIM_ClearFlag(TIM2,TIM_FLAG_Update);//两者作用相同

    //自动装载的周期值0-0xffff，72M/36000=2000
    TIM_TimeBaseStructure.TIM_Period = arr;
    //时钟除数预分频值0-oxffff，下面是36000分频
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    ////普通和高级才有下面两行
    //时钟分割，暂时为0，高级应用才用
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    //计数模式，向上下，中央对齐123
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

    //开启中断
    TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
    //开启外设
    TIM_Cmd(TIM2,ENABLE);
}

