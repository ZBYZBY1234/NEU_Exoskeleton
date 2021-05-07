#include "stm32f10x.h"
#include "stm32_dac.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_dac.h"
/*DAC输出 = Vref x (DOR/4095)*/ 
//DAC的两个通道可以配置使用
//相同触发源/不同触发源
//同时触发/独立触发    DAC_DualSoftwareTriggerCmd函数设置软件同时触发
//使用波形发生器/不使用波形发生器
//使用三角波发生器/使用噪声发生器/不使用波形发生器
//设置相同DAC_LFSRUnmask_TriangleAmplitude的值/设置不相同DAC_LFSRUnmask_TriangleAmplitude的值
//等等以上各种情况可以任意组合，互不影响。
void DAC_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	DAC_InitTypeDef DAC_InitStruct;
	//第一步  使能时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC,ENABLE);
	//第二步  配置参数
	/*一旦使能DACx通道，相应的GPIO引脚就会自动与DAC的模拟输出相连，为了避免寄生的干扰和额外的功耗，引脚PA4/PA5在之前应当设置成“模拟输入”
		注意是“模拟输入“，因为STM32中没有模拟输出，所以虽然PA4 PA5是输出模拟信号，也只能设置成GPIO_Mode_AIN*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	GPIO_SetBits(GPIOA,GPIO_Pin_4 | GPIO_Pin_5)	;//PA.4  PA.5输入高	，上拉输入起抗干扰的作用
	
//	/*DAC 通道1  PA4 产生噪声*/
//	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_Noise;
//	DAC_InitStruct.DAC_Trigger = DAC_Trigger_T6_TRGO;//DAC_Trigger_T6_TRGO;
//	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Disable;//输出缓存可以用来减少输出阻抗，无需外部运放即可直接驱动外部负载
//	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bits10_0;//每次触发计算一次LSFR算法，并将得到的值再加上DAC_DHRx的数值，去掉溢出位后写入DAC_DORx寄存器，输出特定的电压
//	DAC_Init(DAC_Channel_1,&DAC_InitStruct);//参与LSFR算法的位数由DAC_LFSRUnmask_TriangleAmplitude来确定，DAC_LFSRUnmask_Bits10_0数值表示有10位参与LSFR计算
	
	/*DAC 通道1  PA4 普通数模转换*/
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;//关闭波形发生器
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_T6_TRGO;//DAC_Trigger_Software/DAC_Trigger_Ext_IT9
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Disable;//输出缓存可以用来减少输出阻抗，无需外部运放即可直接驱动外部负载
	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;//该参数与噪声/三角波发生器相关，普通DAC转换是设置为0即可
	DAC_Init(DAC_Channel_1,&DAC_InitStruct);																 
 
	/*DAC 通道2  PA5 普通数模转换*/
	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_None;//关闭波形发生器
	DAC_InitStruct.DAC_Trigger = DAC_Trigger_T6_TRGO;//DAC_Trigger_Software/DAC_Trigger_Ext_IT9
	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Disable;//输出缓存可以用来减少输出阻抗，无需外部运放即可直接驱动外部负载
	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;//该参数与噪声/三角波发生器相关，普通DAC转换是设置为0即可
	DAC_Init(DAC_Channel_2,&DAC_InitStruct);	

	/*DAC 通道2  PA5 产生三角波*/
//	DAC_InitStruct.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;
//	DAC_InitStruct.DAC_Trigger = DAC_Trigger_T6_TRGO;
//	DAC_InitStruct.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
//	DAC_InitStruct.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_4095;//内部的三角波计数器每次触发时候之后累加1，该计数器的值与DAC_DHRx的数值相加，去掉溢出位后写入DAC_DORx寄存器，输出电压
//	DAC_Init(DAC_Channel_2,&DAC_InitStruct);//三角波计数器的最大值由DAC_LFSRUnmask_TriangleAmplitude来确定，当计数器达到这个最大值，然后三角波计数器开始递减

	
	//第三步  使能器件
	//DAC_SetDualChannelData(DAC_Align_12b_R,4095,0);等价于DAC_SetChannel1Data(DAC_Align_12b_R, 4095); DAC_SetChannel2Data(DAC_Align_12b_R, 0);  
	/*DAC 通道1  PA4 使能*/
	//DAC_SetChannel1Data(DAC_Align_12b_R, 2048);  //12位右对齐数据格式设置DAC值  设置值最大为4095，设置成4096则溢出，DORx即为0
	DAC_Cmd(DAC_Channel_1, ENABLE);  //使能DAC1
	
	/*DAC 通道2  PA5 使能*/
	DAC_Cmd(DAC_Channel_2, ENABLE);  //使能DAC2
//	DAC_SetChannel2Data(DAC_Align_12b_R, 0);  //12位右对齐数据格式设置DAC值
}
 
 
//基本定时器
void TIM6_Configuration()
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
		//第一步  使能时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //时钟使能
 
		//第二步 配置参数  （10-1+1）*（72-1+1)/72000000=0.00001 秒 10e-5
		TIM_TimeBaseInitStruct.TIM_Period = 10 -1;
		TIM_TimeBaseInitStruct.TIM_Prescaler = 71;
		TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
		TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStruct);//  TIMx->EGR.UG   
 
	
		/*TIM6,7可以输出3种类型的TRGO信号
			#define TIM_TRGOSource_Reset               ((uint16_t)0x0000) //复位 UG
			#define TIM_TRGOSource_Enable              ((uint16_t)0x0010) //使能 CEN
			#define TIM_TRGOSource_Update              ((uint16_t)0x0020) //更新事件
		*/
		
		TIM_SelectOutputTrigger(TIM6,TIM_TRGOSource_Update);//输出触发TRGO信号  这里TRGO信号就是定时器溢出产生的更新信号
		
		//第三步  使能器件
		TIM_Cmd(TIM6,ENABLE);//CEN  位
}

//设置通道1输出电压
//vol:0~3300,代表0~3.3V
void Dac1_Set_Vol(u16 vol)
{
	float temp=vol;
	temp/=1000;
	temp=temp*4096/3.3;
	DAC_SetChannel1Data(DAC_Align_12b_R,temp);//12位右对齐数据格式设置DAC值
}


