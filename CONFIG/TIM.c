/**********************STM32 开源无人机*******************************************************/
//  V1.0 开源作者：小南&zin；日期：2016.11.21
//           STM32F103C8飞控以及遥控基础功能以及核心代码实现；
//  V2.0 开源作者：小刘；日期：2020.05.17
//           scheduler任务架构调整，增加屏幕以及气压计，新增PID在线调整功能；
//  V3.0 开源作者：zhibo_sz&sunsp；日期：2024.06.01
//           新增一键定高起飞，悬停运动控制以及刹车优化，气压遥控屏幕陀螺仪等模块优化，新增无刷电机；
/********************************************************************************************/

//声明：
//      本程序仅对购机用户开源，学习使用，所有权归以上作者所有；
//      未经许可，不得传阅、转载、公开、转卖本代码。


#include "stm32f10x.h"
#include "sys.h"
#include "TIM.h"
#include "LED.h"
#include "ALL_DATA.h" 
#include "ALL_DEFINE.h" 
#include "control.h"
	
//定时器2，给马达0(PWM0)和马达1(PWM1)使用
//管脚：A0，A1
	
void TIM2_PWM_Config(void)
{
   uint16_t TIM_Prescaler;
	
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  /* 使能GPIOA时钟时钟 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = PWM1_io | PWM2_io | PWM3_io| PWM4_io;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(PWM_GPA, &GPIO_InitStructure);
  /* 使能定时器2时钟 */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  /* Time base configuration */
	TIM_Prescaler = SystemCoreClock/(TIM2_PWM_MAX+1)/TIM2_PWM_HZ -1; //pwm频率18KHz，无刷为400Hz
	
  TIM_TimeBaseStructure.TIM_Period = TIM2_PWM_MAX; //定时器计数周期 0-999  1000	 计数上限，无刷为2499
  TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler; //设置预分频：根据pwm频率和计数周期
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分频系数：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //配置为PWM模式1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_Pulse =  TIM2_DUTY;  //初始占空比为0，无刷为1000
    //设置跳变值，当计数器计数到这个值时，电平发生跳变(即占空比) 初始值0
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    //当定时器计数值小于定时设定值时为高电平
  /* 使能通道1 */
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
  /* 使能通道2 */
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  /* 使能通道3 */
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
  /* 使能通道4 */
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM2, ENABLE); // 使能TIM2重载寄存器ARR
  TIM_Cmd(TIM2, ENABLE); //使能定时器2
}


//定时器2，给马达2(PWM2)和马达3(PWM3)使用
//管脚：B4，B5

void TIM3_PWM_Config(void)
{
	uint16_t TIM_Prescaler;
	
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	GPIO_InitTypeDef 					GPIO_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 
	
	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);
	GPIO_InitStructure.GPIO_Pin = PWM5_io | PWM6_io;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_Init(PWM_GPB, &GPIO_InitStructure); 

	TIM_Prescaler = SystemCoreClock/(TIM3_PWM_MAX+1)/TIM3_PWM_HZ -1; //pwm频率18KHz，无刷为400Hz
	TIM_TimeBaseStructure.TIM_Period = TIM3_PWM_MAX; //计数上限 999，无刷为2499
	TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler;

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM3_DUTY;  //初始占空比为0，无刷为1000
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}



////通用定时器中断初始化
////这里时钟选择为APB1的2倍，而APB1为36M
////arr：自动重装值。
////psc：时钟预分频数
////这里使用的是定时器3!
//void TIM3_Config(void)
//{
//  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure; 
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能
//	
//	
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级1级
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
//	
//	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
//	TIM_TimeBaseStructure.TIM_Period = 299; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到300为3ms
//	TIM_TimeBaseStructure.TIM_Prescaler =719; //设置用来作为TIMx时钟频率除数的预分频值  100Khz的计数频率  
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
// 
//	TIM_ITConfig(  //使能或者失能指定的TIM中断
//		TIM3, //TIM2
//		TIM_IT_Update ,
//		ENABLE  //使能
//		);


//	TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设
//							 
//}













