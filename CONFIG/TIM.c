/**********************STM32 ��Դ���˻�*******************************************************/
//  V1.0 ��Դ���ߣ�С��&zin�����ڣ�2016.11.21
//           STM32F103C8�ɿ��Լ�ң�ػ��������Լ����Ĵ���ʵ�֣�
//  V2.0 ��Դ���ߣ�С�������ڣ�2020.05.17
//           scheduler����ܹ�������������Ļ�Լ���ѹ�ƣ�����PID���ߵ������ܣ�
//  V3.0 ��Դ���ߣ�zhibo_sz&sunsp�����ڣ�2024.06.01
//           ����һ��������ɣ���ͣ�˶������Լ�ɲ���Ż�����ѹң����Ļ�����ǵ�ģ���Ż���������ˢ�����
/********************************************************************************************/

//������
//      ��������Թ����û���Դ��ѧϰʹ�ã�����Ȩ�������������У�
//      δ����ɣ����ô��ġ�ת�ء�������ת�������롣


#include "stm32f10x.h"
#include "sys.h"
#include "TIM.h"
#include "LED.h"
#include "ALL_DATA.h" 
#include "ALL_DEFINE.h" 
#include "control.h"
	
//��ʱ��2�������0(PWM0)�����1(PWM1)ʹ��
//�ܽţ�A0��A1
	
void TIM2_PWM_Config(void)
{
   uint16_t TIM_Prescaler;
	
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  /* ʹ��GPIOAʱ��ʱ�� */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitStructure.GPIO_Pin = PWM1_io | PWM2_io | PWM3_io| PWM4_io;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(PWM_GPA, &GPIO_InitStructure);
  /* ʹ�ܶ�ʱ��2ʱ�� */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  /* Time base configuration */
	TIM_Prescaler = SystemCoreClock/(TIM2_PWM_MAX+1)/TIM2_PWM_HZ -1; //pwmƵ��18KHz����ˢΪ400Hz
	
  TIM_TimeBaseStructure.TIM_Period = TIM2_PWM_MAX; //��ʱ���������� 0-999  1000	 �������ޣ���ˢΪ2499
  TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler; //����Ԥ��Ƶ������pwmƵ�ʺͼ�������
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷ�Ƶϵ��������Ƶ
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //���ϼ���ģʽ

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* PWM1 Mode configuration: Channel */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //����ΪPWMģʽ1
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  //TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_Pulse =  TIM2_DUTY;  //��ʼռ�ձ�Ϊ0����ˢΪ1000
    //��������ֵ�������������������ֵʱ����ƽ��������(��ռ�ձ�) ��ʼֵ0
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    //����ʱ������ֵС�ڶ�ʱ�趨ֵʱΪ�ߵ�ƽ
  /* ʹ��ͨ��1 */
  TIM_OC1Init(TIM2, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
  /* ʹ��ͨ��2 */
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  /* ʹ��ͨ��3 */
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
  /* ʹ��ͨ��4 */
  TIM_OC4Init(TIM2, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM2, ENABLE); // ʹ��TIM2���ؼĴ���ARR
  TIM_Cmd(TIM2, ENABLE); //ʹ�ܶ�ʱ��2
}


//��ʱ��2�������2(PWM2)�����3(PWM3)ʹ��
//�ܽţ�B4��B5

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

	TIM_Prescaler = SystemCoreClock/(TIM3_PWM_MAX+1)/TIM3_PWM_HZ -1; //pwmƵ��18KHz����ˢΪ400Hz
	TIM_TimeBaseStructure.TIM_Period = TIM3_PWM_MAX; //�������� 999����ˢΪ2499
	TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler;

	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = TIM3_DUTY;  //��ʼռ�ձ�Ϊ0����ˢΪ1000
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



////ͨ�ö�ʱ���жϳ�ʼ��
////����ʱ��ѡ��ΪAPB1��2������APB1Ϊ36M
////arr���Զ���װֵ��
////psc��ʱ��Ԥ��Ƶ��
////����ʹ�õ��Ƕ�ʱ��3!
//void TIM3_Config(void)
//{
//  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure; 
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //ʱ��ʹ��
//	
//	
//	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�1��
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
//	
//	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
//	TIM_TimeBaseStructure.TIM_Period = 299; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 ������300Ϊ3ms
//	TIM_TimeBaseStructure.TIM_Prescaler =719; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  100Khz�ļ���Ƶ��  
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
//	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
// 
//	TIM_ITConfig(  //ʹ�ܻ���ʧ��ָ����TIM�ж�
//		TIM3, //TIM2
//		TIM_IT_Update ,
//		ENABLE  //ʹ��
//		);


//	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIMx����
//							 
//}













