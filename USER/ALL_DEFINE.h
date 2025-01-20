#ifndef __ALL_DEFINE_H__
#define __ALL_DEFINE_H__
#include "stm32f10x.h"
#include "ALL_DATA.h"
#include "INIT.h"
#include "sys.h"
#include "I2C.h"
#include "SPI.h"
#include "nrf24l01.h"
#include "USART3.h"
#include  "TIM.h"
#include "LED.h"
#include "mpu6050.h"
#include "imu.h"
#include "ANO_DT.h"
#include "Remote.h"
#include "control.h"
#include "myMath.h"
#include "Uart1.h"
#include "USART2.h"
#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1


/***************UART1 GPIO����******************/
#define RCC_UART1		RCC_APB2Periph_GPIOA
#define GPIO_UART1		GPIOA
#define UART1_Pin_TX	GPIO_Pin_9
#define UART1_Pin_RX	GPIO_Pin_10
/*********************************************/
/***************UART2 GPIO����******************/
#define RCC_UART2		RCC_APB2Periph_GPIOA
#define GPIO_UART2		GPIOA
#define UART2_Pin_TX	GPIO_Pin_2
#define UART2_Pin_RX	GPIO_Pin_3
/*********************************************/
/***************UART3 GPIO����******************/
#define RCC_UART3		RCC_APB2Periph_GPIOB
#define GPIO_UART3		GPIOB
#define UART3_Pin_TX	GPIO_Pin_10
#define UART3_Pin_RX	GPIO_Pin_11
/*********************************************/



/***************Ӳ���ж����ȼ�******************/
#define NVIC_UART1_P	5
#define NVIC_UART1_S	1
#define NVIC_UART2_P	4
#define NVIC_UART2_S	1
#define NVIC_UART3_P	3
#define NVIC_UART3_S	1
/***********************************************/


#define MOTOR_NUM 4

#define FLY_TYPE   2    //�ɿ�����(1��С����;  2��С����270��8520���ı����-3.7v-52000rpm�����ٱ�64:11����  3 ����ˢ330)  //1��2Ϊ���ı������3Ϊ��ˢ���
//#define FLY_TYPE   3 //��ˢ��1503-1s-3.7v��ˢ���-4200KV-15500rpm��


#if (FLY_TYPE == 1)   //1��С����; 

	/***************PWM GPIO����******************/	
	#define PWM_GPA  GPIOA
	#define PWM_GPB  GPIOB	
	#define PWM1_io    GPIO_Pin_0		//PWM
	#define PWM2_io    GPIO_Pin_1		//PWM
	#define PWM3_io    GPIO_Pin_2		//PWM
	#define PWM4_io    GPIO_Pin_3		//PWM
	#define PWM5_io    GPIO_Pin_4		//PWM
	#define PWM6_io    GPIO_Pin_5		//PWM
	
	/***************NRF GPIO����******************/
	#define NRF_CE_GP  GPIOA
	#define NRF_CSN_GP  GPIOB
	#define NRF_IRQ_GP  GPIOA

	#define NRF24L01_CE     GPIO_Pin_11		//CE PA11
	#define NRF24L01_CSN    GPIO_Pin_12		//CSN PB12
	#define NRF24L01_IRQ    GPIO_Pin_12		//IRQ PA12
	
	/***************LED GPIO����******************/
	#define LED_GPIOB  GPIOB
	#define fLED_io    GPIO_Pin_1		//������	
	#define hLED_io    GPIO_Pin_2		//������	
	#define aLED_io    GPIO_Pin_8		//����ǰ��	
	#define bLED_io    GPIO_Pin_9		//����ǰ��	
	/*********************************************/
	
	
	#define TIM2_DUTY    0      //��ʼռ�ձ�
	#define TIM2_PWM_MAX 999    //��������
	#define TIM2_PWM_HZ  18000  //PWMƵ��

	#define TIM3_DUTY    0      //��ʼռ�ձ�
	#define TIM3_PWM_MAX 999    //��������
	#define TIM3_PWM_HZ  18000  //PWMƵ��
	
	#define PWM0 TIM2->CCR1   //��ʱ��2 ͨ��1
	#define PWM1 TIM2->CCR2   //��ʱ��2 ͨ��2

	#define PWM2 TIM2->CCR3   //��ʱ��2 ͨ��3
	#define PWM3 TIM2->CCR4   //��ʱ��2 ͨ��4


//#elif (FLY_TYPE == 2)    //2��С����270��
#elif (FLY_TYPE == 2 || FLY_TYPE >= 3)    //2��С����270������ˢ330
	
	/***************PWM GPIO����******************/	
	#define PWM_GPA  GPIOA
	#define PWM_GPB  GPIOB		 
	#define PWM1_io    GPIO_Pin_0		//M1��PWM0  = A0
	#define PWM2_io    GPIO_Pin_1		//M2��PWM1	=	A1
	#define PWM3_io    GPIO_Pin_0		//PWM
	#define PWM4_io    GPIO_Pin_1		//PWM
	#define PWM5_io    GPIO_Pin_4		//M3��PWM2	=	B4
	#define PWM6_io    GPIO_Pin_5		//M4��PWM3	=	B5
		
	/***************NRF GPIO����******************/
	#define NRF_CE_GP  GPIOA
	#define NRF_CSN_GP  GPIOB
	#define NRF_IRQ_GP  GPIOB

	#define NRF24L01_CE     GPIO_Pin_8		  //CE PA8
	#define NRF24L01_CSN    GPIO_Pin_12		//CSN PB12
	#define NRF24L01_IRQ    GPIO_Pin_2		//IRQ PB2
	
	/***************LED GPIO����******************/
	#define LED_GPIOB  GPIOB
	
	/*
	#define bLED_io    GPIO_Pin_9		//�����ƣ���	  LED1		LED4
	#define fLED_io    GPIO_Pin_1		//�����ƣ���	  LED2   	LED1
	#define hLED_io    GPIO_Pin_3		//����ǰ�ƣ���   LED3		LED2
	#define aLED_io    GPIO_Pin_8		//����ǰ�ƣ���		LED4		LED3
	*/
	
	/*
	#define bLED_io    GPIO_Pin_1		//�����ƣ���	  LED1		LED1    
	#define fLED_io    GPIO_Pin_3		//�����ƣ���	  LED2   	LED2     
	#define hLED_io    GPIO_Pin_8		//����ǰ�ƣ���   LED3		LED3     
	#define aLED_io    GPIO_Pin_9		//����ǰ�ƣ���	 	LED4		LED4    
	*/
	

	#define bLED_io    GPIO_Pin_9		//�����ƣ���	  LED1		LED1    
	#define fLED_io    GPIO_Pin_1		//�����ƣ���	  LED2   	LED2     
	#define hLED_io    GPIO_Pin_3		//����ǰ�ƣ���   LED3		LED3     
	#define aLED_io    GPIO_Pin_8		//����ǰ�ƣ���	 	LED4		LED4    

	
	/*********************************************/
	

	
	#define PWM0 TIM2->CCR1   //��ʱ��2 ͨ��1    M1
	#define PWM1 TIM2->CCR2		//��ʱ��2 ͨ��2		M2

	#define PWM2 TIM3->CCR1   //��ʱ��3 ͨ��1		M3
	#define PWM3 TIM3->CCR2		//��ʱ��3 ͨ��2		M4
	

	//���ı�VS��ˢ, PWM������ͬ
	/*********************************************/
	
	
	#if (FLY_TYPE == 2)   //2��С����270��

	#define TIM2_DUTY    0      //��ʼռ�ձ�
	#define TIM2_PWM_MAX 999    //��������
	#define TIM2_PWM_HZ  18000  //PWMƵ��

	#define TIM3_DUTY    0      //��ʼռ�ձ�
	#define TIM3_PWM_MAX 999    //��������
	#define TIM3_PWM_HZ  18000  //PWMƵ��


		
	
  //#elif (FLY_TYPE >= 3)   
	#else 		//3 ����ˢ330

		//����Ƶ�� 400Hz��
		//������С���Ϊ1000us (ռ�ձ�40%���������һ��ת��Ҫ43%��ת)
		//��������Ϊ2500us�� 
		//ʵ�����õ�������Ϊ2000us(ռ�ձ�80%, 80%~95%����һ�����ٶȣ������ת��)

		#define TIM2_DUTY    1000     //��ʼֵ��Ҳ����С�����ռ�ձȣ�1000/2500 = 40%
		#define TIM2_PWM_MAX (2500-1) //�������ޣ�Ҳ��pwm���ڣ�������Ϊ 2000/2500 = 80%
		#define TIM2_PWM_HZ  400      //PWMƵ��

		#define TIM3_DUTY    1000      //��ʼռ�ձ�
		#define TIM3_PWM_MAX (2500-1) //��������
		#define TIM3_PWM_HZ  400      //PWMƵ��		
		


	#endif	






	
//  #elif (FLY_TYPE >= 3)    //3 ����ˢ330)
//	
//	#define TIM2_DUTY    1000     //��ʼռ�ձ�
//	#define TIM2_PWM_MAX (2500-1) //��������
//	#define TIM2_PWM_HZ  400      //PWMƵ��

//	#define TIM3_DUTY    1000     //��ʼռ�ձ�
//	#define TIM3_PWM_MAX (2500-1) //��������
//	#define TIM3_PWM_HZ  400      //PWMƵ��		

#else
	#error Please define FLY_TYPE!
#endif	

//extern uint32_t Control_high;
extern float Control_high;

extern volatile uint32_t SysTick_count;   //ϵͳ�δ�ʱ�Ӽ���

extern volatile uint8_t spl_flag;
#endif

