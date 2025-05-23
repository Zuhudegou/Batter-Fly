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


/***************UART1 GPIO定义******************/
#define RCC_UART1		RCC_APB2Periph_GPIOA
#define GPIO_UART1		GPIOA
#define UART1_Pin_TX	GPIO_Pin_9
#define UART1_Pin_RX	GPIO_Pin_10
/*********************************************/
/***************UART2 GPIO定义******************/
#define RCC_UART2		RCC_APB2Periph_GPIOA
#define GPIO_UART2		GPIOA
#define UART2_Pin_TX	GPIO_Pin_2
#define UART2_Pin_RX	GPIO_Pin_3
/*********************************************/
/***************UART3 GPIO定义******************/
#define RCC_UART3		RCC_APB2Periph_GPIOB
#define GPIO_UART3		GPIOB
#define UART3_Pin_TX	GPIO_Pin_10
#define UART3_Pin_RX	GPIO_Pin_11
/*********************************************/



/***************硬件中断优先级******************/
#define NVIC_UART1_P	5
#define NVIC_UART1_S	1
#define NVIC_UART2_P	4
#define NVIC_UART2_S	1
#define NVIC_UART3_P	3
#define NVIC_UART3_S	1
/***********************************************/


#define MOTOR_NUM 4

#define FLY_TYPE   2    //飞控类型(1、小四轴;  2、小四轴270（8520空心杯电机-3.7v-52000rpm，减速比64:11）；  3 、无刷330)  //1和2为空心杯电机，3为无刷电机
//#define FLY_TYPE   3 //无刷（1503-1s-3.7v无刷电机-4200KV-15500rpm）


#if (FLY_TYPE == 1)   //1、小四轴; 

	/***************PWM GPIO定义******************/	
	#define PWM_GPA  GPIOA
	#define PWM_GPB  GPIOB	
	#define PWM1_io    GPIO_Pin_0		//PWM
	#define PWM2_io    GPIO_Pin_1		//PWM
	#define PWM3_io    GPIO_Pin_2		//PWM
	#define PWM4_io    GPIO_Pin_3		//PWM
	#define PWM5_io    GPIO_Pin_4		//PWM
	#define PWM6_io    GPIO_Pin_5		//PWM
	
	/***************NRF GPIO定义******************/
	#define NRF_CE_GP  GPIOA
	#define NRF_CSN_GP  GPIOB
	#define NRF_IRQ_GP  GPIOA

	#define NRF24L01_CE     GPIO_Pin_11		//CE PA11
	#define NRF24L01_CSN    GPIO_Pin_12		//CSN PB12
	#define NRF24L01_IRQ    GPIO_Pin_12		//IRQ PA12
	
	/***************LED GPIO定义******************/
	#define LED_GPIOB  GPIOB
	#define fLED_io    GPIO_Pin_1		//机身后灯	
	#define hLED_io    GPIO_Pin_2		//机身后灯	
	#define aLED_io    GPIO_Pin_8		//机身前灯	
	#define bLED_io    GPIO_Pin_9		//机身前灯	
	/*********************************************/
	
	
	#define TIM2_DUTY    0      //初始占空比
	#define TIM2_PWM_MAX 999    //计数上限
	#define TIM2_PWM_HZ  18000  //PWM频率

	#define TIM3_DUTY    0      //初始占空比
	#define TIM3_PWM_MAX 999    //计数上限
	#define TIM3_PWM_HZ  18000  //PWM频率
	
	#define PWM0 TIM2->CCR1   //定时器2 通道1
	#define PWM1 TIM2->CCR2   //定时器2 通道2

	#define PWM2 TIM2->CCR3   //定时器2 通道3
	#define PWM3 TIM2->CCR4   //定时器2 通道4


//#elif (FLY_TYPE == 2)    //2、小四轴270；
#elif (FLY_TYPE == 2 || FLY_TYPE >= 3)    //2、小四轴270或者无刷330
	
	/***************PWM GPIO定义******************/	
	#define PWM_GPA  GPIOA
	#define PWM_GPB  GPIOB		 
	#define PWM1_io    GPIO_Pin_0		//M1：PWM0  = A0
	#define PWM2_io    GPIO_Pin_1		//M2：PWM1	=	A1
	#define PWM3_io    GPIO_Pin_0		//PWM
	#define PWM4_io    GPIO_Pin_1		//PWM
	#define PWM5_io    GPIO_Pin_4		//M3：PWM2	=	B4
	#define PWM6_io    GPIO_Pin_5		//M4：PWM3	=	B5
		
	/***************NRF GPIO定义******************/
	#define NRF_CE_GP  GPIOA
	#define NRF_CSN_GP  GPIOB
	#define NRF_IRQ_GP  GPIOB

	#define NRF24L01_CE     GPIO_Pin_8		  //CE PA8
	#define NRF24L01_CSN    GPIO_Pin_12		//CSN PB12
	#define NRF24L01_IRQ    GPIO_Pin_2		//IRQ PB2
	
	/***************LED GPIO定义******************/
	#define LED_GPIOB  GPIOB
	
	/*
	#define bLED_io    GPIO_Pin_9		//机身后灯，红	  LED1		LED4
	#define fLED_io    GPIO_Pin_1		//机身后灯，红	  LED2   	LED1
	#define hLED_io    GPIO_Pin_3		//机身前灯，蓝   LED3		LED2
	#define aLED_io    GPIO_Pin_8		//机身前灯，蓝		LED4		LED3
	*/
	
	/*
	#define bLED_io    GPIO_Pin_1		//机身后灯，红	  LED1		LED1    
	#define fLED_io    GPIO_Pin_3		//机身后灯，红	  LED2   	LED2     
	#define hLED_io    GPIO_Pin_8		//机身前灯，蓝   LED3		LED3     
	#define aLED_io    GPIO_Pin_9		//机身前灯，蓝	 	LED4		LED4    
	*/
	

	#define bLED_io    GPIO_Pin_9		//机身后灯，红	  LED1		LED1    
	#define fLED_io    GPIO_Pin_1		//机身后灯，红	  LED2   	LED2     
	#define hLED_io    GPIO_Pin_3		//机身前灯，蓝   LED3		LED3     
	#define aLED_io    GPIO_Pin_8		//机身前灯，蓝	 	LED4		LED4    

	
	/*********************************************/
	

	
	#define PWM0 TIM2->CCR1   //定时器2 通道1    M1
	#define PWM1 TIM2->CCR2		//定时器2 通道2		M2

	#define PWM2 TIM3->CCR1   //定时器3 通道1		M3
	#define PWM3 TIM3->CCR2		//定时器3 通道2		M4
	

	//空心杯VS无刷, PWM参数不同
	/*********************************************/
	
	
	#if (FLY_TYPE == 2)   //2、小四轴270；

	#define TIM2_DUTY    0      //初始占空比
	#define TIM2_PWM_MAX 999    //计数上限
	#define TIM2_PWM_HZ  18000  //PWM频率

	#define TIM3_DUTY    0      //初始占空比
	#define TIM3_PWM_MAX 999    //计数上限
	#define TIM3_PWM_HZ  18000  //PWM频率


		
	
  //#elif (FLY_TYPE >= 3)   
	#else 		//3 、无刷330

		//设置频率 400Hz，
		//设置最小输出为1000us (占空比40%，有输出不一定转，要43%才转)
		//设置周期为2500us， 
		//实际有用的最大输出为2000us(占空比80%, 80%~95%都是一样的速度，即最高转速)

		#define TIM2_DUTY    1000     //初始值，也是最小输出，占空比：1000/2500 = 40%
		#define TIM2_PWM_MAX (2500-1) //计数上限，也是pwm周期，最大输出为 2000/2500 = 80%
		#define TIM2_PWM_HZ  400      //PWM频率

		#define TIM3_DUTY    1000      //初始占空比
		#define TIM3_PWM_MAX (2500-1) //计数上限
		#define TIM3_PWM_HZ  400      //PWM频率		
		


	#endif	






	
//  #elif (FLY_TYPE >= 3)    //3 、无刷330)
//	
//	#define TIM2_DUTY    1000     //初始占空比
//	#define TIM2_PWM_MAX (2500-1) //计数上限
//	#define TIM2_PWM_HZ  400      //PWM频率

//	#define TIM3_DUTY    1000     //初始占空比
//	#define TIM3_PWM_MAX (2500-1) //计数上限
//	#define TIM3_PWM_HZ  400      //PWM频率		

#else
	#error Please define FLY_TYPE!
#endif	

//extern uint32_t Control_high;
extern float Control_high;

extern volatile uint32_t SysTick_count;   //系统滴答时钟计数

extern volatile uint8_t spl_flag;
#endif

