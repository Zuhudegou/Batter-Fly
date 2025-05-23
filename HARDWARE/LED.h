#ifndef __LED_H
#define __LED_H

#include "sys.h"
#include "GPIO.H"

#define     LED1(x)       x? (GPIOB->BSRR=GPIO_Pin_8):(GPIOB->BRR=GPIO_Pin_8)
#define     LED2(x)       x? (GPIOB->BSRR=GPIO_Pin_5):(GPIOB->BRR=GPIO_Pin_5)
#define     LED3(x)       x? (GPIOB->BSRR=GPIO_Pin_10):(GPIOB->BRR=GPIO_Pin_10)
#define     LED4(x)       x? (GPIOB->BSRR=GPIO_Pin_9):(GPIOB->BRR=GPIO_Pin_9)
#define     OP_LED1       (GPIOB->IDR&GPIO_Pin_8)? (GPIOB->BRR=GPIO_Pin_8):(GPIOB->BSRR=GPIO_Pin_8)
#define     OP_LED2       (GPIOB->IDR&GPIO_Pin_5)? (GPIOB->BRR=GPIO_Pin_5):(GPIOB->BSRR=GPIO_Pin_5)
#define     OP_LED3       (GPIOB->IDR&GPIO_Pin_10)? (GPIOB->BRR=GPIO_Pin_10):(GPIOB->BSRR=GPIO_Pin_10)
#define     OP_LED4       (GPIOB->IDR&GPIO_Pin_9)? (GPIOB->BRR=GPIO_Pin_9):(GPIOB->BSRR=GPIO_Pin_9)




//1 2 3 4 -> b f h a

//机身前灯	，蓝 
#define fLED_H()  		 LED_GPIOB->BSRR = fLED_io
#define fLED_L()  		 LED_GPIOB->BRR  = fLED_io
#define fLED_Toggle()  LED_GPIOB->ODR ^= fLED_io

#define hLED_H()  		 LED_GPIOB->BSRR = hLED_io
#define hLED_L()  		 LED_GPIOB->BRR  = hLED_io
#define hLED_Toggle()  LED_GPIOB->ODR ^= hLED_io

//-------------------------------------------------
//机身后灯，红	 
#define aLED_H()  		 LED_GPIOB->BSRR = aLED_io
#define aLED_L()  		 LED_GPIOB->BRR  = aLED_io
#define aLED_Toggle()  LED_GPIOB->ODR ^= aLED_io

#define bLED_H()  		 LED_GPIOB->BSRR = bLED_io
#define bLED_L()  		 LED_GPIOB->BRR  = bLED_io
#define bLED_Toggle()  LED_GPIOB->ODR ^= bLED_io
//-------------------------------------------------

typedef struct
{
	uint16_t FlashTime;
	
	
	enum
	{
		AlwaysOn,  			//0.全亮，姿态解锁
		AlwaysOff, 			//1.全暗
		AllFlashLight, 	//2.闪烁，默认
		AlternateFlash, //3.交替闪烁，rf初始化后，失联后
		SimpleFlash,		//4.新增，同步闪烁，收到遥控器数据
		WARNING,				//5.慢闪，未用
		DANGEROURS,			//6.常亮 2秒 闪烁 1秒，悬停解锁
		GET_OFFSET	//7
	}status; 
	
	u8 off; //临时关闭led
	u8 to_off; //已打开，需要关闭
	
}sLED;	

extern sLED LED;
extern void LEDInit(void);
extern void LEDtest(void);
extern void PilotLED(void);
extern void CloseLED();
extern u8 LED_warn;

#define LED_TAKE_OFF_ENTER      LED.status = WARNING  
#define LED_TAKE_OFF_EXIT       LED.status = AllFlashLight  
#define LED_HEIGHT_LOCK_ENTER   LED.FlashTime=50;LED.status = AlternateFlash
#define LED_HEIGHT_LOCK_EXIT    LED.FlashTime=100;LED.status = AllFlashLight
#define LED_3D_ROLL_ENTER				LED.status = WARNING 
#define LED_3D_ROLL_EXIT        LED.status = AllFlashLight  
#define LED_SAFTY_TAKE_DOWN_ENTER  LED.status = DANGEROURS 
#define LED_SAFTY_TAKE_DOWN_EXIT   LED.status = AlwaysOn
#define LED_GET_MPU_OFFSET_ENTER   LED.status = GET_OFFSET
#define LED_GO_HOME_ENTER       LED.status = WARNING    
#define LED_GO_HOME_EXIT        LED.status = AllFlashLight  

#endif 


