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
#include "LED.h"
#include "ALL_DATA.h"
#include "ALL_DEFINE.h" 
//---------------------------------------------------------
/*     you can select the LED statue on enum contains            */

//默认为0.3秒的快闪
sLED LED = {300,AllFlashLight};  //LED initial statue is off;
u8 LED_warn;                             //default 300ms flash the status

u16 falg_Time=0;

/**************************************************************
 *  LED Init
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
void LEDInit(void)	
{	
		GPIO_InitTypeDef GPIO_InitStructure;
		//AFIO->MAPR = 0X02000000; //使能4线烧写 释放某些与烧写相关的引脚,,,这个要注释掉，不然需要重新执行 TIM3_PWM_Config()
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);
	  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  	GPIO_InitStructure.GPIO_Pin = fLED_io | hLED_io|aLED_io | bLED_io;		     //LED12
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(LED_GPIOB, &GPIO_InitStructure); 	 
	
}
/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/
//新增
void CloseLED() //2ms调用1次
{
	if( LED.to_off && LED.off ){
		LED.to_off=0;
		bLED_H();
		fLED_H();
		aLED_H();
		hLED_H();
	}
	

}



/**************************************************************
 *  LED system
 * @param[in] 
 * @param[out] 
 * @return     
 ***************************************************************/	
void PilotLED() //flash 300MS interval
{
	static uint32_t LastTime = 0;
  		
	


	
	if(SysTick_count - LastTime < LED.FlashTime)
	{
			//未到切换间隔时间
			return;
	}
	else
			LastTime = SysTick_count;
	
	
	//50ms 调用一次
	if(LED.off){
			static u16 cnt;
			cnt++;
			//使用delay1ms会影响飞行
			switch(cnt%24)
			{
				//从右下角开始，先逆时针，再顺时针
				case 0:
				case 12:
					bLED_L(); //1，红，右下
					LED.to_off=1; //仅点亮2ms，取代delay_ms(1)
					//delay_ms(1); //轮流点亮1ms
					break;
				case 3:
				case 21:
					fLED_L(); //2，蓝，右上
					LED.to_off=1; //仅点亮2ms，取代delay_ms(1)
					//delay_ms(1); //轮流点亮1ms
					break;
				case 6:
				case 18:
					hLED_L(); //3，蓝，左上
					LED.to_off=1; //仅点亮2ms，取代delay_ms(1)
					//delay_ms(1); //轮流点亮1ms
					break;
				case 9:
				case 15:
					aLED_L(); //4，红，左下
					LED.to_off=1; //仅点亮2ms，取代delay_ms(1)
					//delay_ms(1); //轮流点亮1ms
					break;
			}
			
			
			
	}
	
	
	
	if(LED.off){
		//以下转移到CloseLED()
		//bLED_H();
		//fLED_H();
		//aLED_H();
		//hLED_H();
		
		return; //提前结束
	}
	
	
	
	switch(LED.status)
	{
		case AlwaysOff:      //常暗   
			bLED_H();
			fLED_H();
			aLED_H();
			hLED_H();
			break;
		
		//进入解锁状态，并且姿态模式
		case AlwaysOn:  //常亮
		  bLED_L();
			fLED_L();
			aLED_L();
			hLED_L();
		  break;
		
		//这种有可能时同步闪烁，也有可能时交替闪烁
		//看闪烁前，各灯的状态。
		case AllFlashLight:				  //全部同时闪烁
			if(falg_Time++%3==0){
					fLED_Toggle();			
					bLED_Toggle();
					hLED_Toggle();			
					aLED_Toggle();
			}				
		  break;

		//2前灯灭，2后灯亮，
		//rf刚初始化，或者失联遥控3秒后
		case AlternateFlash: //交替闪烁
			//自己先调用下面一行
			//bLED_H();aLED_H();fLED_L();hLED_L(); //交替状态，2前灯灭，2后灯亮，
			if(falg_Time++%3==0){
					fLED_Toggle();
					bLED_Toggle();
					hLED_Toggle();
					aLED_Toggle();
			
			}

			//LED.status = AllFlashLight; //切换为闪烁
			break;
		
		//同步闪烁，表示已连接遥控，但尚未解锁
		case SimpleFlash: //同步闪烁
			//自己先调用下面一行
			//bLED_H();aLED_H();fLED_H();hLED_H(); //同步状态
		 
			fLED_Toggle();			
			bLED_Toggle();
			hLED_Toggle();			
			aLED_Toggle();
			//LED.status = AllFlashLight; //再切换为闪烁
			break;	
		
		//0.3秒默认快闪，改为0.8秒慢闪
		case WARNING:
		  fLED_Toggle();
		  hLED_Toggle();
			bLED_Toggle();
		  aLED_Toggle();
			LED.FlashTime = 800;
			break;
		
		//进入解锁状态，并且悬停模式
		case DANGEROURS:   //常亮 2秒 闪烁 1秒
			falg_Time++;
		  if(falg_Time>11)
					falg_Time=0;
				
			if(falg_Time>8)
			{
					bLED_Toggle(); //闪烁
					aLED_Toggle();
					fLED_Toggle();
					hLED_Toggle();
			}
			else  if((falg_Time<7))
			{
					bLED_L();  //常亮
					fLED_L();
					aLED_L();
					hLED_L();
			}
		 
			LED.FlashTime = 150;
			
			break;
			
		//默认关闭
		default:
			LED.status = AlwaysOff;
			break;
	}
}


/*
开机，初始化rf后，交替闪烁
连接上遥控，同步闪烁
姿态解锁：常亮，
悬停解锁：常亮2s，闪烁1s
失联：交替闪烁。
重新连上遥控，同步闪烁。


*/

/**************************END OF FILE*********************************/



