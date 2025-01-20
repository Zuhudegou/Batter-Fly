


#include "ALL_DEFINE.h"
#include "scheduler.h"
#include "STM32F10x_IWDG.h"



void IWDG_Init(u8 prer,u16 rlr) 
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //使能对寄存器IWDG_PR和IWDG_RLR的写超作
	
	IWDG_SetPrescaler(prer);  //设置IWDG预分频值：设置IWDG预分频值64
	
	IWDG_SetReload(rlr);  //设置IWDG重装载值
	
	IWDG_ReloadCounter();  //按照IWDG重装载寄存器的值重装载IWDG计数器
	
	IWDG_Enable();  //使能IWDG
}
 
//喂独立看门狗
void IWDG_Feed(void)
{   
 	IWDG_ReloadCounter();//喂狗								   
}


int main(void)
{	
	//MPU_Err =1;  //  要等陀螺仪初始化成功后，再做陀螺仪数据有效性判断(scheduler.c调用)

	cycleCounterInit();  //得到系统每个us的系统CLK个数，为以后延时函数，和得到精准的当前执行时间使用
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //4个bit的抢占优先级，4个bit的子优先级
	SysTick_Config(SystemCoreClock / 1000);	//系统滴答时钟


	
	ALL_Init();//系统初始化 
  IWDG_Init(4,625);    //初始化看门狗  与分频为64,重载值为625,溢出时间1s
	
	
	while(1)
	{
		  main_loop();  //程序运行,  Loop_check() 在delay.c调用，2ms一次
		
		  IWDG_Feed();  //喂看门狗防止程序跑死
	}
}










