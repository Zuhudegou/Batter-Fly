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



#include "ALL_DEFINE.h"
#include "scheduler.h"
#include "STM32F10x_IWDG.h"

//特别声明：请在无人空旷的地带或者室内进行飞行。遇到非常紧急的情况，可紧急关闭遥控。

/*初始化独立看门狗
//prer:分频数:0~7(只有低3位有效！)
//分频因子=4*2^prer.最大值只能是256!
//rlr:重装载寄存器值:低11位有效.
//时间计算(大概):Tout=((4*2^prer)*rlr)/40 (ms).   */

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










