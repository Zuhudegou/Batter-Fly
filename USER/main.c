


#include "ALL_DEFINE.h"
#include "scheduler.h"
#include "STM32F10x_IWDG.h"



void IWDG_Init(u8 prer,u16 rlr) 
{	
 	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  
	
	IWDG_SetPrescaler(prer);  
	
	IWDG_SetReload(rlr);  
	
	IWDG_ReloadCounter();  
	
	IWDG_Enable();  
}
 

void IWDG_Feed(void)
{   
 	IWDG_ReloadCounter();								   
}


int main(void)
{	
	//MPU_Err =1;  

	cycleCounterInit();  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
	SysTick_Config(SystemCoreClock / 1000);	


	
	ALL_Init();
  IWDG_Init(4,625);   
	
	
	while(1)
	{
		  main_loop();  
		
		  IWDG_Feed();  
	}
}










