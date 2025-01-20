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



//硬件spi
//spi2给射频模块使用。PB12，PB13，PB14，PB15; 其中CE和CSN管脚初始化在nrf24l01.c
//spi1在程序里没有初始化和管脚定义，PA4，PA5，PA6，PA7；其中A4,A5被改为B6,B7

#include "spi.h"
#include "sys.h"

void SPI_Config(void)
{
      SPI_InitTypeDef  SPI_InitStructure;
   	  GPIO_InitTypeDef GPIO_InitStructure;

			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);  
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO  , ENABLE);
			GPIO_SetBits(NRF_CSN_GP, NRF24L01_CSN); //NRF_CS预置为高 
		
	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	  	GPIO_Init(GPIOB, &GPIO_InitStructure); 

			/* SPI2 configuration */
			SPI_Cmd(SPI2, DISABLE);             //必须先禁能,才能改变MODE
			SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
			SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
			SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
			SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
			SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
			SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
			SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
			SPI_InitStructure.SPI_CRCPolynomial = 7;

			SPI_Init(SPI2, &SPI_InitStructure);

			/* SPI2 enable */
			SPI_Cmd(SPI2, ENABLE);
}


//数据交换，发送数据，同时也会接收数据
u8 SPI_RW(u8 dat)
{
        /* Loop while DR register in not emplty */
        while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);

        /* Send byte through the SPI2 peripheral */
        SPI_I2S_SendData(SPI2, dat);

        /* Wait to receive a byte */
        while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);

        /* Return the byte read from the SPI bus */
        return SPI_I2S_ReceiveData(SPI2);  
}



