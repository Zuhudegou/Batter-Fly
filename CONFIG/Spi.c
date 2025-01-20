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



//Ӳ��spi
//spi2����Ƶģ��ʹ�á�PB12��PB13��PB14��PB15; ����CE��CSN�ܽų�ʼ����nrf24l01.c
//spi1�ڳ�����û�г�ʼ���͹ܽŶ��壬PA4��PA5��PA6��PA7������A4,A5����ΪB6,B7

#include "spi.h"
#include "sys.h"

void SPI_Config(void)
{
      SPI_InitTypeDef  SPI_InitStructure;
   	  GPIO_InitTypeDef GPIO_InitStructure;

			RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);  
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO  , ENABLE);
			GPIO_SetBits(NRF_CSN_GP, NRF24L01_CSN); //NRF_CSԤ��Ϊ�� 
		
	  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; 
	  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	  	GPIO_Init(GPIOB, &GPIO_InitStructure); 

			/* SPI2 configuration */
			SPI_Cmd(SPI2, DISABLE);             //�����Ƚ���,���ܸı�MODE
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


//���ݽ������������ݣ�ͬʱҲ���������
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



