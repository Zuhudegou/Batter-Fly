#ifndef _FLASH_H_
#define _FLASH_H_
#include "stm32f10x.h"


//103C8����64K ROM��  08 00 00 00 H ~  08 00 FF FF H �����ǵĳ�����30��K����ڽӽ�0x08007C00H����
//�����Լ�ѡ����1K�����򣬵�ʵ�������Ǳ�������ݣ�ֻ��Ҫ���ǽ�����Ҫд6��˫�ֽڣ�12���ֽ�
#define FLASH_Page_Size    (0x00000400) //FLASHҳ�Ĵ�С1K
//#define FLASH_Start_Addr   (0x08007C00) //Ҫ��д����ʼ��ַ   λ��31K�ֽڴ�
//#define FLASH_End_Addr     (0x08008000) //Ҫ��д�Ľ�����ַ   λ��32K�ֽڴ�
#define DATA_32            (0x12345678) //Ҫ��д������


//��falsh��0x08007C00��ַд�������⣬����λ������MPU6050��У׼��usbʧЧ�ˡ�
#define FLASH_Start_Addr   (0x0800F400) //Ҫ��д����ʼ��ַ   λ��61K�ֽڴ�, оƬflashһ��64K�ֽ�
#define FLASH_End_Addr     (0x0800F800) //Ҫ��д�Ľ�����ַ   λ��62K�ֽڴ�

//оƬflash��ʼ��ַ 0x08000000
//оƬflash������ַ 0x0800FFFF  //��С64K, ��0x10000(65536)/0x400(1024)
//оƬflash������ַ 0x0801FFFF  //��С128K,��0x20000(131072)/0x400(1024)
//stm32f103c8t6оƬ�����Ϊ64k�����е�оƬʵ����128k falsh


extern void FLASH_read(int16_t *data,uint8_t len);
extern void FLASH_write(int16_t *data,uint8_t len);	

#endif
