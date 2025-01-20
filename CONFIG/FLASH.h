#ifndef _FLASH_H_
#define _FLASH_H_
#include "stm32f10x.h"


//103C8共有64K ROM，  08 00 00 00 H ~  08 00 FF FF H ，我们的程序有30多K大概在接近0x08007C00H结束
//我们自己选中了1K的区域，但实际上我们保存的数据，只是要我们仅仅需要写6个双字节，12个字节
#define FLASH_Page_Size    (0x00000400) //FLASH页的大小1K
//#define FLASH_Start_Addr   (0x08007C00) //要编写的起始地址   位于31K字节处
//#define FLASH_End_Addr     (0x08008000) //要编写的结束地址   位于32K字节处
#define DATA_32            (0x12345678) //要编写的数据


//在falsh的0x08007C00地址写入有问题，在上位机做完MPU6050的校准后，usb失效了。
#define FLASH_Start_Addr   (0x0800F400) //要编写的起始地址   位于61K字节处, 芯片flash一共64K字节
#define FLASH_End_Addr     (0x0800F800) //要编写的结束地址   位于62K字节处

//芯片flash开始地址 0x08000000
//芯片flash结束地址 0x0800FFFF  //大小64K, 即0x10000(65536)/0x400(1024)
//芯片flash结束地址 0x0801FFFF  //大小128K,即0x20000(131072)/0x400(1024)
//stm32f103c8t6芯片规格书为64k，但有的芯片实际有128k falsh


extern void FLASH_read(int16_t *data,uint8_t len);
extern void FLASH_write(int16_t *data,uint8_t len);	

#endif
