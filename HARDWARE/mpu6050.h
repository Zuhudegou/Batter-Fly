#ifndef __MPU6050_H
#define __MPU6050_H


#include "sys.h"
#include "I2C.h"


extern int16_t MpuOffset[7]; //����һ��Ԫ�أ��Ƿ���У׼

extern int8_t MpuInit(void);
extern void MpuGetData(void);
extern void MpuGetOffset(void);
extern void MpuGetOffset_save(void); //�û�����У׼ �ӱ���


#endif // __MPU6050_H__








