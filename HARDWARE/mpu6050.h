#ifndef __MPU6050_H
#define __MPU6050_H


#include "sys.h"
#include "I2C.h"


extern int16_t MpuOffset[7]; //增加一个元素，是否已校准

extern int8_t MpuInit(void);
extern void MpuGetData(void);
extern void MpuGetOffset(void);
extern void MpuGetOffset_save(void); //用户主动校准 加保存


#endif // __MPU6050_H__








