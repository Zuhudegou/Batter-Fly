#ifndef __CONTROL_H
#define __CONTROL_H

#include "pid.h"
#include "sys.h"



//��ȡ AUX72 AUX71,,, AUX73 
#define AUX72()				((uint8_t)( (Remote.AUX7 & 0xf000)>>12 ))		//���ű����� ���ֽڸ�4λ
#define AUX71() 			((uint8_t)( (Remote.AUX7 & 0x0f00)>>8  ))		 //���ȣ�		���ֽڵ�4λ
#define AUX73()  			((int8_t) (  Remote.AUX7 & 0x00ff      ))		//Ŀ��ƫ���ǣ�		���ֽ�

//���� AUX7 = 0x mn ww,  m����(AUX72)��nΪ����(AUX71)��wwƫ����(AUX73)
#define AUX72_s(x)  (Remote.AUX7 = ((Remote.AUX7 & 0x0fff) | ( ((uint16_t)x<<12) & 0xf000) ))		//д����ֽڸ�4λ�����ű���
#define AUX71_s(x)  (Remote.AUX7 = ((Remote.AUX7 & 0xf0ff) | ( ((uint16_t)x<<8)  & 0x0f00) )) 	//д����ֽڵ�4λ������
#define AUX73_s(x)  (Remote.AUX7 = ((Remote.AUX7 & 0xff00) | (  (uint16_t)x      & 0x00ff) ))  	//д����ֽڣ�Ŀ��ƫ����
 
//���� AUX81~AUX88��ֻ���˵��ֽڵ�8λ��������� a���������� b�� ��ͷģʽ c�� ���� e�� LED���� f�� �߶ȼ�ָ�� g�� �߶ȼ�ָ�� h��
// ABC0 0000 abcd efgh 
#define AUX811_isH()  (Remote.AUX8 & 0x8000)	//������У׼ ���ֽڵĵ�һλA
#define AUX822_isH()  (Remote.AUX8 & 0x4000)	//����Զ����ԣ����ֽڵĵ�2λB
#define AUX833_isH()  (Remote.AUX8 & 0x2000)	//��ѹ���ߣ����ֽڵĵ�3λC��Ĭ�ϼ��ⶨ�ߣ���ѹֻ��4�����ϲ���Ч��
#define AUX844_isH()   (Remote.AUX8 & 0x1000)	//����ֶ����ԣ����ֽڵĵ�1λD
//Ҫ�� !=0 �� ==0 �жϣ������� ==1

//����Ϊaux8���ֽ�
#define AUX81_isH()   (Remote.AUX8 & 0x0080)	//������� a
#define AUX82_isH()   (Remote.AUX8 & 0x0040)	//�������� b
#define AUX83_isH()   (Remote.AUX8 & 0x0020)	//��ͷģʽ c
#define AUX84_isH()   (Remote.AUX8 & 0x0010)	//�߶ȵ�����¼ d�����ڶ���ģʽ�������������

#define AUX85_isH()   (Remote.AUX8 & 0x0008)	//���� e
#define AUX86_isH()   (Remote.AUX8 & 0x0004)	//LED���� f��1���ر�LED
#define AUX87_isH()   (Remote.AUX8 & 0x0002)	//�߶ȼ�ָ�� g
#define AUX88_isH()   (Remote.AUX8 & 0x0001)	//�߶ȼ�ָ�� h

#define AUX811_s(x)  (x>0?( Remote.AUX8 = (Remote.AUX8 | 0x8000) ):( Remote.AUX8 = (Remote.AUX8 & ~0x8000) ))//������У׼ ���ֽڵĵ�һλA



//����Ϊaux8���ֽ�
#define AUX81_s(x)  (x>0?( Remote.AUX8 = (Remote.AUX8 | 0x0080) ):( Remote.AUX8 = (Remote.AUX8 & ~0x0080) ))//������� a
#define AUX82_s(x)  (x>0?( Remote.AUX8 = (Remote.AUX8 | 0x0040) ):( Remote.AUX8 = (Remote.AUX8 & ~0x0040) ))//�������� b
#define AUX83_s(x)  (x>0?( Remote.AUX8 = (Remote.AUX8 | 0x0020) ):( Remote.AUX8 = (Remote.AUX8 & ~0x0020) ))//��ͷģʽ c
#define AUX84_s(x)  (x>0?( Remote.AUX8 = (Remote.AUX8 | 0x0010) ):( Remote.AUX8 = (Remote.AUX8 & ~0x0010) ))//�߶ȵ�����¼ d

#define AUX85_s(x)  (x>0?( Remote.AUX8 = (Remote.AUX8 | 0x0008) ):( Remote.AUX8 = (Remote.AUX8 & ~0x0008) ))//���� e
#define AUX86_s(x)  (x>0?( Remote.AUX8 = (Remote.AUX8 | 0x0004) ):( Remote.AUX8 = (Remote.AUX8 & ~0x0004) ))//LED���� f
#define AUX87_s(x)  (x>0?( Remote.AUX8 = (Remote.AUX8 | 0x0002) ):( Remote.AUX8 = (Remote.AUX8 & ~0x0002) ))//�߶ȼ�ָ�� g
#define AUX88_s(x)  (x>0?( Remote.AUX8 = (Remote.AUX8 | 0x0001) ):( Remote.AUX8 = (Remote.AUX8 & ~0x0001) ))//�߶ȼ�ָ�� h





#define INIT_0 0

#define WAITING_1 1
#define WAITING_2 2
#define WAITING_3 3
#define WAITING_4 4
#define WAITING_5 5
#define WAITING_6 6
#define WAITING_7 7
#define WAITING_8 8
#define WAITING_9 9
#define WAITING_10 10

#define READY_11 11
#define READY_12 12
#define READY_13 13
#define READY_14 14
#define READY_15 15
#define READY_16 16
#define READY_17 17
#define READY_18 18
#define READY_19 19
#define READY_20 20
#define READY_21 21
#define READY_22 22
#define READY_23 23
#define READY_24 24
#define READY_25 25
#define READY_26 26
#define READY_27 27
#define READY_28 28
#define READY_29 29
#define READY_30 30

#define PROCESS_31 31
#define PROCESS_32 32
#define PROCESS_33 33
#define PROCESS_34 34
#define PROCESS_35 35
#define PROCESS_36 36
#define PROCESS_37 37
#define PROCESS_38 38
#define PROCESS_39 39
#define PROCESS_40 40
#define PROCESS_41 41
#define PROCESS_42 42
#define PROCESS_43 43
#define PROCESS_44 44
#define PROCESS_45 45
#define PROCESS_46 46
#define PROCESS_47 47
#define PROCESS_48 48
#define PROCESS_49 49
#define PROCESS_50 50
#define PROCESS_51 51
#define PROCESS_52 52
#define PROCESS_53 53
#define PROCESS_54 54
#define PROCESS_55 55
#define PROCESS_56 56
#define PROCESS_57 57
#define PROCESS_58 58
#define PROCESS_59 59
#define PROCESS_60 60


#define CHECK_100 100 //У��
#define CHECK_101 101
#define CHECK_102 102
#define CHECK_103 103
#define CHECK_104 105
#define CHECK_105 106

#define CALIBRATION_110 110 //У׼
#define CALIBRATION_111 111
#define CALIBRATION_112 112
#define CALIBRATION_113 113
#define CALIBRATION_114 114
#define CALIBRATION_115 115 

#define EXIT_255 255  //�˳�
#define EXIT_254 254
#define EXIT_253 253
#define EXIT_252 252
#define EXIT_251 251
#define EXIT_250 250

extern void FlightPidControl(float FightControlTime);
extern void MotorControl(void);
extern void HeightPidControl(float dt);			//�߶ȿ�����     ��ѹ
extern void Flow_Pos_Controler(float dt);  //λ�ö��������  ����
extern void Flow_mode_two(void);
extern void Mode_Controler(float dt);



#endif


