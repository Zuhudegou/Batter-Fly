
#ifndef _SPL06_CANKAO_H
#define _SPL06_CANKAO_H

#include "ALL_DEFINE.h"


//#include "Arduino.h"
	
typedef struct {	
    uint8 chip_id; //芯片id	
		float height;//高度
		float pressure;//气压
		float temperature;//温度
		float traw_sc;//温度原始数据(修正后)
		float praw_sc;//压力原始数据(修正后)
		float Kt;
		float Kp;

}spl0601_new_t ;

extern spl0601_new_t spl0601_new;
	
void Height_Get_New(float dT);
	
	
//void SPL_init(uint8_t spl_address=0x76);
uint8  SPL_init(void);

uint8_t get_spl_id(void);		// Get ID Register 		0x0D
uint8_t get_spl_prs_cfg(void);	// Get PRS_CFG Register	0x06
uint8_t get_spl_tmp_cfg(void);	// Get TMP_CFG Register	0x07
uint8_t get_spl_meas_cfg(void);	// Get MEAS_CFG Register	0x08
uint8_t get_spl_cfg_reg(void);	// Get CFG_REG Register	0x09
uint8_t get_spl_int_sts(void);	// Get INT_STS Register	0x0A
uint8_t get_spl_fifo_sts(void);	// Get FIFO_STS Register	0x0B

double get_altitude(double pressure, double seaLevelhPa);	// get altitude in meters
double get_altitude_f(double pressure, double seaLevelhPa);	// get altitude in feet

int32_t get_traw(void);
double get_traw_sc(void);
double get_temp_c(double traw_sc);
double get_temp_f(double traw_sc);
double get_temperature_scale_factor(void);

int32_t get_praw(void);
double get_praw_sc(void);
double get_pcomp(void);
double get_pressure_scale_factor(void);
double get_pressure(void);

int16_t get_c0(void);
int16_t get_c1(void);
int32_t get_c00(void);
int32_t get_c10(void);
int16_t get_c01(void);
int16_t get_c11(void);
int16_t get_c20(void);
int16_t get_c21(void);
int16_t get_c30(void);

void i2c_eeprom_write_uint8_t(  uint8_t deviceaddress, uint8_t eeaddress, uint8_t data );
uint8_t i2c_eeprom_read_uint8_t(  uint8_t deviceaddress, uint8_t eeaddress );


#endif

