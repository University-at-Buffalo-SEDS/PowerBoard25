#ifndef CLTC2990_H
#define CLTC2990_H

#include "stm32g4xx_hal.h"
#include <math.h>

#include "main.h"

// I2C Register Addresses
#define CSTATUS_REG			(0x00)
#define CCONTROL_REG		(0x01)
#define CTRIGGER_REG		(0x02)
#define CV1_MSB_REG			(0x06)
#define CV1_LSB_REG			(0x07)
#define CV2_MSB_REG			(0x08)
#define CV2_LSB_REG			(0x09)

// Control Register Settings
#define CVOLTAGE_MODE_MASK	(0x07)
#define CV1_V2_T2			(0x07)
#define CTEMP_MEAS_MODE_MASK (0x18)

#define CV1DV2_V3DV4		(0x06)
#define CENABLE_ALL			(0x18)

// Conversion Constants
#define CSINGLE_ENDED_LSB  (5.0f / 16384.0f) // 5V / 2^14
#define CRSENSE 0.005 // was 0.005

// Timeout for data validity in milliseconds
#define CTIMEOUT 1000

#define CLTC2990_I2C_ADDRESS (0x9A >> 1)

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t i2c_address;

	// Internal buffer for voltage readings
	float current;
} CLTC2990_Handle_t;

int CLTC2990_Init(CLTC2990_Handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t addr);
void CLTC2990_Step(CLTC2990_Handle_t *handle);
float CLTC2990_Get_Current(CLTC2990_Handle_t *handle);

int8_t CLTC2990_Enable_V1DV2_V3DV4(CLTC2990_Handle_t *handle);
int8_t CLTC2990_Set_Mode(CLTC2990_Handle_t *handle, uint8_t bits_to_set, uint8_t bits_to_clear);
int8_t CLTC2990_Trigger_Conversion(CLTC2990_Handle_t *handle);
uint8_t CLTC2990_ADC_Read_New_Data(CLTC2990_Handle_t *handle, uint8_t msb_register_address, int16_t* adc_code, int8_t* data_valid);

float CLTC2990_Code_To_Single_Ended_Voltage(CLTC2990_Handle_t *handle, uint16_t adc_code);

//L2C Communication Helpers
int8_t CLTC2990_Read_Register(CLTC2990_Handle_t *handle, uint8_t reg_address, uint8_t* data);
int8_t CLTC2990_Write_Register(CLTC2990_Handle_t *handle, uint8_t reg_address, uint8_t data);

//For usb serial print
extern void CDC_Transmit_Print(const char * format, ...);

#endif
