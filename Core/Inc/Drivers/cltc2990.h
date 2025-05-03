#ifndef CLTC2990_H
#define CLTC2990_H

#include "stm32g4xx_hal.h"
#include <math.h>

#include "main.h"

// I2C Register Addresses
#define STATUS_REG			(0x00)
#define CONTROL_REG			(0x01)
#define TRIGGER_REG			(0x02)
#define V1DV2_MSB_REG		(0x06) // was 0x06
#define V1DV2_LSB_REG		(0x07)
#define V3DV4_MSB_REG		(0x08)
#define V3DV4_LSB_REG		(0x09)

// Control Register Settings
#define VOLTAGE_MODE_MASK	(0x07)
#define TEMP_MEAS_MODE_MASK (0x18)

#define V1DV2_V3DV4			(0x06)
#define ENABLE_ALL			(0x08)

// Conversion Constants
#define CSINGLE_ENDED_LSB 	(0.318 / 16384.0) //should i try 0.00001942f // ex value (0.00000947581f)
#define RSENSE 0.005 // was 0.005

// Timeout for data validity in milliseconds
#define TIMEOUT 1000

#define CLTC2990_I2C_ADDRESS (0x9A >> 1)

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t i2c_address;

	// Internal buffer for voltage readings
	double current;
} CLTC2990_Handle_t;

int CLTC2990_Init(CLTC2990_Handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t address);
void CLTC2990_Step(CLTC2990_Handle_t *handle);
double CLTC2990_Get_Current(CLTC2990_Handle_t *handle);

int8_t CLTC2990_Enable_V1DV2_V3DV4(CLTC2990_Handle_t *handle);
int8_t CLTC2990_Set_Mode(CLTC2990_Handle_t *handle, uint8_t bits_to_set, uint8_t bits_to_clear);
int8_t CLTC2990_Trigger_Conversion(CLTC2990_Handle_t *handle);
uint8_t CLTC2990_ADC_Read_New_Data(CLTC2990_Handle_t *handle, uint8_t msb_register_address, uint16_t* adc_code, int8_t* data_valid);

double CLTC2990_Code_To_Current(CLTC2990_Handle_t *handle, uint16_t adc_code);

//L2C Communication Helpers
int8_t CLTC2990_Read_Register(CLTC2990_Handle_t *handle, uint8_t reg_address, uint8_t* data);
int8_t CLTC2990_Write_Register(CLTC2990_Handle_t *handle, uint8_t reg_address, uint8_t data);

//For usb serial print
extern void CDC_Transmit_Print(const char * format, ...);

#endif
