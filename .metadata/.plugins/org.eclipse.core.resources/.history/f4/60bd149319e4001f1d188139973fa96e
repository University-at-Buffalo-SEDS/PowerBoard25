#ifndef LTC2990_H
#define LTC2990_H

#include "stm32g4xx_hal.h"
#include <math.h>

// I2C Register Addresses
#define STATUS_REG        (0x00)
#define CONTROL_REG       (0x01)
#define TRIGGER_REG       (0x02)
#define V1_MSB_REG        (0x06)
#define V1_LSB_REG        (0x07)
#define V2_MSB_REG        (0x08)
#define V2_LSB_REG        (0x09)
#define V3_MSB_REG        (0x0A)
#define V3_LSB_REG        (0x0B)
#define V4_MSB_REG        (0x0C)
#define V4_LSB_REG        (0x0D)

// Control Register Settings
#define VOLTAGE_MODE_MASK     (0x07)
#define V1_V2_V3_V4           (0x07)
#define ENABLE_ALL            (0x18)
#define TEMP_MEAS_MODE_MASK   (0x18)

// Conversion Constants
#define SINGLE_ENDED_LSB (5.0f / 16384.0f) // 5V / 2^14

// Timeout for data validity in milliseconds
#define TIMEOUT 1000

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t i2c_address;

	// Internal buffer for voltage readings
	float last_voltages[4];
} LTC2990_Handle_t;

int LTC2990_Init(LTC2990_Handle_t *handle);
void LTC2990_Step(LTC2990_Handle_t *handle);
void LTC2990_Get_Voltage(LTC2990_Handle_t *handle, float* voltages);

int8_t LTC2990_Set_Mode(LTC2990_Handle_t *handle, uint8_t bits_to_set, uint8_t bits_to_clear);
int8_t LTC2990_Enable_All_Voltages(LTC2990_Handle_t *handle);
int8_t LTC2990_Trigger_Conversion(LTC2990_Handle_t *handle);
uint8_t LTC2990_ADC_Read_New_Data(LTC2990_Handle_t *handle, uint8_t msb_register_address, int16_t* adc_code, int8_t* data_valid);

float LTC2990_Code_To_Single_Ended_Voltage(LTC2990_Handle_t *handle, int16_t adc_code);

//L2C Communication Helpers
int8_t LTC2990_Read_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t* data);
int8_t LTC2990_Write_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t* data);

#endif
