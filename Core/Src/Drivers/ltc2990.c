#include "Drivers/LTC2990.h"

#include "main.h"

//To ask Parth
//should I use blocking or non-blocking functions for HAL_I2C function calls
//Should HAL_delay be changed to vTaskDelay() or something

extern void CDC_Transmit_Print(const char * format, ...);

/**
  * @brief  Initialize the LTC2990 Chip
  * @param  Pointer to the LTC2990 handle
  * @param  Pointer to the HAL I2C HandleTypeDef
  * @retval HAL status
  */
int LTC2990_Init(LTC2990_Handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t address) {
	int8_t ack;

	handle->hi2c = hi2c;

	//Initialize voltages to NAN
	//Can this be changed so that it is in the struct
	//i.e. last_voltages = {NAN, NAN, NAN, NAN}
	for (int i = 0; i < 4; i++) {
		handle->last_voltages[i] = NAN;
	}

	handle->i2c_address = address;

	ack = LTC2990_Set_Mode(handle, V1_V2_V3_V4, VOLTAGE_MODE_MASK);

	if(ack != 0) {
		CDC_Transmit_Print("Failed to set in Single Voltage Mode \n");
		while(1);
	}


	// Enable all voltage channels
	ack = LTC2990_Enable_All_Voltages(handle);
	if(ack != 0) {
		HAL_Delay(50);
		CDC_Transmit_Print("Failed to enable voltage channels. \n");
		while(1);
	}

	HAL_Delay(100);
	CDC_Transmit_Print("LTC2990 configured for Single-Ended Voltage Monitoring. \n");

	//Initial data reading
	LTC2990_Step(handle);

	return 0;
}

/**
  * @brief  Tell the LTC2990 chip to refresh voltage readings,
  * 		This does not return the voltage(s) read, use LTC2990_Get_Voltage to do so
  * @param  Pointer to the LTC2990 handle
  */
void LTC2990_Step(LTC2990_Handle_t *handle) {
	int8_t ack;
	int16_t adc_code;
	int8_t data_valid;
	//Trigger Conversion
	ack = LTC2990_Trigger_Conversion(handle);
	if(ack != 0) {
		CDC_Transmit_Print("Failed to trigger conversion.");
		return;
	}

	// Allow time for conversion
	HAL_Delay(10);

	// Read voltages V1 to V4
	uint8_t msb_registers[4] = {V1_MSB_REG, V2_MSB_REG, V3_MSB_REG, V4_MSB_REG};
	for(int i = 0; i < 4; i++) {
		ack = LTC2990_ADC_Read_New_Data(handle, msb_registers[i], &adc_code, &data_valid);
		if(ack != 0 || data_valid != 1) {
			CDC_Transmit_Print("Error reading Voltage %d \n", i);
			CDC_Transmit_Print("This is the ack: %d \n", ack);
			CDC_Transmit_Print("This is the data valid: %d \n", data_valid);
			handle->last_voltages[i] = NAN;
			continue;
		}
		handle->last_voltages[i] = LTC2990_Code_To_Single_Ended_Voltage(handle, adc_code);
		//CDC_Transmit_Print("Just Read Voltages, got: %x \n", handle->last_voltages[i]);
	}

}

/**
  * @brief  Puts the latest voltage readings in the array passed
  * @param  Pointer to the LTC2990 handle
  * @param 	Pointer to the array to store voltage values to
  */
void LTC2990_Get_Voltage(LTC2990_Handle_t* handle, float* voltages) {
	for(int i = 0; i < 4; i++) {
		voltages[i] = handle->last_voltages[i];
	}
}


inline int8_t LTC2990_Enable_All_Voltages(LTC2990_Handle_t *handle) {
	return LTC2990_Set_Mode(handle, ENABLE_ALL, TEMP_MEAS_MODE_MASK);
}


int8_t LTC2990_Set_Mode(LTC2990_Handle_t *handle, uint8_t bits_to_set, uint8_t bits_to_clear) {
	uint8_t reg_data;
	int8_t ack;

	// Read current CONTROL_REG
	ack = LTC2990_Read_Register(handle, CONTROL_REG, &reg_data);
	if (ack != 0) {
		CDC_Transmit_Print("Failed to Read_Register in Set_Mode\n");
		return ack;
	}

	//Modify bits
	reg_data &= ~bits_to_clear;
	reg_data |= bits_to_set;

	//Write back to CONTROL_REG
	ack = LTC2990_Write_Register(handle, CONTROL_REG, reg_data);
	if (ack != 0) {
		CDC_Transmit_Print("Failed to Write_Register in Set_Mode\n");
	}
	return ack;
}

int8_t LTC2990_Trigger_Conversion(LTC2990_Handle_t *handle) {
	return LTC2990_Write_Register(handle, TRIGGER_REG, 0x00);
}


uint8_t LTC2990_ADC_Read_New_Data(LTC2990_Handle_t *handle, uint8_t msb_register_address, int16_t* adc_code, int8_t* data_valid) {
	uint16_t timeout = TIMEOUT;
	int8_t ack;
	uint8_t status;
	uint8_t status_bit = (msb_register_address / 2) - 1;

	// Wait for new data
	while (--timeout) {
		ack = LTC2990_Read_Register(handle, STATUS_REG, &status);

		if (ack != 0) {
			return ack;
		}

		if (((status >> status_bit) & 0x01) == 1) {
			break;
		}

		//
		HAL_Delay(1);
	}


	if (timeout == 0) {
		CDC_Transmit_Print("LTC2990 TIMED OUT \n");
		return 1;
	}

	//Read ADC data
	uint8_t msb;
	uint8_t lsb;
	ack = LTC2990_Read_Register(handle, msb_register_address, &msb);
	if(ack != 0) {
		return ack;
	}

	ack = LTC2990_Read_Register(handle, msb_register_address + 1, &lsb);
	if(ack != 0) {
		return ack;
	}


	uint16_t code = ((uint16_t)msb << 8) | lsb;
	*data_valid = (code >> 15) & 0x01;  // Data valid bit
	*adc_code = code & 0x3FFF;

	return (*data_valid == 1) ? 0 : 1;


	//this line is so that there isn't a warning that this function doesn't return
	//In the actual code, it should NEVER reach this point, as this would be the timeout
	//ran out but got messed up
	CDC_Transmit_Print("the thing I said wouldn't happen \n");
	return 2;

}

float LTC2990_Code_To_Single_Ended_Voltage(LTC2990_Handle_t *handle, uint16_t adc_code) {
	float voltage;
	int16_t sign = 1;

	if(adc_code & 0x4000) { //If the code is negative
		adc_code = (adc_code ^ 0x7FFF) + 1;// Two's compliment
		sign = -1;
	}

	adc_code &= 0x3FFF;
	voltage = ((float)adc_code) * SINGLE_ENDED_LSB * sign;

	return voltage;
}

int8_t LTC2990_Read_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t* data) {

	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(handle->hi2c, handle->i2c_address << 1, reg_address, 1, data, 1, TIMEOUT);
	if(status == HAL_OK) {
		return 0;
	}
	CDC_Transmit_Print("I2C Read Register failed, status: %d\n", status);
	return 1;
}

//int8_t LTC2990_Read_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t* data) {
//	HAL_StatusTypeDef status;
//	status = HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_address << 1, &reg_address, 1, TIMEOUT);
//	if(status != HAL_OK) {
//		return 1;
//	}
//
//	status = HAL_I2C_Master_Receive(handle->hi2c, handle->i2c_address << 1, data, 1, TIMEOUT);
//	if(status != HAL_OK) {
//		return 1;
//	}
//	return 0;
//}


int8_t LTC2990_Write_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t data) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(handle->hi2c, handle->i2c_address << 1, (uint16_t)reg_address, I2C_MEMADD_SIZE_8BIT, &data, 1, TIMEOUT);
	if(status == HAL_OK) {
		return 0;
	}
	CDC_Transmit_Print("I2C Write Register failed, status: %d\n", status);

	return 1;
}

//int8_t LTC2990_Write_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t data) {
//	int8_t ack;
//	ack = HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_address << 1, &reg_address, 1, TIMEOUT);
//	if(ack != 0) {
//		return 1;
//	}
//	HAL_I2C_Master_Transmit(handle->hi2c, handle->i2c_address << 1, &data, 1, TIMEOUT);
//	if(ack != 0) {
//		return 1;
//	}
//	return 0;
//}
