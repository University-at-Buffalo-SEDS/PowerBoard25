#include <Drivers/cltc2990.h>
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
int CLTC2990_Init(CLTC2990_Handle_t *handle, I2C_HandleTypeDef *hi2c, uint8_t address) {
	int8_t ack;

	handle->hi2c = hi2c;

	//Initialize current to NAN
	handle->current = NAN;

	handle->i2c_address = address;

	ack = CLTC2990_Set_Mode(handle, 0x01, VOLTAGE_MODE_MASK); // was V1DV2_V3DV4

	if(ack != 0) {
		CDC_Transmit_Print("Failed to set in Differential Voltage Mode \n");
		while(1);
	}


	// Enable all voltage channels
	ack = CLTC2990_Enable_V1DV2_V3DV4(handle);
	if(ack != 0) {
		HAL_Delay(50);
		CDC_Transmit_Print("Failed to enable all voltage channels. \n");
		while(1);
	}

	HAL_Delay(100);
	CDC_Transmit_Print("LTC2990 configured for Differential Voltage Monitoring. \n");

	//Initial data reading
	CLTC2990_Step(handle);

	return 0;
}

/**
  * @brief  Tell the LTC2990 chip to refresh voltage readings,
  * 		This does not return the voltage(s) read, use LTC2990_Get_Voltage to do so
  * @param  Pointer to the LTC2990 handle
  */
void CLTC2990_Step(CLTC2990_Handle_t *handle) {
	int8_t ack;
	uint16_t adc_code;
	int8_t data_valid;
	//Trigger Conversion
	ack = CLTC2990_Trigger_Conversion(handle);
	if(ack != 0) {
		CDC_Transmit_Print("Failed to trigger conversion.");
		return;
	}

	// Allow time for conversion
	HAL_Delay(10);

	// Read differential voltage V1-V2

	double current[1];

	uint8_t msb_registers[2] = {V1DV2_MSB_REG, V3DV4_MSB_REG};
	ack = CLTC2990_ADC_Read_New_Data(handle, msb_registers[0], &adc_code, &data_valid);
	if(ack != 0 || data_valid != 1) {
		CDC_Transmit_Print("Error reading Register %x \n", msb_registers[0]);
		CDC_Transmit_Print("This is the ack: %d \n", ack);
		CDC_Transmit_Print("This is the data valid: %d \n", data_valid);
		current[0] = NAN;
	} else {
		CDC_Transmit_Print("Code is %X \r\n", adc_code);
		HAL_Delay(50);
		current[0] = CLTC2990_Code_To_Current(handle, adc_code);
	}

	//To enable redaings of V3-V4, uncomment lines below

//	ack = CLTC2990_ADC_Read_New_Data(handle, msb_registers[1], &adc_code, &data_valid);
//	if(ack != 0 || data_valid != 1) {
//		CDC_Transmit_Print("Error reading Register %x \n", msb_registers[1]);
//		CDC_Transmit_Print("This is the ack: %d \n", ack);
//		CDC_Transmit_Print("This is the data valid: %d \n", data_valid);
//		voltage[1] = NAN;
//	} else {
//		voltage[1] = CLTC2990_Code_To_Differential_Voltage(handle, adc_code);
//	}

	handle->current = current[0];


}

/**
  * @brief  Puts the latest voltage readings in the array passed
  * @param  Pointer to the LTC2990 handle
  * @param 	Pointer to the array to store voltage values to
  */
double CLTC2990_Get_Current(CLTC2990_Handle_t* handle) {
	return handle->current;
}


inline int8_t CLTC2990_Enable_V1DV2_V3DV4(CLTC2990_Handle_t *handle) {
	return CLTC2990_Set_Mode(handle, ENABLE_ALL, TEMP_MEAS_MODE_MASK);
}


int8_t CLTC2990_Set_Mode(CLTC2990_Handle_t *handle, uint8_t bits_to_set, uint8_t bits_to_clear) {
	uint8_t reg_data;
	int8_t ack;

	// Read current CONTROL_REG
	ack = CLTC2990_Read_Register(handle, CONTROL_REG, &reg_data);
	if (ack != 0) {
		CDC_Transmit_Print("Failed to Read_Register in Set_Mode\n");
		return ack;
	}

	//Modify bits
	reg_data &= ~bits_to_clear;
	reg_data |= bits_to_set;

	//Write back to CONTROL_REG
	ack = CLTC2990_Write_Register(handle, CONTROL_REG, reg_data);
	if (ack != 0) {
		CDC_Transmit_Print("Failed to Write_Register in Set_Mode\n");
	}
	return ack;
}

int8_t CLTC2990_Trigger_Conversion(CLTC2990_Handle_t *handle) {
	return CLTC2990_Write_Register(handle, TRIGGER_REG, 0x00);
}


uint8_t CLTC2990_ADC_Read_New_Data(CLTC2990_Handle_t *handle, uint8_t msb_register_address, uint16_t* adc_code, int8_t* data_valid) {
	uint16_t timeout = TIMEOUT;
	int8_t ack;
	uint8_t status;
	uint8_t status_bit = (msb_register_address / 2) - 1;

	// Wait for new data
	while (--timeout) {
		ack = CLTC2990_Read_Register(handle, STATUS_REG, &status);

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
	ack = CLTC2990_Read_Register(handle, msb_register_address, &msb);
	if(ack != 0) {
		return ack;
	}

	ack = CLTC2990_Read_Register(handle, msb_register_address + 1, &lsb);
	if(ack != 0) {
		return ack;
	}


	uint16_t code = ((uint16_t)msb << 8) | lsb;
	*data_valid = (code >> 15) & 0x01;  // Data valid bit
	*adc_code = code & 0x7FFF;
	//I dont think this line is actually required, not very coolio
	//i replaceed it with a [working?] one
	//*adc_code = code;


	return (*data_valid == 1) ? 0 : 1;


	//this line is so that there isn't a warning that this function doesn't return
	//In the actual code, it should NEVER reach this point, as this would be the timeout
	//ran out but got messed up
	CDC_Transmit_Print("the thing I said wouldn't happen \n");
	return 2;

}

double CLTC2990_Code_To_Current(CLTC2990_Handle_t *handle, uint16_t adc_code) {
	double voltage;
	int16_t sign = 1;


	if(adc_code & 0x4000) { //If the code is negative //was 0x4000
		CDC_Transmit_Print("Negative??? \r\n");
		adc_code = (adc_code ^ 0x3FFF) + 1;// Two's compliment //was 0x7FFF
		sign = -1;
	}

	adc_code &= 0x3FFF;
	voltage = ((double)adc_code) * CSINGLE_ENDED_LSB;
	voltage /= RSENSE;
	voltage *= sign;

	return voltage;
}

int8_t CLTC2990_Read_Register(CLTC2990_Handle_t *handle, uint8_t reg_address, uint8_t* data) {

	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(handle->hi2c, handle->i2c_address << 1, reg_address, 1, data, 1, TIMEOUT);
	if(status == HAL_OK) {
		return 0;
	}
	CDC_Transmit_Print("I2C Read Register failed, status: %d\n", status);
	return 1;
}

//int8_t CLTC2990_Read_Register(CLTC2990_Handle_t *handle, uint8_t reg_address, uint8_t* data) {
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


int8_t CLTC2990_Write_Register(CLTC2990_Handle_t *handle, uint8_t reg_address, uint8_t data) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(handle->hi2c, handle->i2c_address << 1, (uint16_t)reg_address, I2C_MEMADD_SIZE_8BIT, &data, 1, TIMEOUT);
	if(status == HAL_OK) {
		return 0;
	}
	CDC_Transmit_Print("I2C Write Register failed, status: %d\n", status);

	return 1;
}

//int8_t CLTC2990_Write_Register(CLTC2990_Handle_t *handle, uint8_t reg_address, uint8_t data) {
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
