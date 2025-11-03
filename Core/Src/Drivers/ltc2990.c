#include "Drivers/LTC2990.h"

#include "main.h"

//if FREERTOS is running, program uses vTaskDelay, if not it uses HAL_delay
#if defined(USE_FREERTOS)
  #include "FreeRTOS.h"
  #include "task.h"
  static inline void sleep_ms(uint32_t ms) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
      vTaskDelay(pdMS_TO_TICKS(ms));
    } else {
      HAL_Delay(ms);
    }
  }
#else
  static inline void sleep_ms(uint32_t ms) { HAL_Delay(ms); }
#endif


extern void CDC_Transmit_Print(const char * format, ...);

static inline uint8_t status_bit_from_msb(uint8_t msb_reg) {
    switch (msb_reg) {
        case V1_MSB_REG: return 2;
        case V2_MSB_REG: return 3;
        case V3_MSB_REG: return 4;
        case V4_MSB_REG: return 5;
        default:         return 0xFF; // invalid
    }
}

/**
  * @brief  Initialize the LTC2990 Chip
  * @param  Pointer to the LTC2990 handle
  * @param  Pointer to the HAL I2C HandleTypeDef
  * @retval HAL status
  */
int LTC2990_Init(LTC2990_Handle_t *h, I2C_HandleTypeDef *hi2c, uint8_t addr7,LTC2990_ROLE role)
{
    h->hi2c        = hi2c;//store references to the handle
    h->i2c_address = addr7;
    h->role        = role;
    for (int i = 0; i < 4; ++i) h->last_voltages[i] = NAN;//clears all cached readings in the handle


    //prob should make this line below more readable

    //If the role of the chip is VOLTAGE, CTRL_ALL sets [4:3] to "11", which is "All Measurements per Mode"
    //Also V1_V2_V3_V4 sets [2,0] to "111", which you can prob guess activates all voltage measurments (V1 V2 V3 V4)

    //I have in the current setup DISABLED V3 and V4 on the CURRENT chip bc i dont know what it does, the following prob needs to be changed
    //If the role of the chip is CURRENT, CTRL_V1_ONLY sets [4:3] to "01", which is "TR1, V1 or V1 – V2 Only per Mode", we want the V1-V2 
    //bc that is part of the formula to get current according to the datasheet, READ IT
    //Also MODE_V1mV2_TR2 sets [2,0] to "001", which is V1 – V2, TR2 (Ignore TR2 as we arent measuring temps)
    uint8_t control = (role == VOLTAGE)? (CTRL_ALL | V1_V2_V3_V4): (CTRL_V1_ONLY  | MODE_V1mV2_TR2);

    //mask to clear [4:3] and [2:0] before setting the new mode
    uint8_t clear_mask = TEMP_MEAS_MODE_MASK | VOLTAGE_MODE_MASK;

    if (LTC2990_Set_Mode(h, control, clear_mask) != 0) return 1;//sets the new mode

    sleep_ms(100);
    LTC2990_Step(h);//Fills the cache
    return 0;
}

/**
  * @brief  Tell the LTC2990 chip to refresh voltage readings,
  * 		This does not return the voltage(s) read, use LTC2990_Get_Voltage to do so
  * @param  Pointer to the LTC2990 handle
  */
void LTC2990_Step(LTC2990_Handle_t *h)//Use this to fill the cache, auto executes with initialization (LTC2990_Init)
{
    (void)LTC2990_Trigger_Conversion(h);
    sleep_ms(10);

    if (h->role == VOLTAGE) {//Voltage role
        const uint8_t regs[4] = { V1_MSB_REG, V2_MSB_REG, V3_MSB_REG, V4_MSB_REG };
        for (int i = 0; i < 4; ++i) {
            uint16_t raw15; int8_t valid;
            if (LTC2990_ADC_Read_New_Data(h, regs[i], &raw15, &valid) == 0 && valid) {
                uint16_t code14 = (raw15 & 0x3FFF); // SE uses 14-bit magnitude
                h->last_voltages[i] = LTC2990_Code_To_Single_Ended_Voltage(h, code14);
            } else {
                h->last_voltages[i] = NAN;
            }
        }
    } else { //CURRENT role
        uint16_t raw15; int8_t valid;
        if (LTC2990_ADC_Read_New_Data(h, V1_MSB_REG, &raw15, &valid) == 0 && valid) {
            h->last_voltages[0] = LTC2990_Code15_To_CurrentA(raw15);
        } else {
            h->last_voltages[0] = NAN;
        }
        h->last_voltages[1] = NAN;
        h->last_voltages[2] = NAN;
        h->last_voltages[3] = NAN;
    }
}

/**
  * @brief  Puts the latest readings in the array passed
  * @param  Pointer to the LTC2990 handle
  * @param 	Pointer to the array to store values to
  */
void LTC2990_Get_Readings(LTC2990_Handle_t* handle, float* values) {
	for(int i = 0; i < 4; i++) {
		values[i] = handle->last_voltages[i];
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


uint8_t LTC2990_ADC_Read_New_Data(LTC2990_Handle_t *h, uint8_t msb_reg, uint16_t *raw15, int8_t *data_valid)
{
    uint16_t timeout = TIMEOUT;
    uint8_t status;
    uint8_t status_bit = status_bit_from_msb(msb_reg);
    if (status_bit == 0xFF) return 1;

    while (--timeout) {
        if (LTC2990_Read_Register(h, STATUS_REG, &status) != 0) return 1;
        if (((status >> status_bit) & 0x01) == 1) break;
        sleep_ms(1);
    }
    if (!timeout) return 1;

    uint8_t msb, lsb;
    if (LTC2990_Read_Register(h, msb_reg,     &msb) != 0) return 1;
    if (LTC2990_Read_Register(h, msb_reg + 1, &lsb) != 0) return 1;

    uint16_t code = ((uint16_t)msb << 8) | lsb;
    *data_valid = (code >> 15) & 0x01; //D15
    *raw15 = code & 0x7FFF; //keep Sign (D14) + D[13:0]
    return (*data_valid == 1) ? 0 : 1;
}

float LTC2990_Code_To_Single_Ended_Voltage(LTC2990_Handle_t *handle, uint16_t code14) {
	float voltage;

	code14 &= 0x3FFF;
	voltage = ((float)code14) * SINGLE_ENDED_LSB;

	return voltage;
}

float LTC2990_Code15_To_CurrentA(uint16_t raw15)
{	//All convertions in datasheet, look at it
    const float a_per_count = 19.42e-6f / RSENSE_OHM; //19.42 µV / R
    const uint16_t mag14 = (raw15 & 0x3FFF);          //D[13:0]
    const uint8_t  sign  = (raw15 >> 14) & 0x1;       //D14

    if (sign == 0) {
        return  (float)mag14 * a_per_count;
    } else {
        return -(float)(mag14 + 1U) * a_per_count;
    }
}


int8_t LTC2990_Read_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t* data) {

	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(handle->hi2c, handle->i2c_address << 1, reg_address, I2C_MEMADD_SIZE_8BIT, data, 1, TIMEOUT);
	if(status == HAL_OK) {
		return 0;
	}
	CDC_Transmit_Print("I2C Read Register failed, status: %d\n", status);
	return 1;
}


int8_t LTC2990_Write_Register(LTC2990_Handle_t *handle, uint8_t reg_address, uint8_t data) {
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(handle->hi2c, handle->i2c_address << 1, (uint16_t)reg_address, I2C_MEMADD_SIZE_8BIT, &data, 1, TIMEOUT);
	if(status == HAL_OK) {
		return 0;
	}
	CDC_Transmit_Print("I2C Write Register failed, status: %d\n", status);

	return 1;
}
