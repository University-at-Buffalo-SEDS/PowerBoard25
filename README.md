# PowerBoard25
This is the Powerboard for UB SEDS 2025 IREC

Using STM32CubeIDE, we need to convert these drivers from the old firmware:
- [ ] LTC2990 Driver

What we still need to implement:
- [ ] Powerboard logic
- [ ] CAN

How to use LTC2990 Driver:
To use, first init the chip with:
	LTC2990_Init(LTC2990_Handle_t *handle, I2C_HandleTypeDef *hi2c);
	
	ex. usage:
	LTC2990_Init(&LTC2990_Handle, &hi2c2)

Next, step to get the latest voltage values:
	void LTC2990_Step(LTC2990_Handle_t *handle);
	
	ex. usage:
	LTC2990_Step(&LTC2990_Handle);

To get these newly collected values use LTC2990_Get_Voltage
This function will not return the voltages, instead, initialze an array in the main.c with
the voltages values you want to store. Pass a pointer to that array in the function.
	void LTC2990_Get_Voltage(LTC2990_Handle_t *handle, float* voltages);
	
	ex. usage:
	float voltages[4];
	LTC2990_Get_Voltage(&LTC2990_Handle, voltages);