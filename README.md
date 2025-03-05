# PowerBoard25
This is the Powerboard for UB SEDS 2025 IREC

Using STM32CubeIDE, we need to convert these drivers from the old firmware:
- [ ] LTC2990 Driver

What we still need to implement:
- [ ] Powerboard logic
- [ ] CAN

How to use LTC2990:
To use, first init the chip with:
	LTC2990_Init()
	

	
	
	
	
  LTC2990_Init(&LTC2990_Handle, &hi2c2);

  while(1) {
	  LTC2990_Step(&LTC2990_Handle);


	  float voltages[4];
	  LTC2990_Get_Voltage(&LTC2990_Handle, voltages);

	  for (int i = 0; i < 4; i++) {
		  CDC_Transmit_Print("Voltage %d: %d.%03d V\n", i ,  (int)voltages[i], (int)((voltages[i] - (int)voltages[i]) * 1000));;
	  }

	  HAL_Delay(1000);
  }