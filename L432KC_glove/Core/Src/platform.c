/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#include "platform.h"

extern I2C_HandleTypeDef hi2c3;

uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t *value)
{
	uint8_t status = 0;
		uint8_t data_write[2];
		uint8_t data_read[4];

		data_write[0] = (RegisterAdress >> 8) & 0xFF;
		data_write[1] = RegisterAdress & 0xFF;
		status = HAL_I2C_Master_Transmit(&hi2c3, dev, data_write, 2, 100);
		status = HAL_I2C_Master_Receive(&hi2c3, dev, data_read, 4, 100);
		*value =  ((data_read[0] << 24) | (data_read[1]<<16) |
				(data_read[2]<<8)| (data_read[3]));
		return status;
}

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t RegisterAdress, uint16_t *value)
{
	uint8_t status = 0;
		uint8_t data_write[2];
		uint8_t data_read[2];

		data_write[0] = (RegisterAdress >> 8) & 0xFF;
		data_write[1] = RegisterAdress & 0xFF;
		status = HAL_I2C_Master_Transmit(&hi2c3, dev, data_write, 2, 100);
		status = HAL_I2C_Master_Receive(&hi2c3, dev, data_read, 2, 100);
		*value = (data_read[0] << 8) | (data_read[1]);
		return status;
}

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t RegisterAdress, uint8_t *value)
{
	uint8_t status = 0;
		uint8_t data_write[2];
		uint8_t data_read[1];

		data_write[0] = (RegisterAdress >> 8) & 0xFF;
		data_write[1] = RegisterAdress & 0xFF;
		status = HAL_I2C_Master_Transmit(&hi2c3, dev, data_write, 2, 100);
		status = HAL_I2C_Master_Receive(&hi2c3, dev, data_read, 1, 100);
		*value = data_read[0];
		return status;
}

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t RegisterAdress, uint8_t value)
{
	uint8_t data_write[3];
		uint8_t status = 0;

		data_write[0] = (RegisterAdress >> 8) & 0xFF;
		data_write[1] = RegisterAdress & 0xFF;
		data_write[2] = value & 0xFF;
		status = HAL_I2C_Master_Transmit(&hi2c3, dev, data_write, 3, 100);
		return status;
}

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t RegisterAdress, uint16_t value)
{
	uint8_t data_write[4];
		uint8_t status = 0;
		data_write[0] = (RegisterAdress >> 8) & 0xFF;
		data_write[1] = RegisterAdress & 0xFF;
		data_write[2] = (value >> 8) & 0xFF;
		data_write[3] = value & 0xFF;
		status = HAL_I2C_Master_Transmit(&hi2c3, dev, data_write, 4, 100);
		return status;
}

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t value)
{
	uint8_t data_write[6];
		uint8_t status = 0;

		data_write[0] = (RegisterAdress >> 8) & 0xFF;
		data_write[1] = RegisterAdress & 0xFF;
		data_write[2] = (value >> 24) & 0xFF;
		data_write[3] = (value >> 16) & 0xFF;
		data_write[4] = (value >> 8) & 0xFF;
		data_write[5] = value & 0xFF;
		status = HAL_I2C_Master_Transmit(&hi2c3, dev, data_write, 6, 100);
		return status;
}

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t TimeMs)
{
	HAL_Delay(TimeMs);
	return 0;
}
