#ifndef ArmDriver_h
#define ArmDriver_h

#include <inttypes.h>
#include "stm32l4xx_hal.h"
#include "main.h"

extern UART_HandleTypeDef huart3;

int LX16ABus_write_no_retry(uint8_t cmd, const uint8_t *params, int param_cnt,
		uint8_t MYID);

#endif
