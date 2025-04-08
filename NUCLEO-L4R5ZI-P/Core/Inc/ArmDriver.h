#include <inttypes.h>
#include "stm32l4xx_hal.h"
#include "main.h"

void LX16ABus_init(UART_HandleTypeDef* huart);
int LX16ABus_write_no_retry(uint8_t cmd, const uint8_t *params, int param_cnt, uint8_t MYID);
void LX16ABus_set_servo(uint8_t servoID, uint16_t angle, uint16_t timeInMs);
