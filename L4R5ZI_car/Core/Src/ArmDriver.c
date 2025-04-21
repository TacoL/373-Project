// write a command with the provided parameters
// returns true if the command was written without conflict onto the bus
#include "ArmDriver.h"
UART_HandleTypeDef* LX16ABus_huart;
float conversionRatio = 0.24; // 240 degrees/1000 steps = 0.24 degrees/step

void LX16ABus_init(UART_HandleTypeDef* huart)
{
	LX16ABus_huart = huart;
}

/*
 * Range of angle is 0-240 degrees
 * Range of time is 0-30000 ms
 */
void LX16ABus_set_servo(uint8_t servoID, uint16_t angleInDegrees, uint16_t timeInMs)
{
	uint16_t angleInSteps = angleInDegrees / conversionRatio;

	char data[4];
	data[0] = (angleInSteps & 0xFF);
	data[1] = (angleInSteps & (0xFF << 8)) >> 8;
	data[2] = (timeInMs & 0xFF);
	data[3] = (timeInMs & (0xFF << 8)) >> 8;

	LX16ABus_write_no_retry(1, (const uint8_t *)data, 4, servoID);
	//HAL_Delay(timeInMs);
}


int LX16ABus_write_no_retry(uint8_t cmd, const uint8_t *params, int param_cnt,
		uint8_t MYID) {
	if (param_cnt < 0 || param_cnt > 4)
		return 0;

	// prepare packet in a buffer
	int buflen = 6 + param_cnt;
	uint8_t buf[buflen];
	// uint8_t ret[buflen];
	buf[0] = 0x55;
	buf[1] = 0x55;
	buf[2] = MYID;
	buf[3] = buflen - 3;
	buf[4] = cmd;
	for (int i = 0; i < param_cnt; i++)
		buf[5 + i] = params[i];
	uint8_t cksum = 0;
	for (int i = 2; i < buflen - 1; i++)
		cksum += buf[i];
	buf[buflen - 1] = ~cksum;

//// clear input buffer
//	int junkCount = 0;
//	HAL_Delay(2000);
//	while (available()) {
//		if (junkCount == 0) {
////			if (_debug)
////				Serial.print("\n\t\t[ ");
//		}
////		if (_debug)
////			Serial.print(" " + String(read()) + ", ");
////		else
//			read();
//		HAL_Delay(3000);
//		junkCount++;
//	}

//	if (_debug && junkCount!=0) {
//		Serial.print("]\n ");
//		Serial.println(
//				"\t\t Junk bytes = " + String(junkCount) + " id " + String(MYID)
//						+ " cmd " + String(cmd) + " last cmd "
//						+ String(lastCommand));
//	}


	// lastCommand = cmd;
	// send command packet
//	uint32_t t0 = millis();
//	if(!singlePinMode)
//	if (myTXFlagGPIO >= 0) {
//		digitalWrite(myTXFlagGPIO, 1);
//	}
	HAL_UART_Transmit(LX16ABus_huart, buf, buflen, HAL_MAX_DELAY);
//	if(!singlePinMode)
//	if (myTXFlagGPIO >= 0) {
//		digitalWrite(myTXFlagGPIO, 0);
//	}
	// expect to read back command by virtue of single-pin loop-back
//	uint32_t tout = time(buflen+4) + 4; // 2ms margin
//	int got = 0;
	int ok = 1;
//	if(!singlePinMode){
//		if (_deepDebug)
//			Serial.println("RCV: ");
//		while ((got < buflen) && ((millis() - t0) < tout)) {
//			if (available()) {
//				ret[got] = read();
//				if (ret[got] != buf[got]) {
//					ok = false;
//				}
//				got++;
//			}
//		}
//		if (got<buflen){
//			ok = false;
//
//		}
////		if (_debug) {
////			if (!ok) {
////				Serial.print("\n\n\tWrote:[ ");
////				for(int i=0;i<buflen;i++){
////					Serial.print(" " + String(buf[i]) + ", ");
////				}
////				Serial.print("] id " + String(MYID)
////						+ " cmd " + String(cmd) + " last cmd "
////						+ String(lastCommand));
////
////				Serial.print("\n\tGot  :[ ");
////				for(int i=0;i<got;i++){
////					Serial.print(" " + String(ret[i]) + ", ");
////				}
////				Serial.print("] in "+String(millis()-t0)+"\n");
////			}
////		}
//	}

	return ok;
}

void ArmPos(int pos){
	LX16ABus_set_servo(5, 200-((pos/100.0)*80), 120);
	LX16ABus_set_servo(4, 180-((pos/100.0)*60), 120);
	LX16ABus_set_servo(3, 165-((pos/100.0)*90), 120);
	LX16ABus_set_servo(2, 180, 120);
	LX16ABus_set_servo(6, 30, 120);

}

