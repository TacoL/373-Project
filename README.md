# 373 Project
Glove Controls (IMU, Flex Sensors, Display) are in Nucleo folder

## Links to Nucleo Pinouts:
Car: [L4R5ZI](https://os.mbed.com/platforms/NUCLEO-L4R5ZI/) \
Glove: [L432KC](https://os.mbed.com/platforms/ST-Nucleo-L432KC/)

## Component Information
### Flex Sensor
- **Note: Different sensors may have different resistances.**
- When attached to a finger, translucent white touching skin, solder connects pointing to palm
- Finger Straight = 90-115kohm
- Finger Bent = 60-80kohm

## (Abandoned) Drone Research
Drone Controls are in Steval folder
**Drone Fly Analysis**
- 3 NW, 2 NE, 4 SW, 1 SE (4 - M4, 2 - M2, 3 - M3, 1 - M1) = doesn't get off ground and tips forward (N) motors 1 and 4 are stronger
- 3 NW, 2 NE, 4 SW, 1 SE (2 - M4, 4 - M2, 1 - M3, 3 - M1) = doesn't get off ground and tips to left (W) motors 1 and 2 are stronger
- 3 NW, 2 NE, 4 SW, 1 SE (2, 4, 3, 1) = doesn't get off ground and turns in place left (turns CC)

**Drone Controls Research:**
- To control the drone in reference to it's IMU with something resembling a joystick, the drone moves with the following commands
      - gRUD = (joydata[3]-128)*(-13);
      - gTHR = joydata[4]*13;
			- gAIL = (joydata[5]-128)*(-13);
			- gELE = (joydata[6]-128)*13;
- These commands connect to the control of Rudder, Throttle, Ailerons, and Elevator (see this forum: https://deviationtx.com/forum/6-general-discussions/5846-how-to-change-channel-1-from-1-ele-to-1-thr#gallery-2)
- We need to assign our glove IMU and flex sensors to control different aspects of this control
