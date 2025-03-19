Glove Controls (IMU, Flex Sensors, Display) are in Nucleo folder
Drone Controls are in Steval folder

**Drone Controls Research:**
- To control the drone in reference to it's IMU with something resembling a joystick, the drone moves with the following commands
      - gRUD = (joydata[3]-128)*(-13);
      - gTHR = joydata[4]*13;
			- gAIL = (joydata[5]-128)*(-13);
			- gELE = (joydata[6]-128)*13;
- These commands connect to the control of Rudder, Throttle, Ailerons, and Elevator (see this forum: https://deviationtx.com/forum/6-general-discussions/5846-how-to-change-channel-1-from-1-ele-to-1-thr#gallery-2)
- We need to assign our glove IMU and flex sensors to control different aspects of this control

**Flex Sensor Info:**
- When attached to a finger, translucent white touching skin, solder connects pointing to palm
- Finger Straight = 90-115kohm
- Finger Bent = 60-80kohm
