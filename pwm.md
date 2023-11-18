# Using PWM ESCs 
Last Updated: August 28, 2023

## Description:
The following instructions outline the steps necessary to use PWM ESCs with the `main` branch of the fdcl-uav flight code. The files ```include/JHPWMPCA9685.h``` and ```src/JHPWMPCA9685.cpp``` can be obtained from the ```quad_x_race``` branch of the fdcl-uav flight code. All other files are already present in the ```main``` branch.

## Files to Modify:
1. CMakeLists.txt
2. include/common_types.hpp
3. include/JHPWMPCA9685.h 
4. src/JHPWMPCA9685.cpp
5. include/fdcl/i2c.hpp
6. src/fdcl_i2c.cpp
7. rover.cfg

## Details:

### CMakeLists.txt
-   add ```${PROJECT_SOURCE_DIR}/src/JHPWMPCA9685.cpp``` to ```set(rover_libs_src)```(~ line 220).
-   add ```i2c``` to ```target_link_libraries(rover)``` (~ line 57).

### include/common_types.hpp
- add ```#define PWM_ESC``` to the other define statements already in the file. Uncomment if using PWM escs.

### include/JHPWMPCA9685.h
- copy entire file from the ```quad_x_race``` branch into your ```include/``` directory.
  - in line 50, change the address in ```PCA9685(int address=0x43);``` to the address of the servo driver board. The default is 0x40. However, on some Jetson's this address is reserved. To change the address on the board, add solder bridges to the pads A0, A1, etc, which form the binary number ADDED to 0x40 for the new address. For example, bridging A0 and A1, the new address becomes 0x43 (0000011 = 3).

### src/JHPWMPCA9685.cpp
- copy entire file from the ```quad_x_race``` branch into your ```src/``` directory.
  - in line 5, change ```kI2CBus = 0;``` to the I2C bus used on the Jetson.
    - For ORIN Nano: Pins 3 and 5 are on I2C bus 7, and pins 27 and 28 are on I2C bus 1.
    - For TX2: Pins 3 and 5 are on I2C bus 1, and pins 27 and 28 are on I2C bus 0.

### include/fdcl/i2c.hpp
- add ```#include "JHPWMPCA9685.h"``` to other includes at top of file (~ line 21).
- add ```PCA9685 *pca9685;``` to private class member variables (~ line 102).
- add ```int map(int degree, int in_min, int in_max, int out_min, int out_max);``` to private class member functions (~ line 155).
- add ```Vector3 calibPWM;``` to private class member variables (~line 122).

### src/fdcl_i2c.cpp
- replace ```fdcl_i2c.cpp``` from main with ```fdcl_i2c.cpp``` from this branch. 
- #### __NOTE__: You may encounter an issue where the ESCs don't receive the correct PWM signals. The default should work (204-409), however if motors are not acting correctly (such as beeping beyond the normal startup tones), we need to do additional calibration. We noticed this issue when switching from the Jetson TX2 to the Jetson Orin Nano. See the following steps:
  1. Arrange a motor calibration setup (see motor-calibration repo).
  2. Find the min throttle command:
      - The motor should begin spinning at command 50; this is used as the min throttle command, as in ```fdcl::i2c::warmup(bool motor_on)```.
      - To achieve this, adjust the PWM range in ```map()``` in ```fdcl::i2c::write_to_motor(int motor_index):```, starting from the default 204, until the command of 50 in ```warmup``` causes the motor to begin rotating. Then, calibrate the motor as you would normally for command values from 50 to 250.
  3. Once thrust values for commands 50-250 are determined, do the same for 0-40. This will give you the PWM value for command 0 needed for arming in ```pca9685->setPWM(i,0,204);``` in ```fdcl::i2c::open(void)```.

### rover.cfg
- Under ```MOTOR:```, add line ```calibPWM: 68.01, 0.4939, 26.61```. Replace these values after calibration.