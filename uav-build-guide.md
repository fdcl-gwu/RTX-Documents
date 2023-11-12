# UAV Build Guide
Written by GW RTX Drone Challenge Team

## Components Needed:

### Quadcopter:
- Motors (x4): T-Motor MN3110-17 KV-700
- Frame:
- ESC (x4): 
- Jetson Orin Nano
  - NVME SSD (recommended: Crucial P3 500GB)
- 4s 3-5000mAh LiPo Battery
- VN100 IMU
- 5V regulator
- Adafruit PCA9685 PWM Converter Board
- 5-6 Refelctive Markers
- Wires and mounting hardware


# Frame Assembly

## Step 1: Assemble the frame. 
This will include screwing arms to the main plate of the frame. Some frames will have power distribution pads on the frame base plate (labeled with "+" and "-"). If this is the case, create and attach a battery lead with XT60 connector to to the baseplate. To do this, solder a black and a red wire (between 12-16 AWG, but use same gauge for both wires) to a male XT60 connector (the one with the two, golden prongs). Then connect the other end of the red wire to the large "+" pad on the plate of the frame, and the black wire to the large "-". See Soldering 101 in Appendix for details about soldering.

IMG

You may also want to attach a large capacitor to the pads where you soldered the battery lead. Since our UAV will draw bursts of current, the capacitor will help keep the voltage stable.

## Step 2: Attach Motors and ESCs
For each of the arms on the frame, screw on the motor _using the 4 provided motor screws that came with the motor_. If the motor has mutliple screw holes in its base, use the 4 which are symetrically located around the shaft.

For each arm/motor, place an ESC on the upper side of the arm. Make sure to leave some open space on the arm so that we can later attach VICON reflective markers on the arms. You can attach the ESC using zipties, and it should be oriented on the arm such that the +/-/signal wires should face the center of the frame, and the three pads/wires on the opposite side of the ESC face the motor.

For the wiring of the ESC, connect the __three wires__ from the motor to the __three pads__ sometimes labeled A,B,C. The order doesn't matter for right now, as we will swap them later when testing motor spin direction. Then, on the other side of the ESC, connect a red wire (18 AWG) to the "+" and a black wire to the "-" pads on the other side of the ESC. Then, finding an available "+" and "-" pair on the frame base plate, connect the respective wires to those pads. 
- Note: since voltage is constant in a parallel circuit, each of the "+" and "-" pads on the frame base plate will share the same voltage coming from the LiPo battery.

Lastly, the thin black and white braided wire should already be attached to the ESC. The __white__ wire is the PWM Signal wire, while the __black__ wire is Ground (GND). 


## Step 3: Attach IMU
When attaching the IMU, placement is very important. Before doing anything, put a small rubber mat underneath the IMU in order to dampen the effects of the vibrations coming from the motors.

On the IMU are three axes - x,y,z - and they correspond to the directions of the IMU's coordinate system. When mounting, place the IMU __UPSIDE DOWN__ at the center of mass of the frame. Then, rotate the IMU such that the x-axis of the IMU is aligned with one of the arms of the frame (in the code, this arm will then be refered to as body axis 1 (b1, or arm1)). If done correctly, when looking at the underside of the frame, you should see the red part of the IMU with the x- axis arrow pointing along arm 1 of the frame.

IMG

### Additional Information:
If the IMU is not connected to the black connector, do the following:

First we need to prepare a 4 wire ribbon cable. The cable should be stripeed and split up near the end, and we need to solder on male bullet connectors. This can be done by placing a bundle of solid solder into the connector, similar to how we did it with the power supply connector. Apply heat with the iron using the small hole found on the side of the connector and push the wire further in until it is secured.

NATHAN IMG

Next it needs to be connected to the IMU using a connector. We use a ten pin connector, but the ribbon cable is only connected to it at 4 points. The image belowe shows the exact pins that should be inserted into the connector. Hot glue can be applied to the open side of the connector to keep the cables in place.

NATHAN IMG

## Step 4: PWM Converter Board:
The Jetson sends motor signals using I2C, but the ESCs we use only take in PWM signals. Thus, we need to use a converter board which converts the signal. 

When looking at the PCA9685 boardfrom the front (with the blocks of 4x4 pins on the bottom), we will be using the first 4x4 pin header block (columns labeled 0-4) to connect our ESCs. For the ESC on arm 1 of the frame (see Step 3), connect it's white wire to the PWM pin of column 0, and it's black wire to the GND pin of column 0. This should be straightforward, as many ESCs come with a plastic connector that can be plugged directly into the pins. Then, continuing clockwise from arm 1(when looking at the frame from above), connect ESC2 to the column 1 pins. Do this for the remaining ESCs.

At this point, the ESCs are connected to the converter board, but we have not yet connected the board to the Jetson. We will leave this for a later step, but for now, just make sure to keep the pins on the left side of the board accessible. Specifically, __GND, SCL,SDA, and VCC__. 

IMPORTANT: When attaching the converter board to the frame, make sure to insulate it's underside from any other electrical contact. It is very easy to short the board since all pins are exposed on both sides of the board.


### Additional Information: 
Depending on whether the board can be successfully addressed from the code, you may need to change the converter board's address. To do this, add a solder bridge on pads A0 and A1 pads in upper right corner of board. This will change the default board address from 0x40 to 0x43.

5V Regulator

## Step 5: 5V Regulator
Sometimes the Jetson's 5V output pins will not output exactly 5V, which can cause issues since we power our IMU and PWM Converter board with 5V. To solve this, we use a 5V regulator. On the regulator, connect the pad labeled "IN 6-35V" to "+" battery voltage, and "GND" to a "-" battery voltage. Then, connect the "OUT 5/12V" to the component needed a 5V power source, and the "-" pad to an available GND pad.

IMG

If multiple components need a 5V, consider using a small breadboard that you can plug things in to.

## Step 6: Mounting Jetson and connecting wires.
Using a 3D printed mount, attach the jetson to the top of the frame. Make sure not to pierce the underside of the Jetson with any screws that may be protruding from the top of the frame. The only constraint when mounting the jetson is that you can reach the pin row with all the wires that need to be plugged into it. Also, you will need to create a male XT60-to-barrel adapter so that the Jetson can be powered from the battery. You can use the same barrel that the Jetson Orin Nano comes with.

IMU Connections:
From | To
---|---
IMU 5V | 5V regulator "+" 
IMU GND| 5V regulator "-"
IMU RX| Jetson Pin 8
IMU TX| Jetson Pin 10

- If using the IMU with BAUD=230400, f=200Hz, orange wire is IMU RX, yellow wire is IMU TX.

PCA9685 Connections:
From | To
---|---
PCA9685 VCC | 5V regulator "+" 
PCA9685 GND | 5V regulator "-"
PCA9685 SDA | Jetson Pin 3
PCA9685 SCL | Jetson Pin 5

Refer to the following datasheets for additional details:
- VN100: p.26/101 https://mpt-internal.uni-hohenheim.de/lib/exe/fetch.php?media=sensors:vn-100_user_manual.pdf
- Jetson Orin Nano: https://jetsonhacks.com/nvidia-jetson-orin-nano-gpio-header-pinout/
- PWM Converter Board (PCA9685): https://learn.adafruit.com/16-channel-pwm-servo-driver/pinouts

### Final Comments
After everything is connected, make sure keep the area around the arms where the propellers will free. This may mean reorganizing wires, and taping things down.



# Jetson Orin Nano Configuration
## Step 1: Setup Jetson
See [jetson-setup.md](jetson-setup.md) in RTX-Documents.

After the jetson is configured, record it's IP Address. This can be found my connecting a monitor to the Jetson, and going to WIFI settings. Then seelct the small "Settings" wheel next to the connected WIFI network. 

  - For example, when connected to Basestation on the 2nd Floor of SEH, you should see an IP address that looks like 192.168.10.XX. This is the Jetson's IP address which we will use to ssh into.



## Step 2: Setup Flight Code
You will need two computers for this part: the Jetson Orin Nano (ROVER), and either a Linux or Mac computer (BASE). On the BASE computer, open two terminals:

### Terminal 1 (ROVER)
1. type ```$ ssh fdcl@192.168.10.34``` into a terminal, replacing the ```192.168.10.34``` with your Jetson's IP address and ```fdcl``` with the account username associated with your Jetson. Enter your Jetson's password.
2. Clone and build the flight code:
    1. Clone the fdcl-uav_rtx repo: ```$ git clone https://github.com/fdcl-gwu/fdcl-uav_rtx.git```. You'll need to be logged into github for this.
    2. Change into directory: ```$ cd fdcl-uav_rtx```
    3. Switch to quad_x_race branch: ```$ git checkout quad_x_race```
    4. The VRPN library is empty when you clone the flight code. To fix this, go into fdcl-uav_rtx/libraries and run ```$ git clone git@github.com:fdcl-gwu/vrpn.git```
        - If /libraries/vrpn/submodules/hidapi is empty, enter the /libraries/vrpn/submodules directory and run ```$ git clone https://github.com/signal11/hidapi.git```
    5. Go back to fdcl-uav_rtx/ directory and run ```$ . setup_uav.sh```
    6. When prompted, uncomment pc_linux in the 4 files
    7. From /build directory, run ```$ cmake ../ ```
    8. Finally, run ```$ make rover```

### Terinal 2 (Base)
2. Clone and Build Flight Code:
    1. Clone the fdcl-uav_rtx repo: ```$ git clone https://github.com/fdcl-gwu/fdcl-uav_rtx.git```. You'll need to be logged into github for this.
    2. Change into directory: ```$ cd fdcl-uav_rtx```
    3. Switch to quad_x_race branch: ```$ git checkout quad_x_race```
    4. The VRPN library is empty when you clone the flight code. To fix this, go into fdcl-uav_rtx/libraries and run ```$ git clone git@github.com:fdcl-gwu/vrpn.git```
        - If /libraries/vrpn/submodules/hidapi is empty, enter the /libraries/vrpn/submodules directory and run ```$ git clone https://github.com/signal11/hidapi.git```
    5. Go back to fdcl-uav_rtx/ directory and run ```$ sh setup_uav.sh```
          - On Linux, replace ```sh``` with ```.```
    6. When prompted, uncomment pc_linux in the 4 files
    7. From /build directory, run ```$ cmake ../ ```
    8. Finally, run ```$ make base```

Documentation reference: https://fdcl-gwu.github.io/fdcl-uav/

## Step 3: Configure Flight Code
Make the following code changes:

### Terminal 1 (ROVER)
1. src/fdcl_control.cpp
    - in ```fdcl::control::load_config(void)```, change ```fM_to_forces``` depending on where the motors are with respect to the body 1 axis (see "Geometric Control and Estimation for Autonomous UAVs in Ocean Environments
", Appendix B).
2. src/fdc_ekf.cpp
    - in ```fdcl::ekf::init(void)```, change ```R_bi``` and ```R_bi``` depending on whether using X or + shape.
3. rover.cfg
    - IMU:port: ttyTHS0 or ttyTHS2
    - IMU:baud_rate: either 230400 or 115200
    - WIFI:server_ip_addr: IP Address of Jetson
    - I2C:port: NOT USED
    - VICON:rover_quad: "NAME_OF_VICON_OBEJCT@192.168.10.1"
    - UAV:m_quad: mass of quadcopter
    - MOTOR:calib: coefficients from Motor calibration section 
4. include/common_types.hpp
    - Uncomment applicable DEFINE statements within first 100 lines of the file. We are using PWM escs, so uncomment ```#define PWM_ESC```
5. JHPWMPCA9685.h: 
    - ```PCA9685(int address=0x43)```: replace 0x43 with board address (default=0x40).
6. JHPWMPCA9685.c: 
    - kI2CBus = #: The I2C bus being used on the Jetson (/dev/i2c-#_
        - Pins 3,5 use I2C Bus 7 (to check, run ```$ sudo i2cdetect -y -r 7```)

### Terminal 2 (BASE)
The base is used primarily for displaying data collected from the rover. Thus, the code changes aren't as crucial on the base for right now.
1. base.cfg
    - WIFI: server_ip_addr: IP Address of Jetson
    - VICON:object: "NAME_OF_VICON_OBEJCT@192.168.10.1"
2. Extra Changes: same changes as ROVER

__After making code changes, remember to run ```$make rover/base``` again__

## Step 4: Run Flight Code
Before plugging in the abttery for the first time, __double check that all positive and ground wires are wired correctly!!__



### Terminal 1 (ROVER)
1. Run ```sudo ./rover```
### Terminal 2 (BASE)
1. Run ```./base```






The IMU will have a preset BAUD rate and frequency. The BAUD rate is either 230400 or 115200, and can be determined my using the VN100 software (ask Maneesh). Or, if you're lucky, it was written on the IMU itself in permanent ink by a former FDCL lab member.

TODO: explain more about X vs +

## Creating a VICON Object
We add small balls of reflexive material onto the drone so that it can be tracked by our VICON system. This small balls can be added anywhere on the frame or even the Jetson using double sided adhesive. The only thing to keep in mind is that we actually do not want the placement of these balls to be symmetric in pattern. Having an assymmetric arrangement allows the VICON system to know the orientation of the drone at all times. Note that you can also place balls on the actual Jetson itself.


Next, bring the drone into the netted area. The orientaion of the VICON system is marked by the "T" shape of black tape on the floor. It is important to align the body and vicon coordinate systems correctly:

VICON Frame:
```  
      Direction of SEH Elevators

               V2			       
                |
                |
               ___  V1      Direction of Lab workbench
```

### + Configuration
If using + configuration, align body axis 1 (the direction of IMU x-axis) of the drone with axis 1 of the VICON system.

```  
For + config:   R2			       
             W1 X R1 (b1,i1,v1)
                W2
```

### X Configuration
If using X configuration, the NEW body 1 axis will be between arms 1 and 2. Align this new b1 axis with axis 1 of the VICON system. Here, arm 1 is the arm with motor 1 on it. Then, arm 2 is clockwise from arm 1. See Coordinate Frame Documentation (TODO - link) for details.

Placement Diagram for x
```  
For X config: R2    R1
                 X    --> New b1 direction
		      W1    W2
```







Open up the GUI using the same method in steps 11 and 12 in the under the [Motor Test](#motor-test) section. Pay attention to the data labeled "__YPR__" (yaw, pitch, roll). For now we only have to worry about pitch and roll, so ignore the first row of data. Because our IMU was mounted upside down, we want the pitch to be 0 and the roll to be -180. Add small objects such as screws underneath the legs of the drone until these numbers match the figures we want fairly accurately. At this point, we are ready to create the VICON object.
1. Go to the computer with the VICON software installed. Turn on the VICON server 
2. Open the VICON software and click on the tab labeled "OBJECTS"
3. Uncheck any boxes that have already been selected
4. You should see the same number of reflexive balls on the computer screen as you have attached to the drone. If some are missing, move them to different areas of the frame until all can be seen on the software
5. In the bottom left of the screen next to "Create Object", give the drone a name and press "CREATE"
6. Select your new drone object under the onjects tab. If you move the drone around in the net, you should see corresponding movement on the screen
7. On the rover terminal, open the rover.cfg file found in the fdcl-uav. Search for a section that looks like:
```
 VICON:
  on: 1
  object: "JetsonCN@192.168.10.1"
  board: "board@192.168.10.1"
  payload: "Jetson@192.168.10.1"
```
On the object line, replace "JetsonCN", or whatever is there by default, with the name of the object you just created in the VICON software.

While you have the rover.cfg file open, it is also a good time to record the drone's weight. At this point, we can attach an external battery to the bottom of the frame using velco straps and double sided adhesive. When placing the drone on the scale, also add all the propellers, and propeller attachments that will eventually be added for the final flight. These components do not have to actually be attached to the drone yet, so they can just be placed loosely onto the scale. In the rover.cfg file, look for the `UAV` section. Under that, there should be a line that looks like `m: 1.75`. Replace this number with the measured weight of the drone in kilograms.

# Motor Calibration

# Flight Preparations
The inside of the netted area has to be cleaned up and prepared. Make sure the entire floor is filled with the foam puzzle mats. For the very first few flights, it is usually a good idea to take extra preparations in case of a crash. This includes elevating a net above the floor by clipping it to supports outside of the netted area. The picture below shows the netting being held up in the back of the photo, as well as the elevated platform that we use to take off. After the you have confirmed that the drone is stable after the first couple of flights, this bottom net is no longer necessary.

![Netted Area](/Photos/nettedArea.jpg)

Next, the propellers have to be attached. As stated before, motors 1 and 3 are spinning clockwise, which is important because the propellers are different depending on the direction you want them to spin. The propeller will spin towards the side that the elevated part of the blade is on. In other words, the leading edge travels in the direction of the rotation. Two clockwise propellers should be attached to motors 1 and 3, while two counterclockwise propellers should be attached to motors 2 and 4. The example below is motor 1, so it should rotate clockwise

 ![Propeller Direction](/Photos/propellerDirection.jpg)

Ensure that all loose wires are secured to the body of the drone using zipties or tape. 

## Flight Operation and Safety
Flying the drone can be very dangerous, so it is important to the take some saftey precautions. Anyone operating or watching the drone should be outside of the netted area, and keep a safe distance from it during flight. Everyone should also be wearing safety goggles. Additionally, only fly the drone when the per-cell voltage of the battery is between 4.2 and 3.8 volts.

# Appendix

## Soldering 101
### Through-hole 
Through-hole is used when a hole is already built into the board, and we have to push a component lead through it. In this build, we use this method to mount the power supply as well as any connector or header pins. This method is relatively straightforward, as you simply stick the lead through the hole, apply pressure with the soldering iron to the base of the lead, and feed solder into the joint until you create a cone-like shape of solder. 

### Surface Mounting
The other method is surface mounting, and is used whenever we have to mount any component directly to the board without the use of a premade hole. Generally for this method you want to begin by tinning the component that needs to be mounted. Tinning is simply applying flux to the component lead and then applying solder to create a coat of metal around the lead. You can usually apply solder to the lead by melting solder onto the soldering iron, and then touching the iron to the lead. Once this is done, apply more flux to the surface that you are mounting the component onto. Push the component into place and put pressure with the iron. You usually have to also put more solder onto the iron to create a more secure connection.

### XT60 Soldering
To connect the wires to the yellow XT60 connector, first you have to fill the connector with solder. This can be done by placing a bundle of solid solder into the connector, and melting it with the iron. After tinning the wires, you can remelt the solder in the connector and push the wires in to secure them. This connector will be connected to the female connector that we soldered onto the bottom metal plate of the frame earlier on.


After all the soldering has been finished, use a cotton swab with a cleaning solution such as isopropyl alchol to clean the board of any flux or other material that could be corrosive.


## TODOs: 
- add WIFI antennas
- tuning rover.cfg
- add PCA9685 to rover.cfg file, and rename


## IMU BAUD rate vs Frequency:
The BAUD rate is more specific to communication protocols, indicating how fast data is transmitted over a communication channel. It is the number of signal or symbol changes that occur per second in a communication channel (i.e. it's a measure of the rate of information transfer). Frequency, in the context of an IMU or sensor, refers to the rate at which the device samples or updates its measurements.