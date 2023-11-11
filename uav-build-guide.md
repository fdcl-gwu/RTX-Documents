# UAV Build Guide
Written by GW RTX Drone Challenge Team

## Components Needed:

### Quadcopter:
- Motors (x4): T-Motor MN3110-17 KV-700
- Frame:
- ESC (x4): 
- Jetson Orin Nano
  - NVME SSD (see Amazon purchase link)
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
Using a 3D printed mount, attach the jetson to the top of the frame. Make sure not to pierce the underside of the Jetson with any screws that may be protruding from the top of the frame. The only constraint when mounting the jetson is that you can reach the pin row with all the wires that need to be plugged into it.


The IMU will have a preset BAUD rate and frequency. The BAUD rate is either 230400 or 115200, and can be determined my using the VN100 software (ask Maneesh). Or, if you're lucky, it was written on the IMU itself in permanent ink by a former FDCL lab member.

From | To
---|---
IMU 5V | 5V regulator "+" 
IMU GND| 5V regulator "-"
IMU RX| Jetson Pin 8
IMU TX| Jetson Pin 10

- If using the IMU with BAUD=230400, f=200Hz, orange wire is IMU RX, yellow wire is IMU TX.

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






## PCB Assembly

 
  The IMU is the large red component found on the left side of the picture above. This sensor is responsible for collecting a lot of critical data including angular rate and acceleration. We put a small rubber mat underneath the IMU in order to minimize the effects of the vibrations coming from the motors. Additionally, the wired connector attached to the IMU (the black component south of the IMU in the left picture) is slightly more complicated. First we need to prepare a 4 wire ribbon cable. The cable should be stripeed and split up near the end, and we need to solder on male bullet connectors. This can be done by placing a bundle of solid solder into the connector, similar to how we did it with the power supply connector. Apply heat with the iron using the small hole found on the side of the connector and push the wire further in until it is secured.
  
 ![Ribbon Cable](/Photos/ribbonCable.jpg)

  Next it needs to be connected to the IMU using a connector. We use a ten pin connector, but the ribbon cable is only connected to it at 4 points. The image belowe shows the exact pins that should be inserted into the connector. Hot glue can be applied to the open side of the connector to keep the cables in place.

 ![Connector Pin Configuration](/Photos/ribbonCablePins.jpg)

We then create a stack with the PCB on the bottom, the Sprocket in the middle, and the Jetson on top.

![Stack](/Photos/combinedStack.jpg)

The stack can be attached to the drone by screwing it onto a plate which is fastened to the frame with zipties. If this is too loose, you can add double sided adhesive to the bottom of the plate before tieing it. This method can produce slight disturbances to the IMU measurements, but for indoor use the interference is negligible. If this becomes a problem you can drill the stack directly onto the frame of the drone. Once attached, connect the PCB power supply connector to the power supply cable soldered onto the front of the frame near the "S500" text.

## ESC Assembly
4 electronic speed controls (ESC) are needed. The ESC model used is the **BL-Ctrl V2.0**. The ESCs must have motor leads (3 female bullet connectors), a battery power input (1 male and 1 female bullet connector), and a BEC input (header pin) soldered on. Each ESC must also have a unique address that can set by soldering the part of the board labeled __ADR__ on the back side. The system for setting the address is as follows

Address| 1-2 |2-3|4-5
---|---|---|---
1|Open|Open|Open
2|Open|Closed|Open
3|Closed|Open|Open
4|Closed|Closed|Open
5|Open|Open|Closed
6|Open|Closed|Closed
7|Closed|Open|Closed
8|Closed|Closed|Closed

![Example of Soldered ESC](/Photos/Soldered_ESC_Labeled.jpg)

All four of the ESCs were tested for functionality by connecting them to DC motors and using a short Arduino program to control their speed using the serial monitor.

The four ESCs can be straped onto the arms of the frame with zipties, and the motor leads should be connected the DC motors. The specific order of these motor leads can be ignored for now, as we can change them later to reverse the rotation direction if needed. Next, each pair of the bullet connectors that we soldered onto the plate of the frame earlier needs to be connected to the power inputs on the ESCs. This can be seen on the right side of the picture below. Finally, the BEC inputs of each ESC has to be connected to a pair of header pins on the PCB. They can connected using female-female jumper wires, with each header pin on the PCB corresponding to one ESC. Clean and fasten all the wires using zipties. The picture below shows the underside of an arm where the ESC is strapped. The thinner, red and brown wires are connected to the BEC input of the ESC in the left of the picture, and they are connected to the Jetson, as seen in left of the second picture below.

![Wired ESC](/Photos/wired_ESC.jpg)
![Side View of Drone Arm](/Photos/sideViewArm.jpg)

At this point, you should also designate each motor a number. We started with an arbitrary motor 1, and labeled the rest by going clockwise. The specific motor that is designated as "motor 1" does not matter for now. To easily distinguish it, we wrapped the arm of motor 1 with white tape, as seen in the right of the picture above.


## Motor Test
The first motor test is just to check that each one of them works when connected to the Jetson and PCB. For this test, connect the __overall__ power suppy cable soldered onto the side of the metal plate of the frame to an external power supply. In this case, we use a DC power supply. Do not turn on the power supply until the drone has been connected. We power it up to about 12 volts.

![Drone connected to a DC power suppy](/Photos/dcMotorTest.jpg)

1. On one computer, open up 2 terminals. One will act as the 'base' and one will act as the 'rover'
2. On the rover terminal, ssh into the Jetson using `ssh ubuntu@JETSON_IP_ADDRESS`, replacing "JETSON_IP_ADDRESS" with the actual IP address of the Jetson. The default password for this is "ubuntu"
3. On the rover terminal clone the fdcl-uav GitHub using `git clone` followed by the URL that can be obtained by going to [this page](https://github.com/fdcl-gwu/fdcl-uav) and clicking the green button the says "Clone or download"
4. On the base terminal, git clone the fdcl-uav GitHub
5. On the rover terminal, `cd fdcl-uav/build/`.
6. On the rover terminal, `sudo ./rover`. If all the motors are working, you should see:
```
I2C: checking motors
I2C: motor 0 working 
I2C: motor 1 working 
I2C: motor 2 working 
I2C: motor 3 working 
```
7. On the rover terminal, `make rover`
8. On the base terminal, `cd` to the build folder and `make base` 
9. On the rover terminal, find the IP address of the rover, `ip addr show`, and copy the one found under "wlan0"
10. On the base terminal, edit the base.cfg file found in the main fdcl-uav folder. update "server_ip_addr" with the current rover IP address found in the previous step
11. On the rover terminal, `sudo ./rover`
12. On the base terminal in the build folder, `./base`
13. This should open up a GUI in a seperate window. __NOTE: Do not perform this test with the propellers attached__

![GUI](/Photos/guiSS.png)

14. On the GUI change it from "Mode" to "Att" in the upper left
15. Change the command mode to "Motor Test". There is a bug in the GUI, where the "Motor Test" button is labeled as "N/A" near the top right, so click that one
16. Click the "Motor" switch
17. Each motor on the drone should spin for about 2 seconds alternating. Make sure that motors 1 and 3 are clockwise, while 2 and 4 are counterclockwise. If a motor is spinning in the wrong direction, unplug the drone from the power supply, and swap any 2 of the ESC motor leads that are connected to a DC motor

## Creating a VICON Object
We add small balls of reflexive material onto the drone so that it can be tracked by our VICON system. This small balls can be added anywhere on the frame or even the Jetson using double sided adhesive. The only thing to keep in mind is that we actually do not want the placement of these balls to be symmetric in pattern. Having an assymmetric arrangement allows the VICON system to know the orientation of the drone at all times. Note that you can also place balls on the actual Jetson itself.

![Reflexive Ball Arrangement](/Photos/viconBalls.jpg)

Next, bring the drone into the netted area. The orientaion of the VICON system is marked by the "T" shape of black tape on the floor. It is important to align axis 1 (the arms with motors 1 and 3) of the drone with axis 1 of the VICON system.

![Drone Orientation](/Photos/viconOrientation.jpg)

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

## Flight Preparations
The inside of the netted area has to be cleaned up and prepared. Make sure the entire floor is filled with the foam puzzle mats. For the very first few flights, it is usually a good idea to take extra preparations in case of a crash. This includes elevating a net above the floor by clipping it to supports outside of the netted area. The picture below shows the netting being held up in the back of the photo, as well as the elevated platform that we use to take off. After the you have confirmed that the drone is stable after the first couple of flights, this bottom net is no longer necessary.

![Netted Area](/Photos/nettedArea.jpg)

Next, the propellers have to be attached. As stated before, motors 1 and 3 are spinning clockwise, which is important because the propellers are different depending on the direction you want them to spin. The propeller will spin towards the side that the elevated part of the blade is on. In other words, the leading edge travels in the direction of the rotation. Two clockwise propellers should be attached to motors 1 and 3, while two counterclockwise propellers should be attached to motors 2 and 4. The example below is motor 1, so it should rotate clockwise

 ![Propeller Direction](/Photos/propellerDirection.jpg)

Ensure that all loose wires are secured to the body of the drone using zipties or tape. 

## Flight Operation and Safety
Flying the drone can be very dangerous, so it is important to the take some saftey precautions. Anyone operating or watching the drone should be outside of the netted area, and keep a safe distance from it during flight. Everyone should also be wearing safety goggles. Additionally, only fly the drone when the battery is __at least__ 16.2 volts.

To fly, place connect the drone to the battery and place it on top of the elevated platform in the netted area. open up the GUI using the same steps as the [PCB Assembly](#pcb-assembly). You can use the buttons labeled "Idle", "Warm-up", etc. to perform basic movements of the drone. Before the drone actually flies, it is important to always use the "Warm-up" button, to check that all the propellers are working before taking off. Press the "Takeoff" button to get the drone in the air. The drone can be manually controlled using the keyboard on the host computer. Use the __WASD__ keys to move the drone around. Use the "__L__" key to make it go lower, and the "__P__" key to make it go higher. The "__M__" key kills the motors, so you should have one finger on this key at __all times__ in case you lose control on the drone. Note that this key should only be used in emergencies, and it is __not__ a suitable option to use when landing. The easiest way to operate the drone would be to use your left hand for the WASD keys, and use 3 fingers on your right hand to have access to the altitude control, and the kill keys. To properly land, make sure the drone is hovering above a flat surface within the flight area. You can either manually land by lowering the drone, or just pressing the "Land" button on the GUI.

![Finished Drone](/Photos/finishedDrone.jpg)



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