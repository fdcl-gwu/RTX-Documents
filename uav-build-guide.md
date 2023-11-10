# UAV Build Guide
_Written by GW RTX Capstone Team_
## Getting Started
This build was done entirely at the Flight Dynamics and Control Lab at GWU. 
This specific build uses the **Readytosky S500 Kit**. The finished frame is shown below, however additonal work needs to be done before the frame can be completed.

![Readytosky S500 Frame](/Photos/init_frame.jpg)
 
Before screwing on the arms and top metal plate, we have to do some work on the bigger, bottom metal plate. Get the frame to the state shown below.

![Lower Plate](/Photos/lowerPlate.jpg)

We need to solder on pairs of wires with bullet connectors attached. To attach the wires to the frame, use the exposed golden points found on each corner of the frame. These points are circled in red in the first picture below. A wire with a male bullet connector should be soldered to the postive side, and a wire with a female connector should be soldered to the negative side. We can secure the wires using a similar process to the through hole mounting described later in this guide under [PCB Assembly](#pcb-assembly). This needs to be done for every DC motor, giving us 4 pairs of wires soldered to the frame in total. These wires will eventually be connected to the ESCs, as described later on.

![Locations to solder wires for ESCs](/Photos/lowerPlateESCsolder.jpg)
![Pair of wires attached to each corner of the frame](/Photos/ESCframeWires.jpg)

 Next we need to solder on a __female__ power supply connector to the front of the frame, near the __S500__ text. Once again, more detailed instructions about how to solder the power supply connector to the wires can be found under [PCB Assembly](#pcb-assembly). This power supply will eventually be connected to power the PCB and Jetson. 
 
![Locations to solder Jetson power supply](/Photos/lowerPlateJetsonSolder.jpg)
![Jetson Power Supply](/Photos/jetsonFrameWires.jpg)

 Lastly, a __male__ power supply connector has to be soldered onto the side of the plate near the large "__+__" and "__-__" symbols. This connector will eventually be connected to the overall DC power supply.

![Locations to solder overall power supply wires](/Photos/lowerPlatePowerSolder.jpg)
![Overall Power Supply](/Photos/powerFrameWires.jpg)

After this has been done, the arms and top plate can be screwed on. We also attach a DC motor to each arm of the drone, with their wires pointing towards the middle of the frame.


The computer we use is the NVIDIA Jetson TX2 with a [Sprocket Carrier Board](http://connecttech.com/product/sprocket-carrier-nvidia-jetson-tx2-jetson-tx1/). In order to configure the Jetson, we must attach it to the official NVIDIA Jetson Development Board.

![NVIDIA Jetson Development Kit](/Photos/NVIDIAdevKit.jpg)
## Configuration
*The configuration of the Jetson TX2 is loosely based off the information in steps 1-3 in this [link](https://fdcl-gwu.github.io/jetson-config/).*

Configuration consists mainly of two parts: installing JetPack, and installing the board support package (BSP). The steps to install JetPack are as follows:
1. Download and install the [NVIDIA SDK Manager](https://docs.nvidia.com/sdk-manager/download-run-sdkm/index.html#download)
2. Run the manager and change **Target Hardware** to **TX2**
3. Select all and accept. Make sure that the Jetson is connected to the development board with a keyboard, mouse, and monitor as shown above. It should also have a connection to a host computer using the micro USB port
4. Wait for installations to run on the TX2. After the flashing is completed, the TX2 will reboot. Once the login screen appears, enter the username and password of the TX2 onto the screen that appears on the **host computer**
5. Finish and exit

The next step is to install the BSP. This guide assumes you are using JetPack 3.3, and L4T V32.1.0. If you are using a different version, instructions can be found at this [link](http://connecttech.com/resource-center/cti-l4t-nvidia-jetson-board-support-package-release-notes/).
1. Install the BSP [package](http://www.connecttech.com/ftp/Drivers/CTI-L4T-V121.tgz)
2. Copy the package into /nvidia/nvidia_sdk/64_TX2/Linux_for_Tegra/
3. Using the terminal, `cd` into the folder that you just copied the package into. Extract the BSP: `tar -xzf CTI-L4T-V121.tgz`
4. `cd` into the CTI-L4T folder
5. Install the script: `sudo ./install.sh`
6. Move back to the Linux_for_Tegra folder: `cd ..`
7. Use CTI assisited flashing: `./cti-flash.sh`
8. Select option **5-Sprocket**, followed by **Base**, followed by **TX2**

## PCB Assembly
We use a custom printed circuit board that must be wired according to the schematic found [here](https://github.com/fdcl-gwu/jetson-pcb/releases/tag/v2.0). This part of the build is much easier with an intermediate understanding of electronics and experience with soldering is also very helpful. There are mainly 2 different methods for soldering the components: through-hole and surface mounting. Through-hole is used when a hole is already built into the board, and we have to push a component lead through it. In this build, we use this method to mount the power supply as well as any connector or header pins. This method is relatively straightforward, as you simply stick the lead through the hole, apply pressure with the soldering iron to the base of the lead, and feed solder into the joint until you create a cone-like shape of solder. 

![Through Hole Example](/Photos/throughHoleExample.jpg)

The other method is surface mounting, and is used whenever we have to mount any component directly to the board without the use of a premade hole. This is used to attach most of the parts in this build including resistors, capacitors, and LEDs. Generally for this method you want to begin by tinning the component that needs to be mounted. Tinning is simply applying flux to the component lead and then applying solder to create a coat of metal around the lead. You can usually apply solder to the lead by melting solder onto the soldering iron, and then touching the iron to the lead. Once this is done, apply more flux to the surface that you are mounting the component onto. Push the component into place and put pressure with the iron. You usually have to also put more solder onto the iron to create a more secure connection.

![Surface Mount Example](/Photos/surfaceMountExample.jpg)

In the example above, each of the three legs have been tinned and surface mounted, as well as the larger black body.

The component with the black and red wires shown below is the PCB power supply. To connect the wires to the yellow __male__ connector, first you have to fill the connector with solder. This can be done by placing a bundle of solid solder into the connector, and melting it with the iron. After tinning the wires, you can remelt the solder in the connector and push the wires in to secure them. This connector will be connected to the female connector that we soldered onto the bottom metal plate of the frame earlier on.

![PCB Power Supply](/Photos/PCBpowerSupply.jpg)

 If the schematic is followed correctly, the board should match as follows.

![Finished PCB](/Photos/CombinedPCB.jpg) 

 Take note of the small resistors that have been soldered onto the board found on the right side of the picture. After all the soldering has been finished, use a cotton swab with a cleaning solution such as isopropyl alchol to clean the board of any flux or other material that could be corrosive.
 
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
