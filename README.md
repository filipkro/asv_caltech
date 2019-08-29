#Running the Boat in remote control mode
##Powering the boat
Make sure the boat is properly powered before attempting to turn it on. The boat
requires at least **three** batteries. There are six in total in our possession.
Connect the batteries by connecting the Anderson connector on each battery to 
the Anderson connectors on the Vessel Control Unit (VCU, also the blue/silver 
control box in the hull). The VCU connector should be labeled with **24V**.

WARNING: the GPS, LiDAR, and Servo all have Anderson connectors. GPS and Servo 
are powered by **12V** supplies that comes out from the **front of the VCU**. 
Connecting to the wrong port will severely damage the equipment.

##Connecting the servo
The boat is slightly modified to allow control from the onboard computer. Unforunately,
we have to bypass the control signal from the VCU (which hosts the radio module) sometimes
to make this happen. To give VCU control to the servo motor, find the yellow servo
cable from the front of the VCU. Connect the end that labeled **Servo RC signal** to
the end from the servo that labeled **Servo signal**. These are three wire connectors 
that are red, white, and black in color. *Make sure the colors aligned*. Then connect
the part that labeled **Servo Power** to the **Servo Power** from the servo. The 
connectors are Anderson connectors.

##Connecting the ADCP
If you wish to use the ADCP, make sure it is properly powered by connecting the 
alligator clips from the ADCP cable to the **12V** Anderson connectors. There should 
also be an 9 pin DB9 connector from the same cable. Connect that to one of the
onboard USB-Serial adapter and to your computer. 

##Turning on the boat
To turn on the boat, first power on the RC controller. After the title screen, 
you should see a screen displaying some information about your control mode 
and boat status. Then, reach into the boat and find the red key on top of the VCU.
It should be located in the top left corner. Turn it to the left until it snapped
into position. You should hear a *beep* from the controller indicating radio link
has been established. Now you should be able to control the boat.

##Controller mode
The controller allows different controlling mode for the boat. The mode that the 
controller is in right now allows separate control to the left and right thruster,
as well as servo control with the left stick. To change that, tap the control mode
display on the upperleft-most corner. You should be able to see three additional 
control mode you can select. 

##Battery status
The battery of the controller is display on the top right corner in terms of 
percentage. The voltage level of the boat should be displayed immeidately to its
left. The controller will beep if the boat battery level drops below certain level.

## Shutting down the boat
Shut down the VCU by turning the key back into its original position. The controller
will beep once the radio link is disconnected. Shut down the controller.

Note: it is important to shutdown the VCU first. One of the thruster will randomly turn
on if the controller is shutdown first. 


# Running the Boat through on board computer

## Powering the boat
Same as remote control mode.

## Connecting the ADCP
Same as remote control mode. 

## Connecting the servo 
The purpose of this step is to return control to the onboard computer, which
controls the boat via an Arduino. To do this, connect the end of servo cable that
labels **Servo signal** to the **Servo Arduino signal** from the Arduino. Make
sure the colors (red, black, white) aligns. 

## 

## Connect to lidar:
ssh into the Jetson:
```
$ ssh nvidia@192.168.1.100
```
Once this is done run the following command on the Jetson:
```
$ sudo dnsmasq -C /dev/null -kd -F 192.168.1.50,192.168.1.80,100h -i eth0 --bind-dynamic
```

Wait for about 30s, until dnsmasq-dhcp: DHPACK(eth0) 192.168.1.78 .... os1XXXXXXXXX appears.
You can now quit this process by ctrl+C in that terminal. Now you should be connected (try it by ping 192.168.1.78)

## Run program:
ssh into Jetson in at least two terminals:
```
$ ssh nvidia@192.168.1.100
``` 
In one of the Jetson terminal prompts (ssh) (this is where debug meassages are printed):
```
$ roslaunch asv_controller smart_cntrl.launch
```
In another Jetson terminal (ssh):
```
$ roslaunch lidar_reader lidar_transform.launch
```
Chose what program (Smart, Waypoint, Transect) you want to run, Smart is default. Replace <program> with Smart, Waypoint or Transect:
```
$ rosparam set /nav_mode <program>
```
If the Smart program is chosen the mode (Upstream, Transect) needs to be specified. If no mode is chosen the program will imediately go to the Home/Finished state. Replace <mode> with Upstream or Transect:
```
$ rosparam set /smart/mode <mode>
```

Open Rviz on the host computer (not ssh shell):
```
$ rviz
```
If Rviz not configured to show the wanted topics these can be added by pressing the add button in the buttom left corner. 
Choose add by topic and add the wanted points. Add for instance lidar scans, reference points or whatever is of interest.
Save the changes, ctrl+C, to keep the vizualised topics until next time. If 'Smart' mode is running chose a start point by clicking
2D nav goal in the top bar of Rviz, then click on the map where you want the boat to go.

In another terminal prompt:
```
$ rosparam set /run True
```
  (the next command will start the vehicle)
```
$ rosparam set /motor_control/sim False
```
As long as /motor_control/sim is True no command is published to the motors. The I-parts in the controllers are set to zero as long as /motor_control/sim is True. By setting this to False you start the program and the boat will start go to the point specified in Rviz. Once the boat is started the I parts of the controllers are activated again by setting /I/reset to False.
