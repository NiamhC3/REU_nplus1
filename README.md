# REU_nplus1
User Guide
CAN Overview
-CAN Basics
-CAN Debugging
Odrives & Motors
-Odrive Configs
-Setting CAN Resistors
Mechanical
-Onshape Motor Mount
-Tendon Routing
-Linear Approximation of Joint Coupling
-Motor Calibration
Wiring and Power
-Wiring Diagram
Control
-Feedforward
-Torque Balancing
Code
-Github link
-Libraries Used
-Known Code Errors
-UI
Known Major Issues and Recommendations





User Guide
1. Ensure all motors are calibrated, and calibrations are saved. Power down the Odrives and disconnect all Odrives from the computer
2. Connect the teensy to the computer via USB.
3. If using the UI, wait to input desired joint angles. If not using the UI, change the tempJointAngles variable to store the desired joint angles.
4. Run the platformIO script. Once “waiting for Odrive 0” prints in the serial, provide power to the Odrives and motors. If the code doesn’t get to that point, try running it again.
5. The first time the Odrives attempt to enter closed loop control after a power cycle, they usually get stuck, and it can take up to a couple minutes for the Odrives to enter closed loop. In addition, Odrive occasionally falsely send a signal that they have entered closed loop. To avoid this, when the serial prints show that the odrives are attempting to enter closed loop control, instead of waiting for all motors to print “8”, (closed loop control axis state), rerun the whole program. The motor drivers should all immediately enter closed loop control each time until the next power cycle. 
6. When the Odrives begin to run, the finger should move to the upright position, as far clockwise as it can go for splay, (looking from behind the finger). If it fails to move upright, it has collected incorrect motor position offsets, and all following position calculations will be incorrect. Rerun the code until the offsets are collected at the upright position. (Currently, this code is not functioning properly, and the finger may need to be manually moved to the upright position to correctly collect offsets. Consider increasing the delay so there is more time to do this)
For UI:
7. To run the UI, first close or disconnect platformIO from the usb port. Then, run the python script through command prompt. Enter any desired joint angles through the UI.

Notes:
Most unpredictable behaviour is due to a calibration or encoder issue. If a recalibration does not fix the issue, double check that all magnets are in the proper positions for their encoders, and that all magnet mounts are secure.

CAN Overview
CAN Basics
Basic CAN function
CAN is a communication protocol similar to I2C. It functions on a differential voltage provided by two wires, CANH, and CANL, (CAN high and CAN low). The CANH wire carries a voltage of either 2.5 or 3.5V, and the CANL carries either 1.5 or 2.5V. Each wire can be set to carry either voltage, and the differential voltage between the wires gives either 0V or 2V. A large number of devices can be connected to a single CAN line by connecting all the CANH lines in parallel, and all the CANL lines in parallel. Microcontrollers receive CAN communication via an RX and TX pin. RX carries a scaled differential voltage to the Teensy, while the TX carries outgoing messages. These messages are converted to a CANH and CANL voltage by a transceiver. The transceiver can either be included on the microcontroller dev board or such as in the case of the Teensy, it must be connected externally.
Node IDs
Because all CAN devices are connected on a single CANH and CANL pair, the receiving device distinguishes between signals by looking at the node ID of a device. The node ID is an n bit value, (in the case of ODrives, default 6 bit), that is unique to each device. This value is broadcasted at the beginning of every single message to identify the device. 
Baudrate
Baudrate, or bitrate, is the speed at which a device sends and receives data. This must be configured for each device to be the same, otherwise messages will not be received. 
Heartbeat
The heartbeat of a CAN device is a signal that the device sends to confirm that it, and communication, are both working. It can be configured to any frequency, (in this case, 10 Hz).
CAN for the Teensy 4.1
The Teensy 4.1 supports CAN, but does not have a built in transceiver. A transceiver converts incoming voltage differentials to readable signals for the teensy. Teensy can send messages, but cannot receive them. This CAN setup uses a custom pcb from a prior HAND project. It has CAN 1, CAN 2, and CAN 3 capabilities, (the Teensy 4.1 supports up to 3 CAN busses), with 4 CAN bus ports for each. Transceivers for each CAN bus on the teensy can be added to the board as needed. 
CAN Physical Requirements
CAN needs to be wired in a specific way to function correctly. The following must be done for signals to be readable and low noise:
	-Twisted wires, (This ensures any electromagnetic interference is applied evenly to both CANH and CANL, which reduces the noise in the differential voltage. This is especially important for long wires or wires around strong electromagnetic fields)
	-Terminating resistors, (CAN require 120Ohm terminating resistors. These go between CANH and CANL on the ends of the lines. These help with noise reduction, and are a requirement to function properly)
	-Common ground, (Because CAN measures voltages, it is necessary for all CAN devices to share a common ground. This ensures all voltages sent across CANH and CANL are the values they are supposed to be.)

CAN Debugging
While CAN is an especially reliable method of communication due to the twisted wires, there are many ways that CAN can go wrong in setup.
The following is a list of potential problems that may be encountered:
-Inconsistent or incorrect signals
	-No signals visible on receiving device
Inconsistent or Incorrect Signals
Inconsistent or incorrect signals often come from noise on the CAN bus. An easy way to check for noise on the CAN bus is to connect an oscilloscope as a CAN device, connecting one probe to the CANH line, and one to the CANL, (see Figure 1). If the oscilloscope shows a clear square wave,(Figure 2), then the issue lies with the receiving device. The following are possible causes:
	-Poor wiring connection between transmitter and transceiver, (a poor connection between transceiver and Teensy can result in inconsistent signals)
	-CAN lines saturated with messages, (if a device on the CAN line is sending too many messages, other messages may not be able to be sent. This can be verified via CAN sniffer, (Figure 1), or by echoing messages received between two microcontrollers)

  
 <img width="842" height="371" alt="Screenshot 2026-10-12 053508" src="https://github.com/user-attachments/assets/44db03fe-6695-48f7-8e2a-d65d4b04f001" />
Figure 1. Oscilloscope and CAN Sniffer Connected to CANH and CANL


<img width="845" height="320" alt="Screenshot 2026-10-12 053516" src="https://github.com/user-attachments/assets/db58cf9d-d4a5-4629-8574-8aed1b6831ad" />
Figure 2. Healthy Heartbeat Signals on an Oscilloscope
If the oscilloscope shows a large amount of noise that exceeds the magnitude of the square wave, the following are possible causes:
	-High electromagnetic interference, (try isolating the wires, or twisting them tighter to see if that reduces noise in the signal)
	-Incorrect terminating resistors, (if the resistance measured from any CANH to CANL does not show 60Ohms, the two 120Ohm resistors are incorrect)
	-Poor ground connections, (verify that the grounds on each CAN bus have no voltage difference between them)
	-Incorrect voltage to CAN bus, (verify that each CAN bus is receiving the specified amount of voltage. See Wiring and Power for details on power)

No Signals Visible on the Receiving Device
When no signals are present on the receiving device, it can be due to either a software or hardware issue. This can usually be determined by using an oscilloscope as shown in Figure 1 to determine if a signal is present on the lines at all. If there is a clear square wave, the issue lies with the software or configurations. If there is no square wave, or a high noise wave, the issue lies with hardware.

Clear Square Wave:
	-Node IDs incorrect, (if a node ID is unconfigured, that device will not send or receive any messages. In addition, if a receiving device is looking for an incorrect node ID, it will not recognize the signal on the CAN lines)
	-Mismatched baudrates, (if the baudrates of the sending and receiving devices are not matching, no messages will be picked up the receiving device)
	-Extended message IDs, (An unlikely, but possible, error is having a device configured to use different message formats than the other devices. The ODrive had a setting for extended message IDs, which changes the way the node IDs are labelled. This is disabled by default, but if messages are present on a CAN sniffer with unusual node IDs, this could be a cause)
	-Complete saturation of CAN messages, (an issue encountered in this project was the Teensy fully saturating the CAN lines with error messages. Something, (likely an incorrect voltage on the transmitting lines), caused the Teensy to collect error messages until it hit the error limit, (usually 127 error messages). The Teensy then saturated the CAN busses with “STATE_ERROR_PASSIVE”, preventing any other message from being transmitted. Clearing the errors solved this issue, and the errors did not regenerate)
	-Faulty receiving device, (another issue encountered in this project was the Teensy transceiver having a poor physical connection with the Teensy. This was verified by measuring the resistance between the Teensy RX pin and the transceiver RX pin. Ideal resistance should be very low, < 0.5Ohms. Signal quality can also be verified by using a single oscilloscope probe on the RX line of the Teensy. It should show a clear square wave if the Teensy is physically receiving the messages from CANH and CANL)

Unclear or Absent Square Wave:
For high noise square waves, if none of the solutions for inconsistent signals work, the issue may lie with the hardware itself. Checking parts one at a time, and swapping faulty parts may narrow down the issue. If there is a complete absence of a wave, no device is sending signals. This is either due to a hardware issue with the CAN bus on a device, or the device is not configured to send any signals, heartbeat or otherwise.


ODrives & Motors
ODrive Configs
This project used 4 Odrive Pros, 4 Maxon ECX Torque XL 22mm motors, and 4 AS5147 magnetic position sensors. These were chosen because the ODrives handle motor encoding and commutation directly. By inputting the motor and encoder specifications into the ODrive website GUI, the ODrive config to control the motors via encoding can be saved to the individual ODrive. The ODrive GUI was also used for convenient calibration, CAN node, and heartbeat configuring. (All of these things can also be done through the ODrive Arduino library, or the ODrive tool). Table 1 shows the values used to configure the motors and encoders in the GUI. These values were collected from the motor datasheets. Any values not listed were left as defaults.

Table 1. ODrive Config Values
<img width="1323" height="673" alt="Screenshot 2026-10-12 053905" src="https://github.com/user-attachments/assets/3602ab20-6743-475d-824f-c56aa2874333" />



Setting CAN Resistors
To set the 120Ohm CAN resistors on the end of the CAN chain, set the switches on the ODrives to the configurations shown in Figure 3:


<img width="1069" height="467" alt="Screenshot 2026-10-12 053528" src="https://github.com/user-attachments/assets/b70a3c8d-59fb-499d-b9c3-fa3d8df576b0" />
Figure 3. CAN Resistor on the ODrive. On, (left), and off, (right)
Only one of the ODrives needs to have their CAN resistor on, the others must all be off. The second CAN resistor is located on the custom PCB. Function of the resistor can be checked by measuring the resistance across CANH and CANL on the ODrive.

Mechanical
OnShape Motor Mount
https://cad.onshape.com/documents/d9b8be1fdd9e187f86208b44/w/7a0e7395a003ebc5d84da4b7/e/fe12342969d46d16eb942eeb?configuration=default&renderMode=0&uiState=68a9b8f30fa7ff08a49046b4

Tendon Routing
Tendon routing is determined by two factors, null space, and keeping tendons in plane with their pulleys. All tendons are in plane with their pulleys in all configurations, except for the splay joint and motor pulleys. The motor mount allows for two tendons to be wrapped around the splay joint clockwise, and two counterclockwise. The direction each tendon was wrapped was determined by where the tendon exits the proximal joint. The direction each tendon was wrapped around the motor pulleys was also chosen to keep the tendons as close to in plane as possible. Because the motor pulley placement is limited by the radius of the motors, low friction bars on bearings were placed to route the tendons around to avoid the tendons slipping off their pulleys. 

The null space is a vector derived from the tendon routing jacobian, and is specific to the pulley radii and tendon routing. The null space gives the bias force of the system, (the forces to apply to each tendon to maintain tension in the tendons with a net 0 torque around each joint). However, when routing the tendons with a single extensor tendon, and two flexor tendons in the proximal, medial, and distal joints, the null space has a negative first term. This would mean applying a negative torque to motor 0, which isn’t an option with tendons. To solve this problem, one of the flexor tendons, tendon 3, was routed the opposite direction around the proximal joint, as shown in Figure 4. This changed the null space to positive terms.


<img width="1035" height="416" alt="Screenshot 2026-10-12 053543" src="https://github.com/user-attachments/assets/71285ebd-137b-4be0-a532-94f688a0f5d5" />
Figure 4. Tendon Routing Diagram, (Excluding Splay and Motors)


Linear Approximation of Joint Coupling
The proximal and medial joints of the dex finger are kinematically coupled with a 4 bar coupling. This leads to a complex relationship between the angles of the joints. This causes issues with the control because it means that the change in joint angles is not directly dependent on the change in motor angles, it is also dependent on the configuration. Figure 5 illustrates the motion of the joints, and shows the tendon routing jacobian for the specific pulley radii. (Positive rotation is always defined as flexion, while splay is defined as positive for counterclockwise rotation when viewing the finger from behind).

<img width="1005" height="521" alt="Screenshot 2026-10-12 053553" src="https://github.com/user-attachments/assets/e2ed7c95-e0af-4e78-91c1-bc1d0b87d3e8" />
Figure 5. Joint Math without Coupling, and Coupling Illustrations
With no coupling, the joint jacobian can be represented as Jt is above. But because the joints are connected, one of the angles must be represented as a function of the other, and the jacobian has to be simplified to a 4x3 matrix instead of a 4x4. If the function were linear, change in one angle could be represented as a scalar of the change of other angle, but 4 bar linkages are not linear. 

Using the Onshape assembly of the Dex finger, the angle of the medial joint was measured for various proximal joint angles. The data points were graphed in excel, and it was observed that for proximal joint angles zero to approximately seventy degrees, there is a very linear relationship between the joint angles, as shown in Figure 6.

<img width="1092" height="493" alt="Screenshot 2026-10-12 053559" src="https://github.com/user-attachments/assets/9fb07645-35a6-402f-a1f7-0f6babcef2cc" />
Figure 6. Proximal and Medial Joint Angles
This linear relationship between the angles was used for all calculations of joint and motor positions. 

Motor Calibration
For the time being, the easiest way to calibrate the motors is to take off the encoder harnesses, remove the pulleys from the motor shafts, and return the encoder harnesses. The motors can then be calibrated through the ODrive GUI. Motors should be recalibrated whenever unexpected behaviour begins to occur, or when a tendon goes slack. The tendons were always tensioned with a minimum motor torque of 5mNm, so slackness indicates the motors are not providing the correct torque.

Wiring and Power

<img width="885" height="729" alt="Screenshot 2026-10-12 053605" src="https://github.com/user-attachments/assets/366128c1-90d0-4888-8cf5-1c6273f4a9b4" />
Figure 7. Wiring Diagram for the n+1 Control System

Control
The control of the finger is composed of a feedforward term and a PID controller, Figure 8.

<img width="1006" height="389" alt="Screenshot 2026-10-12 053611" src="https://github.com/user-attachments/assets/5765092b-f954-42d5-adec-de518ce9e065" />
Figure 8. Control Scheme Block Diagram
Feedforward: 
For the feedforward, the Onshape assembly was used to find the center of mass of each joint relative to the lower axis of rotation of the joint. This included all machined components of the Dex finger, and components in the assembly that had defined materials. Some components, (ball bearings, etc.), that didn’t have defined materials were excluded from the COM calculations with the anticipation of any small errors being dealt with by the integral term of the PID. Using the mass of each segment, and the relative COM, the overall torque due to gravity on each joint was calculated and applied by the following method, (Figure 9).

<img width="907" height="259" alt="Screenshot 2026-10-12 053617" src="https://github.com/user-attachments/assets/5e0ef9f8-a3cf-4ebd-82d7-ca45eab04cce" />

Figure 9. Feedforward Calculation Method
The resulting counter torque from the feedforward is added to the corrective torques generated by the PID controller, and the summed torque is then balanced.

Torque Balancing:
The torques are balanced in two ways. First, a bias force is added to the torques such that the smallest motor torque is 5mNm. This was chosen as the smallest value to overcome motor cogging and actually rotate the motors, but this should be changed after running anticogging calibration. Second, if any of the torques is over 25mNm after adding the bias force, the torques should be scaled down to the range 5mNm to 25mNm. This does result in the net joint torques being different than intended, but the motors retain a potentially dangerous amount of heat at higher than 25mNm. (3-5 minutes of continuous operation at 25mNm results in motor temps of ~50C, high enough to burn skin).

This balancing prevents motors from turning backwards, and prevents the motor drivers from requesting too high torques from the motors.

Code
Github link:
https://github.com/NiamhC3/REU_nplus1

Libraries used:
C++:
<img width="1242" height="129" alt="Screenshot 2026-10-12 053628" src="https://github.com/user-attachments/assets/94eafe5b-d560-4d54-88f2-11e059decef9" />
lib_deps =
    https://github.com/tonton81/FlexCAN_T4
    https://github.com/odriverobotics/ODriveArduino

Python:
Tkinter
Matplotlib

Known Code Errors:
-When running ODrive calibration in code, motor drivers fail to enter closed loop control. To avoid this, calibrations were done outside of code through the GUI.
-Odrives are configured to send feedback every 10ms. Code does not pick up new feedback that frequently. This could be due to a timing issue, or potentially noise on the CAN lines. The temporary solution is to add a microsecond value delay to wait for new feedback.

New errors (Likely hardware or firmware related, restoring a previously working version of the code did not fix these):
-Odrives are now occasionally showing the error “missing input”. This could also be a timing issue, but the error is new. Rerunning the code removes the error, and it doesn’t immediately regenerate. This also occurred a few times when physically moving wires, so it could be the result of a faulty connection.
-Motor offset collection doesn’t always complete correctly. Motors are supposed to move the finger to an upright position, (reference point of 0 for all further angle calculations), but this has also been failing lately. This must complete correctly for further angle calculations to be reasonable.

UI:
The UI syntax was largely generated by ChatGPT from the selected libraries. It is a separate python script that communicates with the teensy via serial communication over a USB. To make it work correctly, ensure that the serial bitrate, and the port the usb is plugged into match across the python script and the c++ program. Because both platformIO and the python script are trying to connect to the same port, ensure that platformIO is disconnected from the port before trying to run the python script

Known Issues and Recommendations 
The PLA of the motor mounts is softening due to the heat of motors. Consider replacing the PLA with something with a higher softening temperature, or adding a heatsink for the motors. The operating temperature of a motor at 25mNm of torque for 5-10 minutes can easily reach 50C.
Even when the ODrives are only configured to send feedback at 10ms over 250k bitrate, the messages are not consistently received. This could be because the software is not configured to receive messages at a set rate, and only checks for feedback at certain points in the loop. 
The CAN bus seems to be saturated with messages with 4 ODrives sending feedback every 10ms. This is supported by the fact that when the feedback frequency is increased, the heartbeats of the ODrives with larger node IDs are not received. Consider splitting the CAN busses of the ODrives across additional busses on the teensy side. Increased frequency of feedback means that the system can update based on new position and velocity data more often.
Also limiting update frequency is serial prints. When the teensy sends messages to the computer for UI or debug purposes, the code is significantly delayed. With the UI prints, (<100 characters total), the frequency of updates drops from 200Hz to 35Hz. This change is more drastic than expected given the high bitrate the serial communication runs at, (115200), so there may be an issue with the buffer filling and the code delaying because of it. Alternatively, the UI communication method could be changed from serial to something faster. 
The linear approximation used for the 4 bar linkage is helpful for the upper section of the control, but is not useful for controlling the bottom ~20 degrees of motion. To achieve full position control for this particular finger, this approximation needs to be replaced with a complete function.
There is an error in the joint angle calculation. The exact cause is unknown, but it could be due to the linkage, or an error in the formula used to calculate the angles. This results in the feedforward term countering gravity being poor. The values in the feedforward were manually tuned to approximate better, but once the error is corrected, the original feedforward should be reevaluated. 
