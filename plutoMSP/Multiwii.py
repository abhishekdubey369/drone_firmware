#Multiwii protocol

'''
SERIAL2_PROTOCOL = 33 is for DJI FPV or RE Goggles

SERIAL2_PROTOCOL = 32 is for sensors or generic MSP telemetry usage

SERIAL2_PROTOCOL = 42 is for DisplayPort for DJI goggle using wtf-os firmare, HDZero, Walksnail, or DisplayPort MWOSD.

'''

'''
When SERIAL2_PROTOCOL = 33 is selected the protocol decoder can work in polling mode (default) or in “telemetry push” mode. When working in polling mode, both TX and RX must be connected to the MSP telemetry transceiver. While in push mode only the TX line is used. To enable push mode, simply set MSP_OPTIONS bit 0 to “1”; to disable set it to “0” (default).

MSP sensors such as the Matek 3901-L0X are supported by both protocols.

'''

'''

Introduction
This section details on how you could build a remote control for Pluto.
Connecting to Pluto
In the default mode, a wifi hotspot called Pluto_xxxx_xxxx will come up. After connecting to this hotspot, you will be assigned an IP address as: 192.168.4.x. The server is hosted at 192.168.4.1:23. This is a TCP/IP based connection which uses telnet protocol. To access this server, you may use Putty client. After connection is successful. You may use the AT commands to check if you have connected to the drone. Please check this document to see the list of AT commands. After connecting to the server you will need to understand the drone protocols to control the drone.
Packet Structure
Each packet follows the following structure:

Header
Direction
Msg Length
Command
Message Data
Checksum
(2 Bytes)
(1 Byte)
(1 Byte)
(1 Byte)
(N Bytes)
(1 Byte)

Details of packet structure

Type of Byte
ASCII
Hex
Description
Header
"$M"
0x24 0x4d
All MSP messages have this header
Direction
"<" or ">"
0x3c or 0x3e
"<" If the message is to the flight controller (IN)
">" If the message is from the flight controller (OUT)
Msg length

0x00 to 0xff

Command

0x01 to 0xff
This byte tells us about the type of packet. For example attitude packet or RC packet.
You can find more packets in the table below.
Payload


This is the main data of the packet. 
CRC


This is a checksum for ensuring the integrity of the packet.
This is should be verified at the receiving end.
CRC is obtained by XORing all the bytes from "Msg length", "Command" and "Payload".

Basic Commands
You can start making the remote with a single packet "MSP_SET_RAW_RC". As your requirements increase, you may add more packets.
IN Packets
MSP_SET_RAW_RC
Hex : 0xc8
Direction : IN ("<")
Length : 16 bytes
Description : 

Data
Data type
Range
Description
Roll Stick
UINT16
900-2100
1500 = 0 degrees
Pitch Stick
UINT16
900-2100
1500 = 0 degrees
Throttle Stick
UINT16
900-2100
1500 = 0 vertical velocity. (Alt hold mode)
Yaw Stick
UINT16
900-2100
1500 = 0 yaw rate. (Mag hold )
AUX1 (Yaw modes)
UINT16
900-2100
900 < AUX1 < 1300 : MagHold On
1300 < AUX1 < 1700 : HeadFree On
AUX2 (Dev mode)
UINT16
900-2100
If AUX2 = 1500, the User code runs. 
(Developer mode on)
AUX3
UINT16
900-2100
1300 < AUX3 < 1700 : Alt Hold mode On
Outside the above range : Throttle mode On
AUX4
UINT16
900-2100
1300 < AUX4 < 1700 : "ARM" the drone
Outside the above range : "DISARM" the drone
ARMING is necessary to control power to the motors


MSP_SET_COMMAND
Hex : 0xd9
Direction : IN ("<")
Length : 2 bytes
Description : 

Data 
Data type
Range
Description
Current Command
UINT16
1-6
1 : Take-off
2 : Land
3 : Back flip
4 : Front flip
5 : Right flip
6 : Left 

OUT Packets
The following are "OUT" packets. These packets are received only upon request is sent. A request packet for a desired "OUT" packet is created by creating and sending an "IN" packet with the same command and without any payload.
   
MSP_ATTITUDE
Hex : 0x6c
Direction : OUT (">")
Length : 6 bytes
Description : 


Data
Data type​
Range
Units
Description
Roll
INT16
-1800 - 1800
Deci degrees

Pitch
INT16
-900 - 900
Deci degrees

Yaw
INT16
0 - 360
Degrees


MSP_ALTITUDE
Hex : 0x6d
Direction : OUT (">")
Length : 6 bytes
Description : 

Data
Data type​
Units
Description
Altitude
INT32
cm

Vario
INT16
cm/s


MSP_RAW_IMU
Hex : 0x66
Direction : OUT (">")
Length : 18 bytes
Description : 
Data
Data type​
Units
Description
Acc X
INT16


Acc Y
INT16


Acc Z
INT16


Gyro X
INT16


Gyro Y
INT16


Gyro Z
INT16


Mag X
INT16


Mag Y
INT16


Mag Z
INT16


More Commands
There are many more commands you have access to. To get a full list of all the commands you may look at the the specific source code file. All the files are available on GitHub.



'''