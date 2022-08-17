# About the project

MulticopterFCS is a flight controller build from the ground up. It is meant to be a development platform mainly focused on autonomous indoor flying of multi-rotor drones. 

|Diagram|Board|
|---|---|
|![DrawIO block diagrams-Pagina-5 drawio (3)](https://user-images.githubusercontent.com/99826862/185192702-edf96999-37d7-4a41-971a-ddd6ecabeabc.png)|![WhatsApp Image 2022-08-17 at 5 33 48 PM](https://user-images.githubusercontent.com/99826862/185181215-c10b3f6e-3cc6-4cf2-8778-ef5579de345e.jpeg)|

# Key features

**On board:**
* High performance MCU (STM32H743 running up to 480 MHz)
* Accelerometer and gyro (MPU6000)
* Magnetometer (HMC5883L)
* Pressure and temperature sensor (BMP388)
* 64-Kbit of F-RAM (FM24C64B)
* 2x RGB LED

**Connectors:**
| **quantity** 	| **purpose**  	| **connector type**           	| **Comment**                                                           |
|----------	|----------	|--------------------------	|----------------------------------------------------------------------	|
| 1        	| SWD/JTAG 	| 1.27 mm pitch 10-pin     	| For programming and debugging                                        	|
| 1        	| SDIO     	| Micro SD Card            	| For logging data during the flight                                   	|
| 1        	| I²C      	| JST SH 4-pin             	| External I²C connector                                               	|
| 2        	| UART     	| JST SH 4-pin             	| External UART connector                                              	|
| 1        	| Battery  	| JST SH 2-pin             	| Used to measure battery voltage                                      	|
| 1        	| LED	  	| JST SH 8-pin             	| To control external LEDs                                      	|
| 1        	| IBUS     	| 2.54 mm pitch header     	| IBUS used by external receivers to communicate remote control signals |
| 6        	| PWM      	| 2.54 mm pitch header     	| PWM is used to control external ESC's                                	|
| 1        	| Buzzer   	| 2.54 mm pitch header     	| For connecting a buzzer                                              	|
| 1        	| RPLiDAR  	| 2.50 mm MOLEX SPOX 7-pin 	| This connector will be used for the RPLiDAR                          	|
| 2        	| USB      	| USB Mini B               	| USB1 goes directly to the MCU, USB2 gets converted to serial first   	|

# Roadmap:

1. ✔️ Project orientation
2. ✔️ Deciding on components
3. ✔️ Creating schematic
4. ✔️ Routing PCB
5. ✔️ Manufacturing the PCB
6.  Testing
	1. ✔️ Programming MCU
	2. ❌ Gyro/Accelerometer
		1.  Find a solution to the Gyro/Accelerometer not working
	3. Magnetometer
	4. Barometric pressure sensor
	5. On board F-RAM
	6. ESC control
	7. IBUS communication
	8. Buzzer and LEDs
7. Writing drivers
	1. Gyro/Accelerometer
	2. Magnetometer
	3. Barometric pressure sensor
	4. On board F-RAM
	5. ESC control
	6. IBUS communication
	7. Buzzer and LEDs
	8. SD Card
	9. USB
8. Signal processing
	1. Data conversion and correction
	2. Data filtering
	3. Extended Kalman filter implementation for sensor fusion
	4. PI(D) controller
9. FSM controller 
10. Data logging on the SD Card

Future goals on the roadmap are likely to be Simultaneous Localization and Mapping using a LiDAR implementation. Based on this, autonomous indoor flight could be developed.

# Licensing

**This system is still a work in progress, use anything inside this repository at your own risk.**

This an open source project released under the BSD 3-Clause License.
