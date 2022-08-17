# About the project

**This system is still a work in progress, use anything inside this repository at your own risk.**

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
* SWD/JTAG for programming
* Micro SD Card slot
* I²C
* 2x UART
* 2x USB Mini B 
* IBUS header for external receiver
* 6x ESC header
* Buzzer header
* RPLiDAR A1M8
* External LEDs
* Battery (4s LiPo max)

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

This an open source project released under the BSD 3-Clause License.
