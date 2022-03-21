# PI4_RF-LoRa-868

## Interface of PI4 with RF-LORA-868
The RF-LORA-868 is an easily available SX1272 LoRa implementation and readily connected via
the PI4 GPIO connector. The PI4 SPI driver provides the active link with two additional pins
for Reset and TX/RX control. This particular application is as a hub for my LoRaLan implementation.

## Design
The code is mostly derived from Heltec Aduino inteface for the SX1276. There are minor config register 
differences between SX1272 and SX1276 hence the #define SX1272. Of note is that the SX1272 minimum
signal bandwith is 125Khz. The SX1276 chips must be set to the same bandwidth.

My inteface difers from the Heltec implementation in using a non-blocking continuous receive
implementation.

## Build
Make files are provided to build the LoRa + SPI library and the LoRaReceiver and LoRaSender
demonstrations. If the inital register dump gives you zeros you have a problem with the SPI
connection.

## Connections
GPIO		RF-LORA-68	Function
10		15		MOSI
09		14		MISO
11		13		SCCK
08		16		/CEO
23		4		TX/RX
24		12		Reset
Pin 17	5 + 3		Vcc (3v3)
Pin 20	2		Gnd

ChiefEngineer
		