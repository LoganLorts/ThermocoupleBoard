Current iteration uses an Arduino giga with an arduino mega data shield communicating with the LTC2984.
Links:
https://www.analog.com/en/products/ltc2984.html
https://store-usa.arduino.cc/products/giga-r1-wifi?srsltid=AfmBOopj3Obfn5Uupg23lJG5jRPGHVTn0Cs4W42ruEa2sTFBW0iU7hsE
https://learn.adafruit.com/adafruit-data-logger-shield/overview
https://www.analog.com/en/resources/evaluation-hardware-and-software/evaluation-boards-kits/dc2420a.html#eb-overview (demo board)

To Do:

Software:
All software currently made for demo board, once the new LTC2984 (thermocouple board) is completed, adapt code.

harmonic boundry heating:
test heating accuracy
test acquisition rate
test timing of loop
parallelize processes
make LTC get measurements while the rest of the loop runs

ATMC LTC2984:
test heating accuracy
test acquisition rate
test timing of loop
parallelize processes
make LTC get measurements while the rest of the loop runs



Hardware:

Thermocouple board:
Test functionality and accuracy
solder termblocks

Heating Board:
order and solder logic inverters
order and solder optoisolaters
order and solder heating termblocks
testing

giga hat:
(if needed) Once thermocouple and heating board are tested, make giga hat for SD card reader, mosfet drives, thermocouple SPI connections, humidity sensor, and pressure sensor

Demo board:
Order and solder 2 of the remaining heating termblocks and mosfet drivers (currently one working)
3d print case
rewire to clean setup around the case
(optional) Add barrel jack connector for power



Notes:

Demo board notes:
mosfet driver pin: pin 8
LTC chip select pin: pin 7
SD card reader chip select: pin 10
Power from DC power supply up to 15V
serial speed: 115200
Cold junction sensor avaliable on channel 1 and channel 2 (only need one)
Differential thermocouple readings, select higher channel with differential setting; the postive lead goes to the higher channel and negative lead to one channel lower.
Intented to run up to 9 thermocouples with 3 mosfet drivers runnning the heating element.
Real time clock is I2C on wire 1 (address 0x68)
cold junction sensor is an npn BJT shorted to function as a diode, the ideality factor can be changed to reflect this, but default works.
<img width="734" height="500" alt="DC2420A-1" src="https://github.com/user-attachments/assets/6bc8a5da-e7d2-48fa-ab4a-0628b49456e6" />
SPI connections are shown here, ribbon cable connector is not used with the giga. 


Heating board notes:
rated 15V 10 Amps (realistically no more than 5 amps with 3 drivers on)
Do not common ground with the giga

Thermocouple board notes:
Cold junction sensor avaliable on channel 20 and channel 9. Use channel 20 to normalize channels 14-19, channel 9 for the rest.
Intended to run 18 thermocouples, single ended mode (thermocouples go channel to ground)
cold junction sensor is an npn BJT shorted to function as a diode, the ideality factor can be changed to reflect this, but default works.
