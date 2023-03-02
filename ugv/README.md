# ugv

Source code for the completed UGV by UAVionics techs!

Arduino -> Arduino .ino and library files for the low level controller

Example files -> Sample code provided for our day 1 workshop

workspace -> Linux ros workspace files for the high level controller




more documentation is on the way!




LDS connections:



XIAOMI 1S LDS pinouts

MOT+

GND

MOT-

TX

VCC (+5V)

Motor powered by 3.3v, flyback diode between motor terminals.
Low side driver transistor collector on MOT-, base to PWM output (32768hz 10bit res.), emitter to GND
10k ohm resistor between LDS TX and ESP RX

MOT+ to 3.3V

GND to GND

MOT- to base of Low side driver transistor

TX to 10k ohm resistor, to any designated RX pin

VCC to 5V


transistor used: 2N3904

flyback diode used: BAT85
