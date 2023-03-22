# ugv

Source code for the UGV by UAVionics techs!
This project is ongoing and maintained for educational purposes! Stay tuned for additional features and improvements!

## Directory

Arduino -> Arduino .ino and library files for the low level controller

Example files -> Sample code provided for our day 1 workshop

workspace -> Linux ros workspace files for the high level controller




more documentation is on the way!




### LDS extension:

#### An LDS was added in continuation of this project! Here are the connection details as follows:

XIAOMI 1S LDS pinouts

- MOT+

- GND

- MOT-

- TX

- VCC (+5V)

Motor powered by 3.3v, flyback diode between motor terminals.
Low side driver transistor collector on MOT-, base to PWM output (32768hz 10bit res.), emitter to GND
10k ohm resistor between LDS TX and ESP RX

- MOT+ to 3.3V

- GND to GND

- MOT- to base of Low side driver transistor

- TX to 10k ohm resistor, to any designated RX pin

- VCC to 5V


transistor used: 2N3904

flyback diode used: BAT85
>>>>>>> 00dad3c35e3e19b6317d79662334deb289178a02
