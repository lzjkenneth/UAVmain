# ugv

Source code for the UGV by UAVionics techs!
This project is ongoing and maintained for educational purposes! Stay tuned for additional features and improvements!

## Directory



| Directory | Description |
| --- | --- |
| Arduino | Arduino library and .ino files for the low level controller |
| Example files | Sample code files provided in the workshop |
| Workspace | Linux ROS workspace files for the high level controller |

more documentation is on the way!





## LDS extension:
A Laser Distance Sensor (LDS) was added in continuation of this project! Here are the details as follows:
<details>
    <summary>Click me</summary>
  
  ### Physical Connections:
  
  XIAOMI 1S LDS pinout connections

  | Pin no. | on LDS | connected to |
  | --- | --- | --- |
  | 1 | MOT+ | 3.3V |
  | 2 | GND | GND |
  | 3 | MOT- | base of low side driver transistor |
  | 4 | TX | 5V -> 3.3V level shifter, to any designated RX pin |
  | 5 | VCC | 5V |

  

  Motor powered by 3.3v, flyback diode (BAT85) between motor terminals.  
  Low side driver transistor (2N3904) collector on MOT-, base to PWM output (32768hz 10bit res.), emitter to GND.  
  level shifter (BSS138) between LDS TX (5V) and ESP RX (3.3V)
  

  ### Software:
  
  LDS interfacing code was referenced and edited from https://github.com/getSurreal/XV_Lidar_Controller

</details>

