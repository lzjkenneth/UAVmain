// start of XV11 lidar controller code, credits to getsurreal

#include <PID.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <SerialCommand.h>

const int N_ANGLES = 360;              // # of angles (0..359)
const int SHOW_ALL_ANGLES = N_ANGLES;  // value means 'display all angle data, 0..359'

struct EEPROM_Config {
  byte id;
  char version[6];
  int motor_pwm_pin;    // pin connected to mosfet for motor speed control
  double rpm_setpoint;  // desired RPM (uses double to be compatible with PID library)
  double rpm_min;
  double rpm_max;
  double pwm_max;   // max analog value.  probably never needs to change from 1023
  double pwm_min;   // min analog pulse value to spin the motor
  int sample_time;  // how often to calculate the PID values

  // PID tuning values
  double Kp;
  double Ki;
  double Kd;

  boolean motor_enable;         // to spin the laser or not.  No data when not spinning
  boolean raw_data;             // to retransmit the seiral data to the USB port
  boolean show_dist;            // controlled by ShowDist and HideDist commands
  boolean show_rpm;             // controlled by ShowRPM and HideRPM commands
  boolean show_interval;        // true = show time interval, once per revolution, at angle=0
  boolean show_errors;          // Show CRC, signal strength and invalid data errors
  boolean aryAngles[N_ANGLES];  // array of angles to display
} xv_config;

const byte EEPROM_ID = 0x07;  // used to validate EEPROM initialized

double pwm_val_lds = 1023;  // start with ~100% power
double pwm_last;
unsigned long now;
unsigned long motor_check_timer = millis();
unsigned long motor_check_interval = 200;
double motor_rpm;
unsigned int rpm_err_thresh = 10;  // 2 seconds (10 * 200ms) to shutdown motor with improper RPM and high voltage
unsigned int rpm_err = 0;
unsigned long curMillis;
unsigned long lastMillislds = millis();

const unsigned char COMMAND = 0xFA;  // Start of new packet
const int INDEX_LO = 0xA0;           // lowest index value
const int INDEX_HI = 0xF9;           // highest index value

const int N_DATA_QUADS = 4;         // there are 4 groups of data elements
const int N_ELEMENTS_PER_QUAD = 4;  // viz., 0=distance LSB; 1=distance MSB; 2=sig LSB; 3=sig MSB

// Offsets to bytes within 'Packet'
const int OFFSET_TO_START = 0;
const int OFFSET_TO_INDEX = OFFSET_TO_START + 1;
const int OFFSET_TO_SPEED_LSB = OFFSET_TO_INDEX + 1;
const int OFFSET_TO_SPEED_MSB = OFFSET_TO_SPEED_LSB + 1;
const int OFFSET_TO_4_DATA_READINGS = OFFSET_TO_SPEED_MSB + 1;
const int OFFSET_TO_CRC_L = OFFSET_TO_4_DATA_READINGS + (N_DATA_QUADS * N_ELEMENTS_PER_QUAD);
const int OFFSET_TO_CRC_M = OFFSET_TO_CRC_L + 1;
const int PACKET_LENGTH = OFFSET_TO_CRC_M + 1;  // length of a complete packet
// Offsets to the (4) elements of each of the (4) data quads
const int OFFSET_DATA_DISTANCE_LSB = 0;
const int OFFSET_DATA_DISTANCE_MSB = OFFSET_DATA_DISTANCE_LSB + 1;
const int OFFSET_DATA_SIGNAL_LSB = OFFSET_DATA_DISTANCE_MSB + 1;
const int OFFSET_DATA_SIGNAL_MSB = OFFSET_DATA_SIGNAL_LSB + 1;

int Packet[PACKET_LENGTH];  // an input packet
int ixPacket = 0;           // index into 'Packet' array
const int VALID_PACKET = 0;
const int INVALID_PACKET = VALID_PACKET + 1;
const byte INVALID_DATA_FLAG = (1 << 7);  // Mask for byte 1 of each data quad "Invalid data"



/* REF: https://github.com/Xevel/NXV11/wiki
  The bit 7 of byte 1 seems to indicate that the distance could not be calculated.
  It's interesting to see that when this bit is set, the second byte is always 80, and the values of the first byte seem to be
  only 02, 03, 21, 25, 35 or 50... When it's 21, then the whole block is 21 80 XX XX, but for all the other values it's the
  data block is YY 80 00 00 maybe it's a code to say what type of error ? (35 is preponderant, 21 seems to be when the beam is
  interrupted by the supports of the cover) .
*/
const byte STRENGTH_WARNING_FLAG = (1 << 6);  // Mask for byte 1 of each data quat "Strength Warning"
/*
  The bit 6 of byte 1 is a warning when the reported strength is greatly inferior to what is expected at this distance.
  This may happen when the material has a low reflectance (black material...), or when the dot does not have the expected
  size or shape (porous material, transparent fabric, grid, edge of an object...), or maybe when there are parasitic
  reflections (glass... ).
*/
const byte BAD_DATA_MASK = (INVALID_DATA_FLAG | STRENGTH_WARNING_FLAG);

const byte eState_Find_COMMAND = 0;                        // 1st state: find 0xFA (COMMAND) in input stream
const byte eState_Build_Packet = eState_Find_COMMAND + 1;  // 2nd state: build the packet
int eState = eState_Find_COMMAND;
PID rpmPID(&motor_rpm, &pwm_val_lds, &xv_config.rpm_setpoint, xv_config.Kp, xv_config.Ki, xv_config.Kd, DIRECT);

uint8_t inByte = 0;  // incoming serial byte
uint8_t motor_rph_high_byte = 0;
uint8_t motor_rph_low_byte = 0;
uint16_t aryDist[N_DATA_QUADS] = { 0, 0, 0, 0 };  // thre are (4) distances, one for each data quad
// so the maximum distance is 16383 mm (0x3FFF)
uint16_t aryQuality[N_DATA_QUADS] = { 0, 0, 0, 0 };  // same with 'quality'
uint16_t motor_rph = 0;
uint16_t startingAngle = 0;  // the first scan angle (of group of 4, based on 'index'), in degrees (0..359)

SerialCommand sCmd;

boolean ledState = LOW;

#if defined(__AVR_ATmega32U4__) && defined(CORE_TEENSY)  // if Teensy 2.0
const int ledPin = 11;

#elif defined(__AVR_ATmega32U4__)  // if Leonardo (no LED for Pro Micro)
const int ledPin = 13;

#elif defined(__MK20DX256__)  // if Teensy 3.1
const int ledPin = 13;

#elif defined(__MKL26Z64__)  // if Teensy LC
const int ledPin = 13;

#elif defined(ESP32)
const int ledPin = 15;
#endif





// setting PWM properties
const int freq = 32768;
const int ledChannel = 0;
const int resolution = 10;
















// data arrays to publish in scan message
float range[360];
float intensity[360];

int lastAngle = 0;
int currentAngle = 0;  // current angle per lds packet data, 0-359







// start of base controller code

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>  //http://librarymanager/All#SparkFun_VL53L5CX
#include <MPU9250_asukiaaa.h>

#include <ros.h>
#include <tf/tf.h>  // for tf::createQuaternionFromYaw()
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>



//initializing all the variables


// Defining pins

// I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

// Encoder pins
#define encodPinA 18  // encoder A pin
#define encodPinB 19  // encoder B pin

// Motor controller pins
#define APWM 32
#define MA2 33
#define MA1 25
#define STANDBY 26
#define MB1 27
#define MB2 14
#define BPWM 12


// Creating sensor objects

//SparkFun_VL53L5CX myImager;   // Creating Lidar sensor object

// configure definitions as such in vl53l5cx libraries/SparkFun_VL53L5CX_Arduino_Library-main/src/platform.h file, lines 113 to 121
// #define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
// #define VL53L5CX_DISABLE_NB_SPADS_ENABLED
// // #define VL53L5CX_DISABLE_NB_TARGET_DETECTED
// #define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
// #define VL53L5CX_DISABLE_RANGE_SIGMA_MM
// // #define VL53L5CX_DISABLE_DISTANCE_MM
// #define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
// // #define VL53L5CX_DISABLE_TARGET_STATUS
// #define VL53L5CX_DISABLE_MOTION_INDICATOR


//VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
//
//int imageResolution = 0;      //Used to pretty print output
//int imageWidth = 0;           //Used to pretty print output
//
//float range[8] = {0, 0, 0, 0, 0, 0, 0, 0};
//float range_target_status[8] = {0, 0, 0, 0, 0, 0, 0, 0};


// IMU variables
MPU9250_asukiaaa mySensor;            // Creating IMU object
float accelX, accelY, accelZ, aSqrt;  // accelerometer variables
float gyroX, gyroY, gyroZ;            // gyroscope variables
float mX, mY, mZ, mDirection;         // magnetometer variables

float roll, pitch, yaw;  // rotation axis variables

//Sensor calibration iterations
int num_calibration_itrs = 1000;

//Gyroscope sensor offsets
int gyroCalibrated = 0;
float gyroXoffset = 0.00;  // default: 0.08
float gyroYoffset = 0.00;  // default: 1.70
float gyroZoffset = 0.00;  // default: 1.25
float gyroXscale = 1.15;
float gyroYscale = 1.15;
float gyroZscale = 1.15;

//Accelerometer sensor offsets
int accCalibrated = 0;
float accelXoffset = 0.00;
float accelYoffset = 0.00;
float accelZoffset = 0.00;
float accelXscale = 1;
float accelYscale = 1;
float accelZscale = 1;

//Magnetometer sensor offsets
float magXoffset = 0.00;  // default: -50
float magYoffset = 0.00;  // default: -55
float magZoffset = 0.00;  // default: -10

uint8_t sensorId;  // sensor state variable
int result;        // sensor output variable




// Timer variables, for timed function execution
unsigned long lastTime = 0;
unsigned long lastMilli = 0;
unsigned long PIDlastMilli = 0;
unsigned long LDSlastMilli = 0;

#define pubFreq 20;
int LOOPTIME = 1000 / pubFreq;    // Publisher Looptime in milliseconds (50ms)
#define pidFreq 50;
int PIDLOOPTIME = 1000 / pidFreq; // PID Looptime in milliseconds (20ms)

hw_timer_t *My_timer = NULL;





//--- Robot-specific constants ---

#define wheel_diameter 0.065  // Wheel diameter, in m
#define base_diameter 0.135   // Base diameter or wheel separation, in m
#define base_radius 0.0675    // Base radius, in m
#define gear_ratio 1          // Gear ratio between encoder and motor output

#define encoder_cpr 40  // Encoder ticks or counts per rotation
#define MAX_RPM 333     // Max rotations per minute
#define max_speed 0.5   // Max speed in m/s

#define pi 3.1415926
#define two_pi 6.2831853


// Differential drive working variables and constants

// definition of motor directions to integers

#define FORWARD 1
#define BACKWARD 2
#define RELEASE 3

// register motor direction for counter
int IRAM_ATTR direction1 = FORWARD;
int IRAM_ATTR direction2 = FORWARD;

volatile long IRAM_ATTR posA = 0;  // encoder A counter
volatile long IRAM_ATTR posB = 0;  // encoder B counter

long countAnt1 = 0;     // posA in last getMotorData callback
long countAnt2 = 0;     // posB in last getMotorData callback
long countAnt1Pub = 0;  // posA in last getMotorDataPub callback
long countAnt2Pub = 0;  // posB in last getMotorDataPub callback

// PID constants
int period = 20;            // PID period in milliseconds
double kt = 1000 / period;  // number of periods/sec
float Kp = 0.1;
float Kd = 0;
float Ki = 0;

// PID control variable for motor speed
int PWM_val1 = 0;
int PWM_val2 = 0;








// Wifi and Rosserial variables
const char *ssid = "KntpAP";        // ssid of your network
const char *password = "tase3281";  // password of your network

int status = WL_IDLE_STATUS;


// Set the rosserial socket server IP address
IPAddress server(192, 168, 167, 199);  // ip of your ROS server
IPAddress ip_address;

// Set the rosserial socket server port
const uint16_t serverPort = 11411;  // tcp socket port


// ROS nodes //
ros::NodeHandle nh;  // edited in ros.h to be NodeHandle_<ArduinoHardware, 10, 10, 2048, 2048> NodeHandle; // default 25, 25, 512, 512

// ROS message types

nav_msgs::Odometry odom_msg;       // Odometry message
geometry_msgs::Twist cmd_msg;      // cmd_vel message velocity
sensor_msgs::Imu imu_msg;          // imu message
sensor_msgs::LaserScan lidar_msg;  // LaserScan message



// ROS variables
// required rpm received from handle_cmd callback
double rpm_req1 = 0;
double rpm_req2 = 0;

// actual rpm received from getMotorData function
double rpm_act1 = 0;
double rpm_act2 = 0;

// actual speed in m/s received from getMotorData function
double speed_act1 = 0;
double speed_act2 = 0;

// actual rpm received from getMotorDataPub function
double rpm_act1Pub = 0;
double rpm_act2Pub = 0;

// actual speed in m/s received from getMotorDataPub function
double speed_act1Pub = 0;
double speed_act2Pub = 0;


// ros parameters
double rate = 10.0;  // ros spin rate, used in ros::Rate r(rate); (not implemented)

// scaling speed messages
double linear_scale_positive = 1;
double linear_scale_negative = 1;
double angular_scale_positive = 1;
double angular_scale_negative = 1;

// odometry variables for coordinate positioning and orientation
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
double theta_odom = 0.0;
double theta_gyro = 0.0;

// transform variables for coordinate positioning and orientation
double dt = 0.0;
double dx = 0.0;
double dy = 0.0;
double dth = 0.0;
double dth_odom = 0.0;
double dth_gyro = 0.0;
double dxy = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

// frame_ids
char base_link[] = "base_link"; // unused, frame linked using static_transform_publisher in ros
char odom[] = "odom";
char laser[] = "laser";
char imu_link[] = "imu_link";

// ROS subscriber callback
void handle_cmd(const geometry_msgs::Twist &cmd_msg) {
  double linear_velocity = cmd_msg.linear.x;    // in m/s
  double angular_velocity = cmd_msg.angular.z;  // in rad/s

  // (linear_velocity * 60)                   -> convert rad/s to rad/min to m/min
  // / (pi * wheel_diameter)                  -> convert m/min to rotations/min
  double linear_rpm = linear_velocity * 60 / (pi * wheel_diameter);  // convert m/s to rpm

  // (angular_velocity * 60 * base_radius)    -> convert rad/s to rad/min to m/min
  // / (pi * wheel_diameter)                  -> convert m/min to rotations/min
  double angular_rpm = angular_velocity * 60 * base_diameter / (2 * pi * wheel_diameter);  // convert rad/s to rpm

  rpm_req1 = linear_rpm - angular_rpm;
  rpm_req2 = linear_rpm + angular_rpm;
}

// ROS publishers and subscribers
ros::Publisher odom_pub("odom", &odom_msg);
ros::Publisher lidar("scan", &lidar_msg);
ros::Publisher imu("imu", &imu_msg);
ros::Subscriber<geometry_msgs::Twist> cmd("cmd_vel", handle_cmd);

// ROS timesetting
ros::Time current_time = nh.now();
ros::Time last_time = current_time;

void setupNodeHandler() {
  // Set the connection to rosserial socket server

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Start to be polite
  nh.advertise(imu);
  nh.advertise(lidar);
  nh.advertise(odom_pub);

  nh.subscribe(cmd);
}







void setupWiFi() {
  // Connect to provided network and print ip of device

  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to ");
  Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if (i == 21) {
    Serial.print("Could not connect to ");
    Serial.println(ssid);
    while (1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}


void setupMotors() {
  // Setup motor pins and initialize motors

  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(APWM, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);
  pinMode(BPWM, OUTPUT);
  pinMode(STANDBY, OUTPUT);
  digitalWrite(STANDBY, HIGH);  // HIGH if controlling motor


  motor1setSpeed(0);
  motor2setSpeed(0);
  motor1run(FORWARD);
  motor1run(RELEASE);
  motor2run(FORWARD);
  motor2run(RELEASE);
}


void setupSensors() {
  // setup I2C pins and initialize lidar and imu sensors

  Wire.begin(SDA_PIN, SCL_PIN);  //This resets to 100kHz I2C
  Wire.setClock(400000);         //Sensor has max I2C freq of 400kHz

  //  Serial.println("Initializing VL53L5cx. This can take up to 10s. Please wait.");
  //  if (myImager.begin() == false)
  //  {
  //    Serial.println(F("VL53L5cx not found - check your wiring. Freezing"));
  //    while (1) ;
  //  }
  //  Serial.println(F("VL53L5cx found!"));
  //  myImager.setResolution(8 * 8); //Enable all 64 pads
  //  myImager.setRangingFrequency(10);
  //  myImager.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
  //  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  //  imageWidth = sqrt(imageResolution); //Calculate printing width
  //  myImager.startRanging();

  mySensor.setWire(&Wire);
  Serial.println("Initializing gyroY-91. This can take up to 10s. Please wait.");
  if (mySensor.readId(&sensorId) != 0) {
    Serial.println(F("gyroY-91 not found - check your wiring. Freezing"));
    while (1)
      ;
  }
  Serial.println(F("gyroY-91 found!"));
  mySensor.beginAccel(ACC_FULL_SCALE_2_G);
  mySensor.beginGyro(GYRO_FULL_SCALE_250_DPS);
  mySensor.beginMag();

  Serial.println("Calibrating sensors. This can take up to 10s. Please wait.");
  calibrateGyro();
  calibrateAccel();
  Serial.println(F("Sensors calibrated!"));

  // You can calibrate your own offset for mag values
  // mySensor.magXOffset = magXoffset;
  // mySensor.magYOffset = magYoffset;
  // mySensor.magZOffset = magZoffset;
}




// Calibrate gyroscope
void calibrateGyro() {  // Halt motion whilst calibrating
  gyroX = 0;            // erasing existing gyro data
  gyroY = 0;
  gyroZ = 0;

  // Perform multiple readings for calibration
  for (int count = 0; count < num_calibration_itrs; count++) {
    //Poll gyroY-91 for new data
    if (mySensor.gyroUpdate() == 0) {
      gyroX = mySensor.gyroX();
      gyroY = mySensor.gyroY();
      gyroZ = mySensor.gyroZ();
      Serial.println("gyroX: " + String(gyroX) + "gyroY: " + String(gyroY) + "gyroZ: " + String(gyroZ));
    } else {
      Serial.println("Cannot read gyro values " + String(result));
      return;
    }

    gyroXoffset += gyroX;
    gyroYoffset += gyroY;
    gyroZoffset += gyroZ;
  }

  // Compute offsets
  gyroXoffset /= num_calibration_itrs;
  gyroYoffset /= num_calibration_itrs;
  gyroZoffset /= num_calibration_itrs;

  // Print calibrated offsets
  Serial.println("gyroXoffset: " + String(gyroXoffset) + "gyroYoffset: " + String(gyroYoffset) + "gyroZoffset: " + String(gyroZoffset));
  gyroCalibrated = 1;
}

// Calibrate accelerometer
void calibrateAccel() {  // Halt motion whilst calibrating
  accelX = 0;            // erasing existing gyro data
  accelY = 0;
  accelZ = 0;

  // Perform multiple readings for calibration
  for (int count = 0; count < num_calibration_itrs; count++) {
    //Poll gyroY-91 for new data
    if (mySensor.accelUpdate() == 0) {
      accelX = mySensor.accelX();
      accelY = mySensor.accelY();
      accelZ = mySensor.accelZ();
      Serial.println("accelX: " + String(accelX) + "accelY: " + String(accelY) + "accelZ: " + String(accelZ));
    } else {
      Serial.println("Cannot read accel values " + String(result));
      return;
    }

    accelXoffset += accelX;
    accelYoffset += accelY;
    accelZoffset += accelZ;
  }

  // Compute offsets
  accelXoffset /= num_calibration_itrs;
  accelYoffset /= num_calibration_itrs;
  accelZoffset /= num_calibration_itrs;
  accelZoffset -= 1;  // Remove gravity from the z-axis

  // Print calibrated offsets
  Serial.println("accelXoffset: " + String(accelXoffset) + "accelYoffset: " + String(accelYoffset) + "accelZoffset: " + String(accelZoffset));
  accCalibrated = 1;
}






// Get gyroscope readings
void getGyroReadings() {
  gyroX = 0;  // clear data
  gyroY = 0;
  gyroZ = 0;

  //Poll gyroY-91 for new data
  if (mySensor.gyroUpdate() == 0) {
    gyroX = mySensor.gyroX() - gyroXoffset;
    gyroY = mySensor.gyroY() - gyroYoffset;
    gyroZ = mySensor.gyroZ() - gyroZoffset;

    // invert values and convert from deg/s to rad/s
    gyroX *= pi / -180;
    gyroY *= pi / -180;
    gyroZ *= pi / -180;

    // scale values
    gyroX *= gyroXscale;
    gyroY *= gyroYscale;
    gyroZ *= gyroZscale;
    //    Serial.println("gyroX: " + String(gyroX) + "gyroY: " + String(gyroY) + "gyroZ: " + String(gyroZ));

  } else {
    Serial.println("Cannot read gyro values " + String(result));
  }
}

// Get accelerometer readings
void getAccReadings() {
  accelX = 0;  // clear data
  accelY = 0;
  accelZ = 0;

  //Poll gyroY-91 for new data
  if (mySensor.accelUpdate() == 0) {
    accelX = mySensor.accelX() - accelXoffset;
    accelY = mySensor.accelY() - accelYoffset;
    accelZ = mySensor.accelZ() - accelZoffset;

    // Convert from g to m/s^2
    accelX *= 9.80665;
    accelY *= 9.80665;
    accelZ *= 9.80665;

    // scale values
    accelX *= accelXscale;
    accelY *= accelYscale;
    accelZ *= accelZscale;
  }
  //    aSqrt = mySensor.accelSqrt();
  else {
    Serial.println("Cannot read accel values " + String(result));
  }
}

// Get magnetometer readings
void getMagReadings() {
  mX = 0;  // clear data
  mY = 0;
  mZ = 0;

  //Poll gyroY-91 for new data
  if (mySensor.magUpdate() == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    //      mDirection = mySensor.magHorizDirection();
  } else {
    Serial.println("Cannot read mag values " + String(result));
  }
}

void getLidarReadings() {
  //  //Poll VL53L5cx for new data
  //  if (myImager.isDataReady() == true)
  //  {
  //    int count = 0;
  //    float range_temp;
  //    //    for (int x = 7; x >= 0; x--) { // Erase existing range data
  //    //      range[x] = 0;
  //    //    }
  //
  //    if (myImager.getRangingData(&measurementData)) //Read distance data into array
  //    {
  //      //The ST library returns the data transposed from zone mapping shown in datasheet
  //      for (int i = 4; i <= 60; i += 8)
  //      { // selecting a row of data to retrieve
  //
  //        if ( measurementData.target_status[i] == 5) // validate data, 5 if valid
  //        {
  //          //          Serial.println(measurementData.target_status[i]);
  //          //          Serial.println(measurementData.distance_mm[i]);
  //          range_temp = measurementData.distance_mm[i];  // convert mm to m
  //          //          Serial.println(range_temp);
  //          range[count] = range_temp / 1000;
  //          //                    Serial.println(range[count]);
  //        }
  //        else
  //        {
  //          range[count] = 0; // discard range value if target_status equals to 255
  //        }
  //        count++;
  //      }
  //    }
  //  } else {
  //    //    Serial.println("Cannot read lidar values ");
  //  }
}











// Motor Data functions
void getMotorDataPub(unsigned long time) {  // for publishODOM()
  rpm_act1Pub = double((posA - countAnt1Pub) * 60 * 1000) / double(time * encoder_cpr * gear_ratio);
  rpm_act2Pub = double((posB - countAnt2Pub) * 60 * 1000) / double(time * encoder_cpr * gear_ratio);
  countAnt1Pub = posA;
  countAnt2Pub = posB;
  speed_act1Pub = (rpm_act1Pub * pi * wheel_diameter) / 60;
  speed_act2Pub = (rpm_act2Pub * pi * wheel_diameter) / 60;
}


void getMotorData(unsigned long time) {  // for updatePid()
  rpm_act1 = double((posA - countAnt1) * 60 * 1000) / double(time * encoder_cpr * gear_ratio);
  rpm_act2 = double((posB - countAnt2) * 60 * 1000) / double(time * encoder_cpr * gear_ratio);
  countAnt1 = posA;
  countAnt2 = posB;
  speed_act1 = (rpm_act1 * pi * wheel_diameter) / 60;
  speed_act2 = (rpm_act2 * pi * wheel_diameter) / 60;
}

// PID function
int updatePid(int id, int command, double targetValue, double currentValue, double Kp, double Ki, double Kd) {
  double pidTerm = 0;
  double error = targetValue - currentValue;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;

  if (id == 1) {
    int_error1 += error;
    int_error1 = constrain(int_error1, -MAX_RPM, MAX_RPM);  // Prevent integral windup
    pidTerm = Kp * error + Kd * (error - last_error1) + Ki * int_error1;
    last_error1 = error;
  } else {
    int_error2 += error;
    int_error2 = constrain(int_error2, -MAX_RPM, MAX_RPM);  // Prevent integral windup
    pidTerm = Kp * error + Kd * (error - last_error2) + Ki * int_error2;
    last_error2 = error;
  }

  double new_pwm = constrain(double(command) * MAX_RPM / 255.0 + pidTerm, -MAX_RPM, MAX_RPM);
  double new_cmd = 255.0 * new_pwm / MAX_RPM;
  return int(new_cmd);
}



// Interrupt functions
// count encoder ticks for motor A
void IRAM_ATTR encoder1() {
  if (direction1 == FORWARD) {
    posA++;
  }
  if (direction1 == BACKWARD) {
    posA--;
  }
}
// count encoder ticks for motor B
void IRAM_ATTR encoder2() {
  if (direction2 == FORWARD) {
    posB++;
  }
  if (direction2 == BACKWARD) {
    posB--;
  }
}





// set pin logic for motor A direction
void motor1run(int direct) {
  if (direct == FORWARD) {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MA1, HIGH);
    digitalWrite(MA2, LOW);
  } else if (direct == BACKWARD) {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MA1, LOW);
    digitalWrite(MA2, HIGH);
  } else if (direct == RELEASE) {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MA1, HIGH);
    digitalWrite(MA2, HIGH);
  }
}

// set pin logic for motor B direction
void motor2run(int direct) {
  if (direct == FORWARD) {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MB1, LOW);
    digitalWrite(MB2, HIGH);
  } else if (direct == BACKWARD) {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MB1, HIGH);
    digitalWrite(MB2, LOW);
  } else if (direct == RELEASE) {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MB1, HIGH);
    digitalWrite(MB2, HIGH);
  }
}


void motor1setSpeed(int Speed) {  // set pin pwm for motor A direction
  analogWrite(APWM, Speed);
}


void motor2setSpeed(int Speed) {  // set pin pwm for motor B direction
  analogWrite(BPWM, Speed);
}


template<typename T> int sgn(T val) {  // return the sign of a variable

  return (T(0) < val) - (val < T(0));
}






































void IRAM_ATTR onTimer() {  // simple function to be called every period

  getMotorData(millis() - PIDlastMilli);
  PIDlastMilli = millis();  // pass millis() into lastMilli

  PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1, Kp, Ki, Kd);
  PWM_val2 = updatePid(2, PWM_val2, rpm_req2, rpm_act2, Kp, Ki, Kd);

  if (PWM_val1 > 0) direction1 = FORWARD;
  else if (PWM_val1 < 0) direction1 = BACKWARD;
  if (rpm_req1 == 0) {
    direction1 = RELEASE;
    PWM_val1 = 0;
  }

  if (PWM_val2 > 0) direction2 = FORWARD;
  else if (PWM_val2 < 0) direction2 = BACKWARD;
  if (rpm_req2 == 0) {
    direction2 = RELEASE;
    PWM_val2 = 0;
  }

  motor1run(direction1);
  motor2run(direction2);
  motor1setSpeed(abs(PWM_val1));
  motor2setSpeed(abs(PWM_val2));
}










void setup() {
  // put your setup code here, to run once
  
  setupXV11();
  setupWiFi();
  setupSensors();
  setupMotors();
  delay(2000);
  setupNodeHandler();

  // configure encoder interrupt
  pinMode(encodPinA, INPUT_PULLUP);
  pinMode(encodPinB, INPUT_PULLUP);
  attachInterrupt(encodPinA, encoder1, CHANGE);
  attachInterrupt(encodPinB, encoder2, CHANGE);

  // configure timer interrupt
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);  // Attach onTimer function to our timer.
  timerAlarmWrite(My_timer, period * 1000, true);  // set timer in microseconds, on repeat
  timerAlarmEnable(My_timer);                      //Enable timer
}



void loop() {
  // put your main code here, to run repeatedly
  nh.spinOnce();                            // process subscriber callbacks

  getXV11Readings();                        // poll packet of data from LDS
  if (currentAngle == 359) {                // enter LDS publish on full LDS revolution, every ~217ms ,at 4.6Hz
    publishLIDAR(millis() - LDSlastMilli);
    LDSlastMilli = millis();
  }

  if ((millis() - lastMilli) >= LOOPTIME) {  // enter timed loop

    // get IMU data and publish
    getGyroReadings();
    getAccReadings();
    publishIMU(millis() - lastMilli);

    // get ODOM data and publish
    getMotorDataPub(millis() - lastMilli);
    publishODOM(millis() - lastMilli);

    // timestamp of last publish
    lastMilli = millis();                    // pass millis() into lastMilli
  }

  if ((millis() - lastMilli) > LOOPTIME) {
    //print a warning if execution time of the loop in longer than the specified looptime
    Serial.println(" Too Long! LOOPTIME: " + String((millis() - lastMilli)));
  }

}  //loop
