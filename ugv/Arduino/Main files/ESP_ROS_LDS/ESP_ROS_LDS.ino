
#include <PID.h>
#include <EEPROM.h>
#include <EEPROMAnything.h>
#include <SerialCommand.h>

const int N_ANGLES = 360;                // # of angles (0..359)
const int SHOW_ALL_ANGLES = N_ANGLES;    // value means 'display all angle data, 0..359'

struct EEPROM_Config {
  byte id;
  char version[6];
  int motor_pwm_pin;            // pin connected to mosfet for motor speed control
  double rpm_setpoint;          // desired RPM (uses double to be compatible with PID library)
  double rpm_min;
  double rpm_max;
  double pwm_max;              // max analog value.  probably never needs to change from 1023
  double pwm_min;              // min analog pulse value to spin the motor
  int sample_time;             // how often to calculate the PID values

  // PID tuning values
  double Kp;
  double Ki;
  double Kd;

  boolean motor_enable;        // to spin the laser or not.  No data when not spinning
  boolean raw_data;            // to retransmit the seiral data to the USB port
  boolean show_dist;           // controlled by ShowDist and HideDist commands
  boolean show_rpm;            // controlled by ShowRPM and HideRPM commands
  boolean show_interval;       // true = show time interval, once per revolution, at angle=0
  boolean show_errors;         // Show CRC, signal strength and invalid data errors
  boolean aryAngles[N_ANGLES]; // array of angles to display
}
xv_config;

const byte EEPROM_ID = 0x07;   // used to validate EEPROM initialized

double pwm_val_lds = 500;          // start with ~50% power
double pwm_last;
unsigned long now;
unsigned long motor_check_timer = millis();
unsigned long motor_check_interval = 200;
double motor_rpm;
unsigned int rpm_err_thresh = 10;  // 2 seconds (10 * 200ms) to shutdown motor with improper RPM and high voltage
unsigned int rpm_err = 0;
unsigned long curMillis;
unsigned long lastMillislds = millis();

const unsigned char COMMAND = 0xFA;        // Start of new packet
const int INDEX_LO = 0xA0;                 // lowest index value
const int INDEX_HI = 0xF9;                 // highest index value

const int N_DATA_QUADS = 4;                // there are 4 groups of data elements
const int N_ELEMENTS_PER_QUAD = 4;         // viz., 0=distance LSB; 1=distance MSB; 2=sig LSB; 3=sig MSB

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

int Packet[PACKET_LENGTH];                 // an input packet
int ixPacket = 0;                          // index into 'Packet' array
const int VALID_PACKET = 0;
const int INVALID_PACKET = VALID_PACKET + 1;
const byte INVALID_DATA_FLAG = (1 << 7);   // Mask for byte 1 of each data quad "Invalid data"



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
uint16_t aryDist[N_DATA_QUADS] = {0, 0, 0, 0};   // thre are (4) distances, one for each data quad
// so the maximum distance is 16383 mm (0x3FFF)
uint16_t aryQuality[N_DATA_QUADS] = {0, 0, 0, 0}; // same with 'quality'
uint16_t motor_rph = 0;
uint16_t startingAngle = 0;                      // the first scan angle (of group of 4, based on 'index'), in degrees (0..359)

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

















float range[360];
float intensity[360];










#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <MPU9250_asukiaaa.h>

#include <ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h> // for tf::createQuaternionFromYaw()
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>



//initializing all the variables
#define LOOPTIME                      100     //Looptime in milliseconds

// Defining pins

// I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

// Encoder pins
#define encodPinA      18     // encoder A pin
#define encodPinB      19     // encoder B pin

// Motor controller pins
#define APWM           32
#define MA2            33
#define MA1            25
#define STANDBY        26
#define MB1            27
#define MB2            14
#define BPWM           12


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

MPU9250_asukiaaa mySensor;    // Creating IMU object
float aX, aY, aZ, aSqrt;      // accelerometer variables
float gX, gY, gZ;             // gyroscope variables
float mX, mY, mZ, mDirection; // magnetometer variables

float roll, pitch, yaw;       // rotation axis variables

//Sensor calibration iterations
int num_calibration_itrs = 500;

//Gyroscope sensor offsets
float gyroXoffset = 0.00; // default: 0.08
float gyroYoffset = 0.00; // default: 1.70
float gyroZoffset = 0.00; // default: 1.25
int gyroCalibrated = 0;


float gXscale = 1.15;
float gYscale = 1.15;
float gZscale = 1.15;




//Accelerometer sensor offsets
float accelXoffset = 0.00;
float accelYoffset = 0.00;
float accelZoffset = 0.00;
int accCalibrated = 0;

//Magnetometer sensor offsets
float magXoffset = 0.00; // default: -50
float magYoffset = 0.00; // default: -55
float magZoffset = 0.00; // default: -10

uint8_t sensorId;        // sensor state variable
int result;              // sensor output variable




// Timer variables, for timed function execution
unsigned long lastTime = 0;
unsigned long lastMilli = 0;
unsigned long PIDlastMilli = 0;

hw_timer_t * My_timer = NULL;





//--- Robot-specific constants ---

#define wheel_diameter  0.065       // Wheel diameter, in m
#define base_diameter   0.135       // Base diameter or wheel separation, in m
#define base_radius     0.0675      // Base radius, in m
#define gear_ratio      1           // Gear ratio between encoder and motor output

#define encoder_cpr     40           // Encoder ticks or counts per rotation
#define MAX_RPM         333          // Max rotations per minute
#define max_speed       0.5          // Max speed in m/s

#define pi              3.1415926
#define two_pi          6.2831853


// Differential drive working variables and constants

// conversion of motor direction to integers
int FORWARD = 0;
int BACKWARD = 1;
int RELEASE = 2;

// register motor direction for counter
int IRAM_ATTR direction1 = FORWARD;
int IRAM_ATTR direction2 = FORWARD;

volatile long IRAM_ATTR posA = 0;          // encoder A counter
volatile long IRAM_ATTR posB = 0;          // encoder B counter

long countAnt1 = 0;              // posA in last getMotorData callback
long countAnt2 = 0;              // posB in last getMotorData callback
long countAnt1Pub = 0;           // posA in last getMotorDataPub callback
long countAnt2Pub = 0;           // posB in last getMotorDataPub callback

// PID constants
int period = 20;                                   // PID period in milliseconds
double kt = 1000 / period;                         // number of periods/sec
float Kp =   0.1;
float Kd =   0;
float Ki =   0;

// PID control variable for motor speed
int PWM_val1 = 0;
int PWM_val2 = 0;








// Wifi and Rosserial variables
const char* ssid = "KntpAP";          // ssid of your network
const char* password = "tase3281";    // password of your network

int status = WL_IDLE_STATUS;


// Set the rosserial socket server IP address
IPAddress server(192, 168, 33, 199); // ip of your ROS server
IPAddress ip_address;

// Set the rosserial socket server port
const uint16_t serverPort = 11411; // tcp socket port


// ROS nodes //
ros::NodeHandle nh; // edited in ros.h to be NodeHandle_<ArduinoHardware, 10, 10, 2048, 2048> NodeHandle; // default 25, 25, 512, 512

tf::TransformBroadcaster broadcaster;

// ROS message types
geometry_msgs::TransformStamped t;    // transformation frame for odom to base_link
geometry_msgs::TransformStamped l;    // transformation frame for lidar to base_link
geometry_msgs::TransformStamped I;    // transformation frame for imu_link to base_link

nav_msgs::Odometry odom_msg;          // Odometry message
geometry_msgs::Twist cmd_msg;         // cmd_vel message velocity
sensor_msgs::Imu imu_msg;             // imu message
sensor_msgs::LaserScan lidar_msg;     // LaserScan message



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
double rate = 10.0;                 // ros spin rate, used in ros::Rate r(rate); (not implemented)

// scaling speed messages
double linear_scale_positive = 1;
double linear_scale_negative = 1;
double angular_scale_positive = 1;
double angular_scale_negative = 1;

// odometry and transform variables

// odometry variables for coordinate positioning and orientation
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
double theta_odom = 0.0;
double theta_gyro = 0.0;

// transform variables for coordinate positioning and orientation
bool publish_tf = false; // disable tf publishing
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
char base_link[] = "base_link";
char odom[] = "odom";
char laser[] = "laser";
char imu_link[] = "imu_link";
char camera_link[] = "camera_link";
char dummy_link[] = "dummy_link";

// ROS subscriber callback
void handle_cmd( const geometry_msgs::Twist& cmd_msg)
{
  double linear_velocity = cmd_msg.linear.x;      // in m/s
  double angular_velocity = cmd_msg.angular.z;    // in rad/s


  // (linear_velocity * 60)                   -> convert rad/s to rad/min to m/min
  // / (pi * wheel_diameter)                  -> convert m/min to rotations/min
  double linear_rpm = linear_velocity * 60 / (pi * wheel_diameter);                     // convert m/s to rpm

  // (angular_velocity * 60 * base_radius)    -> convert rad/s to rad/min to m/min
  // / (pi * wheel_diameter)                  -> convert m/min to rotations/min
  double angular_rpm = angular_velocity * 60 * base_diameter / ( 2 * pi * wheel_diameter);  // convert rad/s to rpm

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

void setupNodeHandler()
{
  // Set the connection to rosserial socket server

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  broadcaster.init(nh);

  // Start to be polite
  nh.advertise(imu);
  nh.advertise(lidar);
  nh.advertise(odom_pub);

  nh.subscribe(cmd);
}







void setupWiFi()
{
  // Connect to provided network and print ip of device

  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if (i == 21) {
    Serial.print("Could not connect to "); Serial.println(ssid);
    while (1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}


void setupMotors()
{
  // Setup motor pins and initialize motors

  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(APWM, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);
  pinMode(BPWM, OUTPUT);
  pinMode(STANDBY, OUTPUT);
  digitalWrite(STANDBY, HIGH); // HIGH if controlling motor


  motor1setSpeed(0);
  motor2setSpeed(0);
  motor1run(FORWARD);
  motor1run(RELEASE);
  motor2run(FORWARD);
  motor2run(RELEASE);
}


void setupSensors()
{
  // setup I2C pins and initialize lidar and imu sensors

  Wire.begin(SDA_PIN, SCL_PIN); //This resets to 100kHz I2C
  Wire.setClock(400000); //Sensor has max I2C freq of 400kHz

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
  Serial.println("Initializing GY-91. This can take up to 10s. Please wait.");
  if (mySensor.readId(&sensorId) != 0)
  {
    Serial.println(F("GY-91 not found - check your wiring. Freezing"));
    while (1) ;
  }
  Serial.println(F("GY-91 found!"));
  mySensor.beginAccel();
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





void calibrateGyro() // Halt motion whilst calibrating
{ // Halt motion whilst calibrating
  gX = 0; // erasing existing gyro data
  gY = 0;
  gZ = 0;
  for (int count = 0; count < num_calibration_itrs; count++) {
    //Poll GY-91 for new data
    result = mySensor.gyroUpdate();
    if (result == 0)
    {
      gX = mySensor.gyroX();
      gY = mySensor.gyroY();
      gZ = mySensor.gyroZ();
      Serial.println("gX: " + String(gX) + "gY: " + String(gY) + "gZ: " + String(gZ));
    } else {
      Serial.println("Cannot read gyro values " + String(result));
      return;
    }
    gyroXoffset += gX;
    gyroYoffset += gY;
    gyroZoffset += gZ;
  }
  gyroXoffset /= num_calibration_itrs;
  gyroYoffset /= num_calibration_itrs;
  gyroZoffset /= num_calibration_itrs;

  //  gyroXoffset /= num_calibration_itrs - num_calibration_itrs / 5; // increase offset magnitude
  //  gyroYoffset /= num_calibration_itrs - num_calibration_itrs / 5;
  //  gyroZoffset /= num_calibration_itrs - num_calibration_itrs / 5;

  Serial.println("gyroXoffset: " + String(gyroXoffset) + "gyroYoffset: " + String(gyroYoffset) + "gyroZoffset: " + String(gyroZoffset));
  gyroCalibrated = 1;
}

void calibrateAccel() // Halt motion whilst calibrating
{ // Halt motion whilst calibrating
  aX = 0; // erasing existing gyro data
  aY = 0;
  aZ = 0;
  for (int count = 0; count < num_calibration_itrs; count++) {
    //Poll GY-91 for new data
    result = mySensor.accelUpdate();
    if (result == 0)
    {
      aX = mySensor.accelX();
      aY = mySensor.accelY();
      aZ = mySensor.accelZ();
      Serial.println("aX: " + String(aX) + "aY: " + String(aY) + "aZ: " + String(aZ));
    } else {
      Serial.println("Cannot read accel values " + String(result));
      return;
    }
    accelXoffset += aX;
    accelYoffset += aY;
    accelZoffset += aZ;
  }
  accelXoffset /= num_calibration_itrs;
  accelYoffset /= num_calibration_itrs;
  accelZoffset /= num_calibration_itrs;

  //  accelXoffset /= num_calibration_itrs - num_calibration_itrs / 5; // increase offset magnitude
  //  accelYoffset /= num_calibration_itrs - num_calibration_itrs / 5;
  //  accelZoffset /= num_calibration_itrs - num_calibration_itrs / 5;

  Serial.println("accelXoffset: " + String(accelXoffset) + "accelYoffset: " + String(accelYoffset) + "accelZoffset: " + String(accelZoffset));
  accCalibrated = 1;

}







void getGyroReadings()
{
  gX = 0; // clear data
  gY = 0;
  gZ = 0;
  //Poll GY-91 for new data
  result = mySensor.gyroUpdate();
  if (result == 0)
  {
    gX = mySensor.gyroX() - gyroXoffset;
    gY = mySensor.gyroY() - gyroYoffset;
    gZ = mySensor.gyroZ() - gyroZoffset;
    // invert values and change from deg/s to rad/s
    gX *= pi / -180;
    gY *= pi / -180;
    gZ *= pi / -180;
    gX *= gXscale;
    gY *= gYscale;
    gZ *= gZscale;
    //    Serial.println("gX: " + String(gX) + "gY: " + String(gY) + "gZ: " + String(gZ));

    //    float gyroX_temp = mySensor.gyroX();
    //    if (abs(gyroX_temp) > gyroXoffset) {
    //      gX = gyroX_temp;
    //    }
    //    float gyroY_temp = mySensor.gyroY();
    //    if (abs(gyroY_temp) > gyroYoffset) {
    //      gY = gyroY_temp;
    //    }
    //    float gyroZ_temp = mySensor.gyroZ();
    //    if (abs(gyroZ_temp) > gyroZoffset) {
    //      gZ = gyroZ_temp;
  } else {
    Serial.println("Cannot read gyro values " + String(result));
  }
}

void getAccReadings()
{
  aX = 0; // clear data
  aY = 0;
  aZ = 0;
  //Poll GY-91 for new data
  result = mySensor.accelUpdate();
  if (result == 0)
  {
    aX = mySensor.accelX() - accelXoffset;
    aY = mySensor.accelY() - accelYoffset;
    aZ = mySensor.accelZ() - accelZoffset;

    //    float accelX_temp = mySensor.accelX();
    //    if (abs(accelX_temp) > accelXerror) {
    //      aX = accelX_temp;
    //    }
    //    float accelY_temp = mySensor.accelY();
    //    if (abs(accelY_temp) > accelYerror) {
    //      aY = accelY_temp;
    //    }
    //    float accelZ_temp = mySensor.accelZ();
    //    if (abs(accelZ_temp) > accelZerror) {
    //      aZ = accelZ_temp;
  }
  //    aSqrt = mySensor.accelSqrt();
  else {
    Serial.println("Cannot read accel values " + String(result));
  }
}

void getMagReadings()
{
  mX = 0; // clear data
  mY = 0;
  mZ = 0;
  //Poll GY-91 for new data
  result = mySensor.magUpdate();
  if (result == 0)
  {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    //      mDirection = mySensor.magHorizDirection();
  } else {
    Serial.println("Cannot read mag values " + String(result));
  }
}

void getLidarReadings()
{
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






void publishIMU(unsigned long time)
{ // require getGyroReadings() and getAccReadings()

  current_time = nh.now();
  dt = double(time) / 1000;          //Time in s

  dth_gyro = dt * gZ;
  theta_gyro += dth_gyro;

  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = imu_link;

  imu_msg.linear_acceleration.x = aX;
  imu_msg.linear_acceleration.y = aY;
  imu_msg.linear_acceleration.z = aZ;

  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;

  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;

  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.angular_velocity.x = gX;
  imu_msg.angular_velocity.y = gY;
  imu_msg.angular_velocity.z = gZ;

  imu_msg.angular_velocity_covariance[0] = 0.02;
  imu_msg.angular_velocity_covariance[1] = 0;
  imu_msg.angular_velocity_covariance[2] = 0;

  imu_msg.angular_velocity_covariance[3] = 0;
  imu_msg.angular_velocity_covariance[4] = 0.02;
  imu_msg.angular_velocity_covariance[5] = 0;

  imu_msg.angular_velocity_covariance[6] = 0;
  imu_msg.angular_velocity_covariance[7] = 0;
  imu_msg.angular_velocity_covariance[8] = 0.02;

  geometry_msgs::Quaternion gyro_quat = tf::createQuaternionFromYaw(theta_gyro);

  imu_msg.orientation = gyro_quat;

  imu_msg.orientation_covariance[0] = 0.0025;
  imu_msg.orientation_covariance[1] = 0;
  imu_msg.orientation_covariance[2] = 0;

  imu_msg.orientation_covariance[3] = 0;
  imu_msg.orientation_covariance[4] = 0.0025;
  imu_msg.orientation_covariance[5] = 0;

  imu_msg.orientation_covariance[6] = 0;
  imu_msg.orientation_covariance[7] = 0;
  imu_msg.orientation_covariance[8] = 0.0025;

  imu.publish(&imu_msg);
  //  nh.spinOnce();
  //  nh.loginfo("Publishing imu");
}

void publishLIDAR(unsigned long time)
{
  lidar_msg.header.stamp = nh.now();
  lidar_msg.header.frame_id = laser;
  lidar_msg.angle_min = 0.0;
  lidar_msg.angle_max = two_pi;
  lidar_msg.angle_increment = (two_pi / 360);
  lidar_msg.time_increment = 0.0005; //motor_speed/good_sets/1e8;
  lidar_msg.scan_time = 0.2;
  lidar_msg.range_min = 0.06;
  lidar_msg.range_max = 5.00;
  lidar_msg.ranges_length = 360;
  lidar_msg.ranges = range;
  lidar_msg.intensities = intensity;
  lidar.publish(&lidar_msg);
  memset(range, 0, sizeof(range));
  memset(intensity, 0, sizeof(intensity));
  //  nh.spinOnce();
  //  nh.loginfo("Publishing lidar");
}

void publishODOM(unsigned long time)
{ // require getMotorDataPub()
  current_time = nh.now();

  // register time step duration
  dt = double(time) / 1000;                // Change in time, time step, in s

  // calculate velocities
  vx = (dt == 0) ?  0 : (speed_act1Pub + speed_act2Pub) / 2;            // linear velocity, in m
  vth = (dt == 0) ? 0 : (speed_act2Pub - speed_act1Pub) / base_diameter;  // angular velocity, in m

  // scale velocities by set factor
  if (vx > 0) vx *= linear_scale_positive;
  if (vx < 0) vx *= linear_scale_negative;
  if (vth > 0) vth *= angular_scale_positive;
  if (vth < 0) vth *= angular_scale_negative;

  // differentiate velocities into displacement
  dxy = vx * dt;                           // Change in position, displacement, in m
  dth_odom = vth * dt;                     // Change in yaw, angular displacement, in m

  //  nh.loginfo("dt : %f", dt);
  //  nh.loginfo("dxy : %f", dxy);

  // extract change in x and change in y
  dx = cos(dth_odom) * dxy;                // Change in x position
  dy = sin(dth_odom) * dxy;                // Change in y position

  // add change in x and change in y to position variable
  x_pos += dx;
  y_pos += dy;

  //  x_pos += (cos(theta_odom) * dx - sin(theta_odom) * dy);
  //  y_pos += (sin(theta_odom) * dx + cos(theta_odom) * dy);

  // add change in yaw to yaw variable
  theta_odom += dth_odom;

  // account for rad overflow
  if (theta_odom >= two_pi) theta_odom -= two_pi;
  if (theta_odom <= -two_pi) theta_odom += two_pi;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta_odom);
  geometry_msgs::Quaternion empty_quat = tf::createQuaternionFromYaw(0);

  odom_msg.header.stamp = current_time;
  odom_msg.header.frame_id = odom;
  odom_msg.child_frame_id = base_link;
  odom_msg.pose.pose.position.x = x_pos;
  odom_msg.pose.pose.position.y = y_pos;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;

  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = vth;

  // writing covariances
  if (speed_act1Pub == 0 && speed_act2Pub == 0) {
    odom_msg.pose.covariance[0] = 1e-9;
    odom_msg.pose.covariance[7] = 1e-3;
    odom_msg.pose.covariance[8] = 1e-9;
    odom_msg.pose.covariance[14] = 1e6;
    odom_msg.pose.covariance[21] = 1e6;
    odom_msg.pose.covariance[28] = 1e6;
    odom_msg.pose.covariance[35] = 1e-9;
    odom_msg.twist.covariance[0] = 1e-9;
    odom_msg.twist.covariance[7] = 1e-3;
    odom_msg.twist.covariance[8] = 1e-9;
    odom_msg.twist.covariance[14] = 1e6;
    odom_msg.twist.covariance[21] = 1e6;
    odom_msg.twist.covariance[28] = 1e6;
    odom_msg.twist.covariance[35] = 1e-9;
  }
  else {
    odom_msg.pose.covariance[0] = 1e-3;
    odom_msg.pose.covariance[7] = 1e-3;
    odom_msg.pose.covariance[8] = 0.0;
    odom_msg.pose.covariance[14] = 1e6;
    odom_msg.pose.covariance[21] = 1e6;
    odom_msg.pose.covariance[28] = 1e6;
    odom_msg.pose.covariance[35] = 1e3;
    odom_msg.twist.covariance[0] = 1e-3;
    odom_msg.twist.covariance[7] = 1e-3;
    odom_msg.twist.covariance[8] = 0.0;
    odom_msg.twist.covariance[14] = 1e6;
    odom_msg.twist.covariance[21] = 1e6;
    odom_msg.twist.covariance[28] = 1e6;
    odom_msg.twist.covariance[35] = 1e3;
  }

  odom_pub.publish(&odom_msg);

  if (publish_tf) { // transform relation can be handled by static publishers in ros

    t.header.frame_id = odom;
    t.child_frame_id = base_link;
    t.transform.translation.x = x_pos;
    t.transform.translation.y = y_pos;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom_quat;
    t.header.stamp = current_time;

    l.header.frame_id = laser;
    l.child_frame_id = camera_link;
    l.transform.translation.x = 0.0;
    l.transform.translation.y = 0.0;
    l.transform.translation.z = 0.0;
    l.transform.rotation = empty_quat;
    l.header.stamp = current_time;

    I.header.frame_id = imu_link;
    I.child_frame_id = dummy_link;
    I.transform.translation.x = 0.0;
    I.transform.translation.y = 0.0;
    I.transform.translation.z = 0.0;
    I.transform.rotation = empty_quat;
    I.header.stamp = current_time;

    broadcaster.sendTransform(t);
    broadcaster.sendTransform(l);
    broadcaster.sendTransform(I);
  }

  //  Serial.println("dt: " + String(dt) + "  theta_odom: " + String(dxy));
  //  Serial.println("dx: " + String(dx) + "  dy: " + String(dy));
  //  Serial.println("x_pos: " + String(x_pos) + "  y_pos: " + String(y_pos));
  //  Serial.println("dth_odom: " + String(dth_odom) + "  theta_odom: " + String(theta_odom));
  //  nh.spinOnce();
  //  nh.loginfo("Publishing odom");
}





void getMotorDataPub(unsigned long time)
{ // for publishODOM()
  rpm_act1Pub = double((posA - countAnt1Pub) * 60 * 1000) / double(time * encoder_cpr * gear_ratio);
  rpm_act2Pub = double((posB - countAnt2Pub) * 60 * 1000) / double(time * encoder_cpr * gear_ratio);
  countAnt1Pub = posA;
  countAnt2Pub = posB;
  speed_act1Pub = (rpm_act1Pub * pi * wheel_diameter) / 60;
  speed_act2Pub = (rpm_act2Pub * pi * wheel_diameter) / 60;

}


void getMotorData(unsigned long time)
{ // for updatePid()
  rpm_act1 = double((posA - countAnt1) * 60 * 1000) / double(time * encoder_cpr * gear_ratio);
  rpm_act2 = double((posB - countAnt2) * 60 * 1000) / double(time * encoder_cpr * gear_ratio);
  countAnt1 = posA;
  countAnt2 = posB;
  speed_act1 = (rpm_act1 * pi * wheel_diameter) / 60;
  speed_act2 = (rpm_act2 * pi * wheel_diameter) / 60;

}


int updatePid(int id, int command, double targetValue, double currentValue)
{ // compute new pwm values
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;

  error = targetValue - currentValue;
  if (id == 1) {
    int_error1 += error;
    pidTerm = Kp * error + Kd * (error - last_error1) + Ki * int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp * error + Kd * (error - last_error2) + Ki * int_error2;
    last_error2 = error;
  }
  new_pwm = constrain(double(command) * MAX_RPM / 255.0 + pidTerm, -MAX_RPM, MAX_RPM);
  new_cmd = 255.0 * new_pwm / MAX_RPM;
  return int(new_cmd);
}

void IRAM_ATTR encoder1()
{ // count encoder ticks for motor A
  if (direction1 == FORWARD) {
    posA++;
  }
  if (direction1 == BACKWARD) {
    posA--;
  }
}

void IRAM_ATTR encoder2()
{ // count encoder ticks for motor B
  if (direction2 == FORWARD) {
    posB++;
  }
  if (direction2 == BACKWARD) {
    posB--;
  }
}






void motor1run(int direct)
{ // set pin logic for motor A direction
  if (direct == FORWARD)
  {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MA1, HIGH);
    digitalWrite(MA2, LOW);
  }
  else if (direct == BACKWARD)
  {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MA1, LOW);
    digitalWrite(MA2, HIGH);
  }
  else if (direct == RELEASE)
  {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MA1, HIGH);
    digitalWrite(MA2, HIGH);
  }
}


void motor2run(int direct)
{ // set pin logic for motor B direction
  if (direct == FORWARD)
  {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MB1, LOW);
    digitalWrite(MB2, HIGH);
  }
  else if (direct == BACKWARD)
  {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MB1, HIGH);
    digitalWrite(MB2, LOW);
  }
  else if (direct == RELEASE)
  {
    digitalWrite(STANDBY, HIGH);
    digitalWrite(MB1, HIGH);
    digitalWrite(MB2, HIGH);
  }
}


void motor1setSpeed(int Speed)
{ // set pin pwm for motor A direction
  analogWrite(APWM, Speed);
}


void motor2setSpeed(int Speed)
{ // set pin pwm for motor B direction
  analogWrite(BPWM, Speed);
}


template <typename T> int sgn(T val)
{ // return the sign of a variable

  return (T(0) < val) - (val < T(0));
}










void IRAM_ATTR onTimer()
{ // simple code to be triggered every period

  getMotorData(millis() - PIDlastMilli);
  PIDlastMilli = millis(); // pass millis() into lastMilli

  PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1);
  PWM_val2 = updatePid(2, PWM_val2, rpm_req2, rpm_act2);

  if (PWM_val1 > 0) direction1 = FORWARD;
  else if (PWM_val1 < 0) direction1 = BACKWARD;
  if (rpm_req1 == 0) direction1 = RELEASE;
  if (rpm_req1 == 0) PWM_val1 = 0;

  if (PWM_val2 > 0) direction2 = FORWARD;
  else if (PWM_val2 < 0) direction2 = BACKWARD;
  if (rpm_req2 == 0) direction2 = RELEASE;
  if (rpm_req2 == 0) PWM_val2 = 0;

  motor1run(direction1);
  motor2run(direction2);
  motor1setSpeed(abs(PWM_val1));
  motor2setSpeed(abs(PWM_val2));

}

















/*
   processIndex - Process the packet element 'index'
   index is the index byte in the 90 packets, going from A0 (packet 0, readings 0 to 3) to F9
      (packet 89, readings 356 to 359).
   Enter with: N/A
   Uses:       Packet
               ledState gets toggled if angle = 0
               ledPin = which pin the LED is connected to
               ledState = LED on or off
               xv_config.show_dist = true if we're supposed to show distance
               curMillis = milliseconds, now
               lastMillislds = milliseconds, last time through this subroutine
               xv_config.show_interval = true ==> display time interval once per revolution, at angle 0
   Calls:      digitalWrite() - used to toggle LED pin
               Serial.print
   Returns:    The first angle (of 4) in the current 'index' group
*/
uint16_t processIndex() {
  uint16_t angle = 0;
  uint16_t data_4deg_index = Packet[OFFSET_TO_INDEX] - INDEX_LO;
  angle = data_4deg_index * N_DATA_QUADS;     // 1st angle in the set of 4
  if (angle == 0) {
    if (ledState) {
      ledState = LOW;
    }
    else {
      ledState = HIGH;
    }
    digitalWrite(ledPin, ledState);

    if (xv_config.show_rpm) {
      Serial.print(F("R,"));
      Serial.print((int)motor_rpm);
      Serial.print(F(","));
      Serial.println((int)pwm_val_lds);
    }

    curMillis = millis();
    if (xv_config.show_interval) {
      Serial.print(F("T,"));                                // Time Interval in ms since last complete revolution
      Serial.println(curMillis - lastMillislds);
    }
    lastMillislds = curMillis;

  } // if (angle == 0)
  return angle;
}
/*
   processSpeed- Process the packet element 'speed'
   speed is two-bytes of information, little-endian. It represents the speed, in 64th of RPM (aka value
      in RPM represented in fixed point, with 6 bits used for the decimal part).
   Enter with: N/A
   Uses:       Packet
               angle = if 0 then enable display of RPM and PWM
               xv_config.show_rpm = true if we're supposed to display RPM and PWM
   Calls:      Serial.print
*/
void processSpeed() {
  motor_rph_low_byte = Packet[OFFSET_TO_SPEED_LSB];
  motor_rph_high_byte = Packet[OFFSET_TO_SPEED_MSB];
  motor_rph = (motor_rph_high_byte << 8) | motor_rph_low_byte;
  motor_rpm = float( (motor_rph_high_byte << 8) | motor_rph_low_byte ) / 64.0;
}
/*
   Data 0 to Data 3 are the 4 readings. Each one is 4 bytes long, and organized as follows :
     byte 0 : <distance 7:0>
     byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8>
     byte 2 : <signal strength 7:0>
     byte 3 : <signal strength 15:8>
*/
/*
   processDistance- Process the packet element 'distance'
   Enter with: iQuad = which one of the (4) readings to process, value = 0..3
   Uses:       Packet
               dist[] = sets distance to object in binary: ISbb bbbb bbbb bbbb
                                       so maximum distance is 0x3FFF (16383 decimal) millimeters (mm)
   Calls:      N/A
   Exits with: 0 = okay
   Error:      1 << 7 = INVALID_DATA_FLAG is set
               1 << 6 = STRENGTH_WARNING_FLAG is set
*/
byte processDistance(int iQuad) {
  uint8_t dataL, dataM;
  aryDist[iQuad] = 0;                     // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_DISTANCE_LSB;
  // byte 0 : <distance 7:0> (LSB)
  // byte 1 : <"invalid data" flag> <"strength warning" flag> <distance 13:8> (MSB)
  dataM = Packet[iOffset + 1];           // get MSB of distance data + flags
  if (dataM & BAD_DATA_MASK)             // if either INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set...
    return dataM & BAD_DATA_MASK;        // ...then return non-zero
  dataL = Packet[iOffset];               // LSB of distance data
  aryDist[iQuad] = dataL | ((dataM & 0x3F) << 8);
  return 0;                              // okay
}
/*
   processSignalStrength- Process the packet element 'signal strength'
   Enter with: iQuad = which one of the (4) readings to process, value = 0..3
   Uses:       Packet
               quality[] = signal quality
   Calls:      N/A
*/
void processSignalStrength(int iQuad) {
  uint8_t dataL, dataM;
  aryQuality[iQuad] = 0;                        // initialize
  int iOffset = OFFSET_TO_4_DATA_READINGS + (iQuad * N_DATA_QUADS) + OFFSET_DATA_SIGNAL_LSB;
  dataL = Packet[iOffset];                  // signal strength LSB
  dataM = Packet[iOffset + 1];
  aryQuality[iQuad] = dataL | (dataM << 8);
}

/*
   eValidatePacket - Validate 'Packet'
   Enter with: 'Packet' is ready to check
   Uses:       CalcCRC
   Exits with: 0 = Packet is okay
   Error:      non-zero = Packet is no good
*/
byte eValidatePacket() {
  unsigned long chk32;
  unsigned long checksum;
  const int bytesToCheck = PACKET_LENGTH - 2;
  const int CalcCRC_Len = bytesToCheck / 2;
  unsigned int CalcCRC[CalcCRC_Len];

  byte b1a, b1b, b2a, b2b;
  int ix;

  for (int ix = 0; ix < CalcCRC_Len; ix++)       // initialize 'CalcCRC' array
    CalcCRC[ix] = 0;

  // Perform checksum validity test
  for (ix = 0; ix < bytesToCheck; ix += 2)      // build 'CalcCRC' array
    CalcCRC[ix / 2] = Packet[ix] + ((Packet[ix + 1]) << 8);

  chk32 = 0;
  for (ix = 0; ix < CalcCRC_Len; ix++)
    chk32 = (chk32 << 1) + CalcCRC[ix];
  checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
  checksum &= 0x7FFF;
  b1a = checksum & 0xFF;
  b1b = Packet[OFFSET_TO_CRC_L];
  b2a = checksum >> 8;
  b2b = Packet[OFFSET_TO_CRC_M];
  if ((b1a == b1b) && (b2a == b2b))
    return VALID_PACKET;                       // okay
  else
    return INVALID_PACKET;                     // non-zero = bad CRC
}

/*
   initEEPROM
*/
void initEEPROM() {
  xv_config.id = 0x07;
  strcpy(xv_config.version, "1.4.0");

#if defined(__AVR_ATmega32U4__) && defined(CORE_TEENSY)  // if Teensy 2.0
  xv_config.motor_pwm_pin = 9;  // pin connected N-Channel Mosfet

#elif defined(__AVR_ATmega32U4__)  // if Leonardo or Pro Micro
  xv_config.motor_pwm_pin = 5;  // pin connected N-Channel Mosfet

#elif defined(__MK20DX256__)  // if Teensy 3.1
  xv_config.motor_pwm_pin = 33;  // pin connected N-Channel Mosfet

#elif defined(__MKL26Z64__)  // if Teensy LC
  xv_config.motor_pwm_pin = 4;  // pin connected N-Channel Mosfet

#elif defined(ESP32)
  xv_config.motor_pwm_pin = 17;  // pin connected N-Channel Mosfet

#endif

  xv_config.rpm_setpoint = 200;  // desired RPM
  xv_config.rpm_min = 200;
  xv_config.rpm_max = 300;
  xv_config.pwm_min = 100;
  xv_config.pwm_max = 1023;
  xv_config.sample_time = 20;
  xv_config.Kp = 2.0;
  xv_config.Ki = 1.0;
  xv_config.Kd = 0.0;

  xv_config.motor_enable = true;
  xv_config.raw_data = true;
  xv_config.show_dist = false;
  xv_config.show_rpm = false;
  xv_config.show_interval = false;
  xv_config.show_errors = false;
  for (int ix = 0; ix < N_ANGLES; ix++)
    xv_config.aryAngles[ix] = true;

  EEPROM_writeAnything(0, xv_config);
}
/*
   initSerialCommands
*/
void initSerialCommands() {
  sCmd.addCommand("help",        help);
  sCmd.addCommand("Help",        help);
  sCmd.addCommand("?",        help);
  sCmd.addCommand("ShowConfig",  showConfig);
  sCmd.addCommand("SaveConfig",  saveConfig);
  sCmd.addCommand("ResetConfig", initEEPROM);

  sCmd.addCommand("SetAngle",      setAngle);
  sCmd.addCommand("SetRPM",        setRPM);
  sCmd.addCommand("SetKp",         setKp);
  sCmd.addCommand("SetKi",         setKi);
  sCmd.addCommand("SetKd",         setKd);
  sCmd.addCommand("SetSampleTime", setSampleTime);

  sCmd.addCommand("MotorOff", motorOff);
  sCmd.addCommand("MotorOn",  motorOn);

  sCmd.addCommand("ShowRaw",  showRaw);
  sCmd.addCommand("HideRaw", hideRaw);
  sCmd.addCommand("ShowDist",  showDist);
  sCmd.addCommand("HideDist",  hideDist);
  sCmd.addCommand("ShowRPM",  showRPM);
  sCmd.addCommand("HideRPM",  hideRPM);
  sCmd.addCommand("ShowErrors", showErrors);
  sCmd.addCommand("HideErrors", hideErrors);
  sCmd.addCommand("ShowInterval", showInterval);
  sCmd.addCommand("HideInterval", hideInterval);
  sCmd.addCommand("ShowAll", showAll);
  sCmd.addCommand("HideAll", hideAll);

}
/*
   showAll - Show Dist, Errors, RPM, and Interval data
*/
void showAll() {
  showDist();
  showErrors();
  showRPM();
  showInterval();
}
/*
   hideAll - Hide Dist, Errors, RPM, and Interval data
*/
void hideAll() {
  hideDist();
  hideErrors();
  hideRPM();
  hideInterval();
}
/*
   showInterval - enable display of Time interval (which happens once per revolution, at angle 0
*/
void showInterval() {
  xv_config.show_interval = true;
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Showing time interval (ms per revolution)"));
  }
}
/*
   hideInterval - suppress display of Time interval
*/
void hideInterval() {
  xv_config.show_interval = false;
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding time interval"));
  }
}
/*
   showErrors
*/
void showErrors() {
  xv_config.show_errors = true;                                  // enable error display
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Showing errors"));
  }
}
/*
   hideErrors
*/
void hideErrors() {                                    // disable error display
  xv_config.show_errors = false;
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding errors"));
  }
}
/*
   showRPM
*/
void showRPM() {
  xv_config.show_rpm = true;
  if (xv_config.raw_data == true) {
    hideRaw();
  }
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Showing RPM data"));
  }
}
/*
   hideRPM
*/
void hideRPM() {
  xv_config.show_rpm = false;
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding RPM data"));
  }
}

void showDist() {
  hideRaw();
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Code,Angle,Distance(mm),Signal strength"));
  }
  xv_config.show_dist = true;
}

void hideDist() {
  xv_config.show_dist = false;
  if (xv_config.show_dist == false) {                  // suppress activity message if we're executing 'show distance'
    Serial.println(F(" "));
    Serial.println(F("Hiding Distance data"));
  }
}
/*
   doSetAngle - Multi-angle range(s) implementation - DSH
   Command: SetAngles ddd, ddd-ddd, etc.
   Enter with: N/A
   Uses:       xv_config.aryAngles (an array of 360 booleans) is set to appropriate values
   Calls:      showDist
   Exits with: N/A
   TEST THIS STRING:  SetAngles 16-20, 300-305, 123-124, 10
*/
void setAngle() {
  char c, *arg;
  boolean syntax_error = false;
  int doing_from_to, from, to, ix, lToken, n_groups = 0;

  for (ix = 0; ix < N_ANGLES; ix++)                      // initialize
    xv_config.aryAngles[ix] = false;
  doing_from_to = 0;                                     // state = doing 'from'
  // Make sure that there is at least 1 angle or group of angles present
  do {
    arg = sCmd.next();                                   // get the next token
    if (arg == NULL) {                                   // it's empty -- just exit
      sCmd.readSerial();
      arg = sCmd.next();
      break;
    }
    // see if the token has an embedded "-", meaning from - to
    lToken = strlen(arg);                                // get the length of the current token
    for (ix = 0; ix < lToken; ix++) {
      c = arg[ix];
      if (c == ',') {                                    // optional trailing comma
        doing_from_to = 0;
        break;
      }
      else if (c == '-') {                               // optional '-' means "from - to"
        to = 0;
        doing_from_to = 1;                               // from now on, we're doing 'to'
      }
      else if (c == ' ') {                               // ignore blanks
        Serial.println(F("{ }"));
      }
      else if ((c >= '0') && (c <= '9')) {
        if (doing_from_to == 0) {
          from *= 10;
          from += c - '0';
          to = from;                                      // default to = from
          n_groups++;                                     // count the number of active groups (s/b >= 1)
        }
        else {
          to *= 10;
          to += c - '0';
        }
      }
      else {
        syntax_error = true;
        n_groups = 0;
        break;
      }
    }  // for (ix = 0; ix < lToken; ix++)
    // validate 'from' and 'to' and set 'xv_config.aryAngles' with correct values
    if ((from >= 0) && (from < N_ANGLES) && (to >= 0) && (to < N_ANGLES)) {
      if (to >= from) {
        for (ix = from; ix <= to; ix++) {
          xv_config.aryAngles[ix] = true;
        }
      }
      else {
        syntax_error = true;
        break;
      }
    }
    else {
      syntax_error = true;
      break;
    }
    from = 0;
    to = 0;
    doing_from_to = 0;
  }  // do
  while (arg != NULL);
  if (n_groups == 0)
    syntax_error = true;

  // Handle syntax errors
  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax"));
    Serial.println(F("  Example: SetAngle 0, 15-30, 45-50, 10"));
    Serial.println(F("  Example: SetAngle 0-359 to show all angles."));
    Serial.println(F("Notes: Use a space after each comma"));
    Serial.println(F("       No particular order is required"));
    Serial.println(F("       In a from-to pair, the 1st value must be lowest. From-to pairs can overlap ranges."));
  }
  else {                                                  // no errors detected, display the angles and start
    // We're ready to process multiple angles
    Serial.println(F(""));
    Serial.print(F("Angles:"));
    for (int ix = 0; ix < N_ANGLES; ix++) {               // display the angle array
      if (xv_config.aryAngles[ix]) {
        Serial.print(ix, DEC);
        Serial.print(F(","));
      }
    }
    Serial.println(F(""));
    showDist();
  }  // if not (syntax_error)
}

void motorOff() {
  xv_config.motor_enable = false;
  ledcWrite(ledChannel, 0);
  Serial.println(F(" "));
  Serial.println(F("Motor off"));
}

void motorOn() {
  xv_config.motor_enable = true;
  ledcWrite(ledChannel, pwm_val_lds);
  rpm_err = 0;  // reset rpm error
  Serial.println(F(" "));
  Serial.println(F("Motor on"));
}

void motorCheck() {  // Make sure the motor RPMs are good else shut it down
  now = millis();
  if (now - motor_check_timer > motor_check_interval) {
    if ((motor_rpm < xv_config.rpm_min or motor_rpm > xv_config.rpm_max) and pwm_val_lds > 1000) {
      rpm_err++;
    }
    else {
      rpm_err = 0;
    }
    if (rpm_err > rpm_err_thresh) {
      motorOff();
      ledState = LOW;
      digitalWrite(ledPin, ledState);
    }
    motor_check_timer = millis();
  }
}

void hideRaw() {
  xv_config.raw_data = false;
  //Serial.println(F(" "));
  //Serial.println(F("Raw lidar data disabled"));
}

void showRaw() {
  xv_config.raw_data = true;
  hideDist();
  hideRPM();
  //Serial.println(F(" "));
  //Serial.println(F("Lidar data enabled"));
}

void setRPM() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
    if (sVal < xv_config.rpm_min) {
      sVal = xv_config.rpm_min;
      Serial.println(F(" "));
      Serial.print(F("RPM too low. Setting to minimum "));
      Serial.println(xv_config.rpm_min);
    }
    if (sVal > xv_config.rpm_max) {
      sVal = xv_config.rpm_max;
      Serial.println(F(" "));
      Serial.print(F("RPM too high. Setting to maximum "));
      Serial.println(xv_config.rpm_max);
    }
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetRPM 200"));
  }
  else {
    Serial.print(F("Old RPM setpoint:"));
    Serial.println(xv_config.rpm_setpoint);
    xv_config.rpm_setpoint = sVal;
    //Serial.println(F(" "));
    Serial.print(F("New RPM setpoint: "));
    Serial.println(sVal);
  }
}

void setKp() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetKp 1.0"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Kp to: "));
    Serial.println(sVal);
    xv_config.Kp = sVal;
    rpmPID.SetTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  }
}

void setKi() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetKi 0.5"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Ki to: "));
    Serial.println(sVal);
    xv_config.Ki = sVal;
    rpmPID.SetTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  }
}

void setKd() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atof(arg);    // Converts a char string to a float
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetKd 0.001"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Kd to: "));
    Serial.println(sVal);
    xv_config.Kd = sVal;
    rpmPID.SetTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  }
}

void setSampleTime() {
  double sVal = 0.0;
  char *arg;
  boolean syntax_error = false;

  arg = sCmd.next();
  if (arg != NULL) {
    sVal = atoi(arg);    // Converts a char string to an integer
  }
  else {
    syntax_error = true;
  }

  arg = sCmd.next();
  if (arg != NULL) {
    syntax_error = true;
  }

  if (syntax_error) {
    Serial.println(F(" "));
    Serial.println(F("Incorrect syntax.  Example: SetSampleTime 20"));
  }
  else {
    Serial.println(F(" "));
    Serial.print(F("Setting Sample time to: "));
    Serial.println(sVal);
    xv_config.sample_time = sVal;
    rpmPID.SetSampleTime(xv_config.sample_time);
  }
}

void help() {
  if (xv_config.raw_data == true) {
    hideRaw();
  }
  Serial.println(F(" "));
  Serial.println(F(" "));

  Serial.print(F("XV Lidar Controller Firmware Version "));
  Serial.println(xv_config.version);
  Serial.print(F("GetSurreal.com *"));

  Serial.println(F(" "));
  Serial.println(F(" "));

  Serial.println(F("List of available commands"));

  Serial.println(F(" "));
  Serial.println(F("Control commands"));
  Serial.println(F("  ShowConfig    - Show the running configuration"));
  Serial.println(F("  SaveConfig    - Save the running configuration to EEPROM"));
  Serial.println(F("  ResetConfig   - Restore the original configuration"));
  Serial.println(F("  SetAngle      - Show distance data for a multiple angles (Ex: SetAngle 0, 15-30, 45-50, 10)"));
  Serial.println(F("  SetRPM        - Set the desired rotation speed (min: 180, max: 349)"));
  Serial.println(F("  MotorOff      - Stop spinning the lidar"));
  Serial.println(F("  MotorOn       - Enable spinning of the lidar"));

  Serial.println(F(" "));
  Serial.println(F("Data commands"));
  Serial.println(F("  ShowRaw       - Enable the output of the raw lidar data (default)"));
  Serial.println(F("  HideRaw       - Stop outputting the raw data from the lidar"));
  Serial.println(F("  ShowDist      - Show angles with distance data"));
  Serial.println(F("  HideDist      - Hide the distance data"));
  Serial.println(F("  ShowErrors    - Show all error types (CRC, Signal Strength, and Invalid"));
  Serial.println(F("  HideErrors    - Hide angles with errors"));
  Serial.println(F("  ShowRPM       - Show the rotation speed"));
  Serial.println(F("  HideRPM       - Hide the rotation speed"));
  Serial.println(F("  ShowInterval  - Show time interval per revolution in ms, at angle=0"));
  Serial.println(F("  HideInterval  - Hide time interval"));
  Serial.println(F("  ShowAll       - Show the distance, errors, RPMs and interval data"));
  Serial.println(F("  HideAll       - Hide the distance, errors, RPMs and interval data"));

  Serial.println(F(" "));
  Serial.println(F("PID commands"));
  Serial.println(F("  SetKp         - Set the proportional gain"));
  Serial.println(F("  SetKi         - Set the integral gain"));
  Serial.println(F("  SetKd         - Set the derivative gain"));
  Serial.println(F("  SetSampleTime - Set the frequency the PID is calculated (ms)"));

  Serial.println(F(" "));
  Serial.println(F("Output comma-separated format:"));
  Serial.println(F("  A,<Angle>,<Distance in mm>,<Strength>"));
  Serial.println(F("  C,CRC error was generated by LIDAR"));
  Serial.println(F("  R,<RPMs>,<PWM value>"));
  Serial.println(F("  T,<Time interval in milliseconds>"));

  Serial.println(F(" "));
  Serial.println(F("Errors:"));
  Serial.println(F("  CRC = CRC Error"));
  Serial.println(F("    I = LIDAR reports Invalid data for this angle"));
  Serial.println(F("    S = LIDAR reports Poor signal strength for this angle"));
  Serial.println(F(" "));
}

void showConfig() {
  if (xv_config.raw_data == true) {
    hideRaw();
  }
  Serial.println(F(" "));
  Serial.println(F(" "));

  Serial.print(F("XV Lidar Controller Firmware Version "));
  Serial.println(xv_config.version);
  Serial.print(F("GetSurreal.com"));

  Serial.println(F(" "));
  Serial.println(F(" "));

  Serial.print(F("PWM pin: "));
  Serial.println(xv_config.motor_pwm_pin);

  Serial.print(F("Target RPM: "));
  Serial.println(xv_config.rpm_setpoint);

  Serial.print(F("Max PWM: "));
  Serial.println(xv_config.pwm_max);
  Serial.print(F("Min PWM: "));
  Serial.println(xv_config.pwm_min);

  Serial.print(F("PID Kp: "));
  Serial.println(xv_config.Kp);
  Serial.print(F("PID Ki: "));
  Serial.println(xv_config.Ki);
  Serial.print(F("PID Kd: "));
  Serial.println(xv_config.Kd);
  Serial.print(F("SampleTime: "));
  Serial.println(xv_config.sample_time);

  Serial.print(F("Motor Enable: "));
  Serial.println(xv_config.motor_enable);
  Serial.print(F("Show Raw Data: "));
  Serial.println(xv_config.raw_data);
  Serial.print(F("Show Dist Data: "));
  Serial.println(xv_config.show_dist);
  Serial.print(F("Show RPM Data: "));
  Serial.println(xv_config.show_rpm);
  Serial.print(F("Show Time Interval: "));
  Serial.println(xv_config.show_interval);
  Serial.print(F("Show Angle(s): "));
  for (int ix = 0; ix < N_ANGLES; ix++) {               // display the angle array
    if (xv_config.aryAngles[ix]) {
      Serial.print(ix, DEC);
      Serial.print(F(","));
    }
  }
  Serial.println(F(" "));
  Serial.println(F(" "));
}

void saveConfig() {
  EEPROM_writeAnything(0, xv_config);
  Serial.println(F("Config Saved."));
}

























void setup()
{
  EEPROM_readAnything(0, xv_config);
  if ( xv_config.id != EEPROM_ID) { // verify EEPROM values have been initialized
    initEEPROM();
  }
  pinMode(xv_config.motor_pwm_pin, OUTPUT);
  Serial.begin(115200);                    // USB serial
#if defined(__AVR_ATmega32U4__)
  Serial1.begin(115200);                   // XV LDS data
#elif defined(__MK20DX256__)               // if Teensy 3.1
  Serial1.begin(115200);                   // XV LDS data
#elif defined(__MKL26Z64__)                // if Teensy LC
  Serial1.begin(115200);                   // XV LDS data
#elif defined(ESP32)                // if Teensy LC
  Serial1.begin(115200, SERIAL_8N1, 16, 4);                   // XV LDS data Rx Tx
#endif


  //added from here

#if defined(ESP32)
  Serial.println("enter setup");
#endif

  xv_config.show_dist = true;
  xv_config.rpm_setpoint = 300;


  ledcSetup(ledChannel, freq, resolution); // configure LED PWM functionalitites
  ledcAttachPin(xv_config.motor_pwm_pin, ledChannel); // attach the channel to the GPIO to be controlled


  //to here


  rpmPID.SetOutputLimits(xv_config.pwm_min, xv_config.pwm_max);
  rpmPID.SetSampleTime(xv_config.sample_time);
  rpmPID.SetTunings(xv_config.Kp, xv_config.Ki, xv_config.Kd);
  rpmPID.SetMode(AUTOMATIC);

  initSerialCommands();
  pinMode(ledPin, OUTPUT);

  eState = eState_Find_COMMAND;
  for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // Initialize
    Packet[ixPacket] = 0;
  ixPacket = 0;


















  // put your setup code here, to run once
  //  Serial.begin(115200);
  setupWiFi();
  setupSensors();
  setupMotors();
  delay(2000);
  setupNodeHandler();

  // configure encoder interrupt
  pinMode(encodPinA, INPUT_PULLUP);
  pinMode(encodPinB, INPUT_PULLUP);
  //  digitalWrite(encodPinA, HIGH);                // turn on pullup resistor
  //  digitalWrite(encodPinB, HIGH);
  attachInterrupt(encodPinA, encoder1, CHANGE);
  attachInterrupt(encodPinB, encoder2, CHANGE);

  // configure timer interrupt
  My_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer, &onTimer, true);  // Attach onTimer function to our timer.
  timerAlarmWrite(My_timer, period * 1000, true); // set timer in microseconds, on repeat
  timerAlarmEnable(My_timer); //Enable timer
}

void loop()
{
  byte aryInvalidDataFlag[N_DATA_QUADS] = {0, 0, 0, 0}; // non-zero = INVALID_DATA_FLAG or STRENGTH_WARNING_FLAG is set
  //  Serial.println("t");

  sCmd.readSerial();  // check for incoming serial commands
  if (Serial1.available() > 0) {                  // read byte from LIDAR and relay to USB
    inByte = Serial1.read();                      // get incoming byte:
    if (xv_config.raw_data)
      Serial.write(inByte);                 // relay

    // Switch, based on 'eState':
    // State 1: We're scanning for 0xFA (COMMAND) in the input stream
    // State 2: Build a complete data packet
    if (eState == eState_Find_COMMAND) {          // flush input until we get COMMAND byte
      if (inByte == COMMAND) {
        eState++;                                 // switch to 'build a packet' state
        Packet[ixPacket++] = inByte;              // store 1st byte of data into 'Packet'
      }
    }
    else {                                            // eState == eState_Build_Packet
      Packet[ixPacket++] = inByte;                    // keep storing input into 'Packet'
      if (ixPacket == PACKET_LENGTH) {                // we've got all the input bytes, so we're done building this packet
        if (eValidatePacket() == VALID_PACKET) {      // Check packet CRC
          startingAngle = processIndex();             // get the starting angle of this group (of 4), e.g., 0, 4, 8, 12, ...
          processSpeed();                             // process the speed
          // process each of the (4) sets of data in the packet
          for (int ix = 0; ix < N_DATA_QUADS; ix++)   // process the distance
            aryInvalidDataFlag[ix] = processDistance(ix);
          for (int ix = 0; ix < N_DATA_QUADS; ix++) { // process the signal strength (quality)
            aryQuality[ix] = 0;
            if (aryInvalidDataFlag[ix] == 0)
              processSignalStrength(ix);
          }
          if (xv_config.show_dist) {                           // the 'ShowDistance' command is active
            for (int ix = 0; ix < N_DATA_QUADS; ix++) {
              if (xv_config.aryAngles[startingAngle + ix]) {             // if we're supposed to display that angle
                if (aryInvalidDataFlag[ix] & BAD_DATA_MASK) {  // if LIDAR reported a data error...
                  if (xv_config.show_errors) {                           // if we're supposed to show data errors...
                    Serial.print(F("A,"));
                    Serial.print(startingAngle + ix);
                    Serial.print(F(","));
                    if (aryInvalidDataFlag[ix] & INVALID_DATA_FLAG)
                      Serial.println(F("I"));
                    if (aryInvalidDataFlag[ix] & STRENGTH_WARNING_FLAG)
                      Serial.println(F("S"));
                  }
                }
                else {                                         // show clean data
                  //                  Serial.print(F("A,"));
                  //                  Serial.print(startingAngle + ix);
                  //                  Serial.print(F(","));
                  //                  Serial.print(int(aryDist[ix]));
                  //                  Serial.print(F(","));
                  //                  Serial.println(aryQuality[ix]);
                  range[startingAngle + ix] = float(aryDist[ix]) / 1000; //mm to m
                  intensity[startingAngle + ix] = float(aryQuality[ix]);


                }
              }  // if (xv_config.aryAngles[startingAngle + ix])
            }  // for (int ix = 0; ix < N_DATA_QUADS; ix++)
          }  // if (xv_config.show_dist)
        }  // if (eValidatePacket() == 0
        else if (xv_config.show_errors) {                                // we have encountered a CRC error
          Serial.println(F("C,CRC"));
        }
        // initialize a bunch of stuff before we switch back to State 1
        for (int ix = 0; ix < N_DATA_QUADS; ix++) {
          aryDist[ix] = 0;
          aryQuality[ix] = 0;
          aryInvalidDataFlag[ix] = 0;
        }
        for (ixPacket = 0; ixPacket < PACKET_LENGTH; ixPacket++)  // clear out this packet
          Packet[ixPacket] = 0;
        ixPacket = 0;
        eState = eState_Find_COMMAND;                // This packet is done -- look for next COMMAND byte
      }  // if (ixPacket == PACKET_LENGTH)
    }  // if (eState == eState_Find_COMMAND)
  }  // if (Serial1.available() > 0)
  if (xv_config.motor_enable) {
    rpmPID.Compute();
    if (pwm_val_lds != pwm_last) {
      ledcWrite(ledChannel, pwm_val_lds);  // replacement for analogWrite()
      pwm_last = pwm_val_lds;
    }
    motorCheck();
  }  // if (xv_config.motor_enable)























  // put your main code here, to run repeatedly
  nh.spinOnce();


  if ((millis() - PIDlastMilli) >= period)
  { // enter PID timed loop

    //        getMotorData(millis() - PIDlastMilli);
    //        PIDlastMilli = millis(); // pass millis() into lastMilli
    //
    //        PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1);
    //        PWM_val2 = updatePid(2, PWM_val2, rpm_req2, rpm_act2);
    //
    //        if (PWM_val1 > 0) direction1 = FORWARD;
    //        else if (PWM_val1 < 0) direction1 = BACKWARD;
    //        if (rpm_req1 == 0) direction1 = RELEASE;
    //        //    if (rpm_req1 == 0) PWM_val1 = 0;
    //
    //        if (PWM_val2 > 0) direction2 = FORWARD;
    //        else if (PWM_val2 < 0) direction2 = BACKWARD;
    //        if (rpm_req2 == 0) direction2 = RELEASE;
    //        //    if (rpm_req1 == 0) PWM_val2 = 0;
    //
    //        motor1run(direction1);
    //        motor2run(direction2);
    //        motor1setSpeed(abs(PWM_val1));
    //        motor2setSpeed(abs(PWM_val2));

  }







  if ((millis() - lastMilli) >= LOOPTIME)
  { // enter timed loop

    //  Serial.println("direction1: " + String(direction1) + "  direction2: " + String(direction2));
    //  Serial.println("rpm_req1: " + String(rpm_req1) + "  rpm_req2: " + String(rpm_req2));
    //  Serial.println("speed_act1Pub: " + String(speed_act1Pub) + "  speed_act2Pub: " + String(speed_act2Pub));
    //  Serial.println("posA: " + String(posA) + "  posB: " + String(posB));
    //        Serial.println("PWM_val1: " + String(PWM_val1) + "  PWM_val2: " + String(PWM_val2));
    //        Serial.println("rpm_act1Pub: " + String(rpm_act1Pub) + "  rpm_act2Pub: " + String(rpm_act2Pub));

    getGyroReadings();
    getAccReadings();
    publishIMU(millis() - lastMilli);

    getLidarReadings();
    publishLIDAR(millis() - lastMilli);

    getMotorDataPub(millis() - lastMilli);   // getMotorData since last publish
    publishODOM(millis() - lastMilli);

    //    Serial.println("LOOPTIME: " + String((millis() - lastMilli)));
    lastMilli = millis(); // pass millis() into lastMilli

    nh.spinOnce();
  }





  if ((millis() - lastMilli) >= LOOPTIME)
  { //print a warning if execution time of the loop in longer than the specified looptime
    Serial.println(" Too Long! LOOPTIME: " + String((millis() - lastMilli)));
  }

}//loop
