
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

SparkFun_VL53L5CX myImager;   // Creating Lidar sensor object

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


VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

int imageResolution = 0;      //Used to pretty print output
int imageWidth = 0;           //Used to pretty print output

float range[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float range_target_status[8] = {0, 0, 0, 0, 0, 0, 0, 0};

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

  Serial.println("Initializing VL53L5cx. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("VL53L5cx not found - check your wiring. Freezing"));
    while (1) ;
  }
  Serial.println(F("VL53L5cx found!"));
  myImager.setResolution(8 * 8); //Enable all 64 pads
  myImager.setRangingFrequency(10);
  myImager.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS);
  imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution); //Calculate printing width
  myImager.startRanging();

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
  //Poll VL53L5cx for new data
  if (myImager.isDataReady() == true)
  {
    int count = 0;
    float range_temp;
    //    for (int x = 7; x >= 0; x--) { // Erase existing range data
    //      range[x] = 0;
    //    }

    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      //The ST library returns the data transposed from zone mapping shown in datasheet
      for (int i = 4; i <= 60; i += 8)
      { // selecting a row of data to retrieve

        if ( measurementData.target_status[i] == 5) // validate data, 5 if valid
        {
          //          Serial.println(measurementData.target_status[i]);
          //          Serial.println(measurementData.distance_mm[i]);
          range_temp = measurementData.distance_mm[i];  // convert mm to m
          //          Serial.println(range_temp);
          range[count] = range_temp / 1000;
          //                    Serial.println(range[count]);
        }
        else
        {
          range[count] = 0; // discard range value if target_status equals to 255
        }
        count++;
      }
    }
  } else {
    //    Serial.println("Cannot read lidar values ");
  }

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
{ // require getLidarReadings()
  lidar_msg.header.stamp = nh.now();
  lidar_msg.header.frame_id = laser;
  lidar_msg.angle_min = -0.3927;      // -22.5 degrees
  lidar_msg.angle_max = 0.3927;       // 22.5 degrees
  lidar_msg.angle_increment = 0.0981; // 45 degrees / ranges length
  lidar_msg.time_increment = 0;
  lidar_msg.scan_time = 0;
  lidar_msg.range_min = 0.02;
  lidar_msg.range_max = 4.00;
  lidar_msg.ranges_length = 8;
  lidar_msg.ranges = range;
  lidar.publish(&lidar_msg);
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



void setup()
{ // put your setup code here, to run once
  Serial.begin(115200);
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
{ // put your main code here, to run repeatedly
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

}
