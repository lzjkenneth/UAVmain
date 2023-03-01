//
//  UAVionics example codes
//  GY-91 MPU9250 sensor
//




#include <Wire.h>
#include <MPU9250_asukiaaa.h> //https://github.com/asukiaaa/MPU9250_asukiaaa

// I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

#define pi              3.1415926
#define two_pi          6.2831853

MPU9250_asukiaaa mySensor;    // Creating IMU object
float gX, gY, gZ;             // gyroscope variables
float aX, aY, aZ, aSqrt;      // accelerometer variables
float mX, mY, mZ, mDirection; // magnetometer variables

float roll, pitch, yaw;       // rotation axis variables

//Sensor calibration iterations
int num_calibration_itrs = 500; // how much data should be received for offset/bias calibration

//Gyroscope sensor offset/bias
float gyroXoffset = 0.00; // default: 0.08
float gyroYoffset = 0.00; // default: 1.70
float gyroZoffset = 0.00; // default: 1.25
int gyroCalibrated = 0;

//Gyroscope sensor scales
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

double theta_gyro = 0.0;



void setupSensors()
{
  // setup I2C pins and initialize lidar and imu sensors

  Wire.begin(SDA_PIN, SCL_PIN); //This resets to 100kHz I2C
  Wire.setClock(400000); //Sensor has max I2C freq of 400kHz

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

  Serial.println("Calibrating sensors. This can take up to 10s. Please wait and do not move the robot!");
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

  Serial.println("accelXoffset: " + String(accelXoffset) + "accelYoffset: " + String(accelYoffset) + "accelZoffset: " + String(accelZoffset));
  accCalibrated = 1;

}







void getGyroReadings()
{
  // clear pre-existing data
  gX = 0;
  gY = 0;
  gZ = 0;

  //Poll GY-91 for new data
  result = mySensor.gyroUpdate();
  if (result == 0)
  {
    gX = mySensor.gyroX() - gyroXoffset;
    gY = mySensor.gyroY() - gyroYoffset;
    gZ = mySensor.gyroZ() - gyroZoffset;

    // invert gyro data and change from deg/s to rad/s
    gX *= -pi / 180;
    gY *= -pi / 180;
    gZ *= -pi / 180;

    // scale gyro data
    gX *= gXscale;
    gY *= gYscale;
    gZ *= gZscale;

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
    //    aSqrt = mySensor.accelSqrt();
  }

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




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  setupSensors(); // initialize sensor

}

void loop() {
  // put your main code here, to run repeatedly:
  getGyroReadings();
  getAccReadings();
  getMagReadings();

  Serial.println("gX: " + String(gX) + "gY: " + String(gY) + "gZ: " + String(gZ));
  Serial.println("aX: " + String(aX) + "aY: " + String(aY) + "aZ: " + String(aZ));
  Serial.println("mX: " + String(mX) + "mY: " + String(mY) + "mZ: " + String(mZ));

  // calculating angle / orientation
  double dt = 0.1;            // timestep in S
  double dth_gyro = dt * gZ;  // integrating angular velocity to obtain angular displacement
  theta_gyro += dth_gyro;     // angular displacement in radians

  // account for rad overflow
  if (theta_gyro >= two_pi) theta_gyro -= two_pi;
  if (theta_gyro <= -two_pi) theta_gyro += two_pi;
  
  Serial.println("theta_gyro: " + String(theta_gyro));

  delay(100); //Small delay between polling

}
