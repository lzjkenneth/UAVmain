//
//  UAVionics example codes
//  VL53L5CX lidar sensor
//


#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX

// I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM


int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

float range[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float range_target_status[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void setupSensors()
{
  // setup I2C pins and initialize lidar and imu sensors

  Wire.begin(SDA_PIN, SCL_PIN);                  // This resets to 100kHz I2C
  Wire.setClock(400000);                         // Sensor has max I2C freq of 400kHz

  // Load the Sensor
  Serial.println("Initializing VL53L5cx. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("VL53L5cx not found - check your wiring. Freezing"));
    while (1) ;
  }
  Serial.println(F("VL53L5cx found!"));

  myImager.setResolution(8 * 8);                 // Enable all 64 pads
  myImager.setRangingFrequency(10);              // Set frequency of collecting ranging data (max 15Hz)
  myImager.setRangingMode(SF_VL53L5CX_RANGING_MODE::CONTINUOUS); // Configure sensor to continuously retrieve ranging data
  imageResolution = myImager.getResolution();    // Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution);            // Calculate printing width
  myImager.startRanging();                       // Start ranging

}



void getLidarReadings()
{
  //Poll VL53L5cx for new data
  if (myImager.isDataReady() == true)
  {
    int count = 0;
    float range_temp;

    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    { //The ST library returns the data transposed from zone mapping shown in datasheet

      for (int i = 4; i <= 60; i += 8)
      { // selecting a row of data to retrieve

        range_target_status[count] = measurementData.target_status[i]; // log data validity status into array

        if ( measurementData.target_status[i] == 5) // validate data, 5 if valid (based on datasheet)
        {
          range_temp = measurementData.distance_mm[i];      // retrieve distance measurement in mm
          range[count] = range_temp / 1000;                 // convert mm to m and log into array
        }
        else
        {
          range[count] = 0;                 // discard range value if target_status not equals to 5
        }
        count++;
      }
    }
  } else {
    //    Serial.println("Cannot read lidar values ");
  }
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  setupSensors(); // initialize sensor

}

void loop() {
  // put your main code here, to run repeatedly:

  getLidarReadings();                 // retrieve lidar data



  Serial.print("range_target_status: \t{");
  for (int i = 0; i < 8; i++)         // repeat 8 times (size of array)
  {
    Serial.print(range_target_status[i]);
    if (i < 7)
    Serial.print(", ");
  }
  Serial.println("}");
  Serial.print("range: \t\t\t{");
  for (int i = 0; i < 8; i++)         // repeat 8 times (size of array)
  {
    Serial.print(range[i]);
    if (i < 7)
    Serial.print(", ");
  }
  Serial.println("}");



  delay(1000); // delay between polling
}
