#include <ros.h>

//WiFi variables
const char* ssid = "****";
const char* password = "****";

IPAddress server(xxx, xxx, xxx, xxx); // ip of your ROS server
IPAddress ip_address;
int status = WL_IDLE_STATUS;

const uint16_t serverPort = 11411; // tcp socket port

ros::NodeHandle nh;

void setupNodeHandler()
{
  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
}

void setupWiFi()
{
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}

void setup()
{
  Serial.begin(115200);
  setupWiFi();
  setupNodeHandler();
}

void loop(){
  delay(1000);
  nh.spinOnce();
}