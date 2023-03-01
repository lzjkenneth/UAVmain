# Main files

this is where the completed arduino code is located!

ESP_ROS_VL53L5cx, completed arduino code as of 25/02/23

Platform: 
ESP32 Wroom DA - on the ESP32 DevKitC V4

Functions
Wireless ROSSERIAL over tcp connection

Odometry               published via       odom
VL53L5CX lidar         published via       scan
GY-91 IMU              published via       imu
required velocities    subscribed via      cmd_vel

Wireless connection of esp32 and rosmaster via hotspot
PID control loop within timed function
IMU calibration on startup

