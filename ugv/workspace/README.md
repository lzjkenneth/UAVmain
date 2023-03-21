# Workspace



Remove build and devel files before running $ catkin_make

Required packages:

  roscpp
  rospy
  tf
  geometry_msgs
  sensor_msgs
  std_msgs
  nav_msgs
  
  rosserial_python
  robot_localization
  rviz
  teleop_twist_keyboard
  gmapping
  map_server
  amcl
  move_base


## robot_localization
http://docs.ros.org/en/noetic/api/robot_localization/html/configuring_robot_localization.html

### Don't Repeat Data
only fuse primary sources from odom(x_vel and yaw) and IMU(x_accel and yaw_vel): dont introduce unnecessary biases from duplicate data in kalman filter

### Strategically Include Unmeasured Data
nonholonomic planar robot, zero out unmeasured data in odom(y_vel, z_vel) and IMU(y_accel, z_accel, roll_vel, pitch_vel): fuse sensor data when the data value would consistently be 0, so Kalman filter knows that the data shouldn't change, and can help eliminate problematic data that contradicts that understanding

### Differential parameter
one source of orientation data, _differential false
N sources, _differential true for N−1 of them

set _differential for both to false as we only have one source of absolute orientation data
