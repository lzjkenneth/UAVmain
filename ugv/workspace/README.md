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

#### Don't Repeat Data
Only fuse primary sources from odom(yaw and x_vel) and IMU(yaw_vel and x_accel):  
dont introduce unnecessary biases from duplicate data in kalman filter.

#### Strategically Include Unmeasured Data
For a nonholonomic planar robot, zero out unmeasured data in odom(y_vel, z_vel) and IMU(y_accel, z_accel, roll_vel, pitch_vel):  
fuse sensor data when the data value would consistently be 0, so Kalman filter knows that the data shouldn't change, and can help eliminate problematic data that contradicts that understanding.

Set the two_d_mode parameter to true.  
This will automatically zero out all 3D variables, such as Z, roll, pitch, their respective velocities, and Z acceleration.

#### Differential parameter
If you only have one source of orientation data, set _differential param as false.  
For N sources, set _differential as true for N−1 of them.



### Conclusion
Thus, we publish a value of 0 for odom's y_vel and IMU's y_accel.  
We will fuse odom's yaw, x_vel, y_vel and IMU's yaw_vel, x_accel, y_accel.  
Set _differential for both to false as we only have one source of absolute orientation data (odom's yaw).

