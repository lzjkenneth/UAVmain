// ROS functions for publishing messages



void publishIMU(unsigned long time) {  // require getGyroReadings() and getAccReadings()

  current_time = nh.now();
  dt = double(time) / 1000;  //Time in s

  dth_gyro = dt * gyroZ;
  theta_gyro += dth_gyro;

  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = imu_link;

  imu_msg.linear_acceleration.x = accelX;
  imu_msg.linear_acceleration.y = 0; // accelY = 0, nonholonomic robot
  imu_msg.linear_acceleration.z = 0; // accelZ = 0, planar robot

  imu_msg.linear_acceleration_covariance[0] = 0.04;
  imu_msg.linear_acceleration_covariance[1] = 0;
  imu_msg.linear_acceleration_covariance[2] = 0;

  imu_msg.linear_acceleration_covariance[3] = 0;
  imu_msg.linear_acceleration_covariance[4] = 0.04;
  imu_msg.linear_acceleration_covariance[5] = 0;

  imu_msg.linear_acceleration_covariance[6] = 0;
  imu_msg.linear_acceleration_covariance[7] = 0;
  imu_msg.linear_acceleration_covariance[8] = 0.04;

  imu_msg.angular_velocity.x = 0; // gyroX = 0, planar robot
  imu_msg.angular_velocity.y = 0; // gyroY = 0, planar robot
  imu_msg.angular_velocity.z = gyroZ;

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
}

void publishLIDAR(unsigned long time) {
  lidar_msg.header.stamp = nh.now();
  lidar_msg.header.frame_id = laser;
  lidar_msg.angle_min = 0.0;
  lidar_msg.angle_max = two_pi;
  lidar_msg.angle_increment = (two_pi / 360);
  lidar_msg.time_increment = (double(time) / 1000) / 360;  // scan time / points
  lidar_msg.scan_time = double(time) / 1000;               // time since last call, ms to s
  lidar_msg.range_min = 0.06;
  lidar_msg.range_max = 5.00;
  lidar_msg.ranges_length = 360;
  lidar_msg.ranges = range;
  lidar_msg.intensities = intensity;
  lidar.publish(&lidar_msg);

  lastAngle = currentAngle; // register index of latest range data for validation purposes
  memset(range, 0, sizeof(range)); //clear array in the event errors preserve old data
  memset(intensity, 0, sizeof(intensity));
}

void publishODOM(unsigned long time) {  // require getMotorDataPub()
  current_time = nh.now();

  // register time step duration
  dt = double(time) / 1000;  // Change in time, time step, in s

  // calculate velocities
  vx = (dt == 0) ? 0 : (speed_act1Pub + speed_act2Pub) / 2;               // linear velocity, in m
  vth = (dt == 0) ? 0 : (speed_act2Pub - speed_act1Pub) / base_diameter;  // angular velocity, in m

  // scale velocities by set factor
  if (vx > 0) vx *= linear_scale_positive;
  if (vx < 0) vx *= linear_scale_negative;
  if (vth > 0) vth *= angular_scale_positive;
  if (vth < 0) vth *= angular_scale_negative;

  // differentiate velocities into displacement
  dxy = vx * dt;        // Change in position, displacement, in m
  dth_odom = vth * dt;  // Change in yaw, angular displacement, in m

  // extract change in x and change in y
  dx = cos(dth_odom) * dxy;  // Change in x position
  dy = sin(dth_odom) * dxy;  // Change in y position

  // add change in x and change in y to position variable
  x_pos += dx;
  y_pos += dy;

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
  } else {
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

  if (publish_tf) {  // transform relation can be handled by static publishers in ros

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

}