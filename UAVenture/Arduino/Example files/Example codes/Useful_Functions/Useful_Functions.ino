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


// Get the actual motor speed and rpm
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




// Publish lidar data
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



void setup()
{// put your setup code here, to run once
  
}



void loop()
{// put your main code here, to run repeatedly

  
  nh.spinOnce();


  if ((millis() - PIDlastMilli) >= period)
  { // enter PID timed loop

    getMotorData(millis() - PIDlastMilli);
    
    PIDlastMilli = millis(); // pass millis() into lastMilli

    PWM_val1 = updatePid(1, PWM_val1, rpm_req1, rpm_act1);
    PWM_val2 = updatePid(2, PWM_val2, rpm_req2, rpm_act2);

    if (PWM_val1 > 0) direction1 = FORWARD;
    else if (PWM_val1 < 0) direction1 = BACKWARD;
    if (rpm_req1 == 0) direction1 = RELEASE;
    //    if (rpm_req1 == 0) PWM_val1 = 0;

    if (PWM_val2 > 0) direction2 = FORWARD;
    else if (PWM_val2 < 0) direction2 = BACKWARD;
    if (rpm_req2 == 0) direction2 = RELEASE;
    //    if (rpm_req1 == 0) PWM_val2 = 0;

    motor1run(direction1);
    motor2run(direction2);
    motor1setSpeed(abs(PWM_val1));
    motor2setSpeed(abs(PWM_val2));

  }
}
