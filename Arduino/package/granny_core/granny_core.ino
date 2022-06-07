/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho, Gilbert */

#include "granny_core_config.h"

/*******************************************************************************
* Setup function
*******************************************************************************/
void setup()
{
  DEBUG_SERIAL.begin(57600);

  // Initialize ROS node handle, advertise and subscribe the topics
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(cmd_vel_sub);
  nh.subscribe(sound_sub);
  nh.subscribe(motor_power_sub);
  nh.subscribe(reset_sub);

  nh.advertise(sensor_state_pub);
  nh.advertise(version_info_pub);
  nh.advertise(imu_pub);
  nh.advertise(odom_pub);
  nh.advertise(joint_states_pub);
  nh.advertise(battery_state_pub);
  nh.advertise(mag_pub);

  tf_broadcaster.init(nh);

  motor_driver.init(NAME,MOTOR_BAUDRATE,debug_buf);  // Setting for Dynamixel motors

  sensors.init();  // Setting for IMU
  diagnosis.init();  // Init diagnosis

  // Setting for ROBOTIS RC100 remote controller and cmd_vel
  controllers.init(MAX_LINEAR_VELOCITY, MAX_ANGULAR_VELOCITY);      ////수정필요 한지 잘 모르겠음 LINEAR X,Y

  initOdom();  // Setting for SLAM and navigation (odometry, joint states, TF)
  initJointStates();
  prev_update_time = millis();
  pinMode(LED_WORKING_CHECK, OUTPUT);
  setup_end = true;
}

/*******************************************************************************
* Loop function
*******************************************************************************/
void loop()
{
  uint32_t t = millis();
  updateTime();
  updateVariable(nh.connected());
  updateTFPrefix(nh.connected());

  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    updateGoalVelocity();
    if ((t-tTime[6]) > CONTROL_MOTOR_TIMEOUT)  motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, zero_velocity);
    else motor_driver.controlMotor(WHEEL_RADIUS, WHEEL_SEPARATION, goal_velocity);           /////////////goal_velocity를 받아와서 모터를 움직여야됨
    tTime[0] = t;
  }

  if ((t-tTime[2]) >= (1000 / DRIVE_INFORMATION_PUBLISH_FREQUENCY))  {
    publishSensorStateMsg();
    publishBatteryStateMsg();
    publishDriveInformation();
    tTime[2] = t;
  }

  if ((t-tTime[3]) >= (1000 / IMU_PUBLISH_FREQUENCY))  {
    publishImuMsg();
    publishMagMsg();
    tTime[3] = t;
  }

  if ((t-tTime[4]) >= (1000 / VERSION_INFORMATION_PUBLISH_FREQUENCY))  {
    publishVersionInfoMsg();
    tTime[4] = t;
  }

#ifdef DEBUG
  if ((t-tTime[5]) >= (1000 / DEBUG_LOG_FREQUENCY))  {
    sendDebuglog();
    tTime[5] = t;
  }
#endif

  sendLogMsg();// Send log message after ROS connection
  driveTest(diagnosis.getButtonPress(3000));  // Check push button pressed for simple test drive
  sensors.updateIMU();// Update the IMU unit
  updateGyroCali(nh.connected());  // Start Gyro Calibration after ROS connection
  diagnosis.showLedStatus(nh.connected());  // Show LED status
  battery_state = diagnosis.updateVoltageCheck(setup_end);  // Update Voltage
  nh.spinOnce();  // Call all the callbacks waiting to be called at that point in time
  waitForSerialLink(nh.connected());  // Wait the serial link time to process
}

/*******************************************************************************
* Callback function for cmd_vel msg                                                     /////////////수정필요 LINEAR를 X, Y로 나눠줘야함 따라서 goal_velocity_from_cmd 함수도 바꿔줘야함
*******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg)
{
  goal_velocity_from_cmd[LINEAR_X]  = cmd_vel_msg.linear.x;
  goal_velocity_from_cmd[LINEAR_Y]  = cmd_vel_msg.linear.y;
  goal_velocity_from_cmd[ANGULAR] = cmd_vel_msg.angular.z;
  goal_velocity_from_cmd[LINEAR_X]  = constrain(goal_velocity_from_cmd[LINEAR_X],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[LINEAR_Y]  = constrain(goal_velocity_from_cmd[LINEAR_Y],  MIN_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY);
  goal_velocity_from_cmd[ANGULAR] = constrain(goal_velocity_from_cmd[ANGULAR], MIN_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
  tTime[6] = millis();
}

/*******************************************************************************
* Callback function for sound msg
*******************************************************************************/
void soundCallback(const turtlebot3_msgs::Sound& sound_msg){
  sensors.makeSound(sound_msg.value);
}

/*******************************************************************************
* Callback function for motor_power msg
*******************************************************************************/
void motorPowerCallback(const std_msgs::Bool& power_msg){
  bool dxl_power = power_msg.data;
  motor_driver.setTorque(dxl_power);
  char log_msg[50];
  if (dxl_power) sprintf(log_msg, "Motor Torque On!!");
  else sprintf(log_msg, "Motor Torque Off!!");
  nh.loginfo(log_msg);
}

/*******************************************************************************
* Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty& reset_msg){
  char log_msg[50];
  (void)(reset_msg);
  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);
  sensors.calibrationGyro();
  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);
  initOdom();
  sprintf(log_msg, "Reset Odometry");
  nh.loginfo(log_msg);
}

/*******************************************************************************
* Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void publishImuMsg(void)
{
  imu_msg = sensors.getIMU();
  imu_msg.header.stamp    = rosNow();
  imu_msg.header.frame_id = imu_frame_id;
  imu_pub.publish(&imu_msg);
}

/*******************************************************************************
* Publish msgs (Magnetic data)
*******************************************************************************/
void publishMagMsg(void)
{
  mag_msg = sensors.getMag();
  mag_msg.header.stamp    = rosNow();
  mag_msg.header.frame_id = mag_frame_id;
  mag_pub.publish(&mag_msg);
}

/*******************************************************************************
* Publish msgs (sensor_state: bumpers, cliffs, buttons, encoders, battery)
*******************************************************************************/
void publishSensorStateMsg(void)
{
  int32_t left_rear_encoder, right_rear_encoder, left_front_encoder,right_front_encoder;
  bool dxl_comm_result = motor_driver.readEncoder(left_rear_encoder, right_rear_encoder, left_front_encoder, right_front_encoder); ////수정완료
  if (dxl_comm_result == true)
    updateMotorInfo(left_rear_encoder, right_rear_encoder, left_front_encoder, right_front_encoder);   /////수정완료
  else return;
  //sj: we reuse old sensorstate
  sensor_state_msg.header.stamp = rosNow();
  sensor_state_msg.battery = sensors.checkVoltage();
  sensor_state_msg.bumper = sensors.checkPushBumper();
  sensor_state_msg.cliff = sensors.getIRsensorData();
  sensor_state_msg.illumination = sensors.getIlluminationData();
  sensor_state_msg.button = sensors.checkPushButton();
  sensor_state_msg.torque = motor_driver.getTorque();
  sensor_state_pub.publish(&sensor_state_msg);
}

/*******************************************************************************
* Publish msgs (version info)
*******************************************************************************/
void publishVersionInfoMsg(void)
{
  version_info_msg.hardware = "0.0.0";
  version_info_msg.software = "0.0.0";
  version_info_msg.firmware = FIRMWARE_VER;
  version_info_pub.publish(&version_info_msg);
}

/*******************************************************************************
* Publish msgs (battery_state)
*******************************************************************************/
void publishBatteryStateMsg(void)
{
  battery_state_msg.header.stamp = rosNow();
  battery_state_msg.design_capacity = 1.8f; //Ah
  battery_state_msg.voltage = sensors.checkVoltage();
  battery_state_msg.percentage = (float)(battery_state_msg.voltage / 11.1f);
  if (battery_state == 0)    battery_state_msg.present = false;
  else    battery_state_msg.present = true;
  battery_state_pub.publish(&battery_state_msg);
}

/*******************************************************************************
* Publish msgs (odometry, joint states, tf)                                               //수정필요 odom 계산해서 바꿔줘야함 나중에
*******************************************************************************/
void publishDriveInformation(void)
{
  unsigned long time_now = millis();
  unsigned long step_time = time_now - prev_update_time;

  prev_update_time = time_now;
  ros::Time stamp_now = rosNow();

  // calculate odometry
  calcOdometry((double)(step_time * 0.001));

  // odometry
  updateOdometry();
  odom.header.stamp = stamp_now;
  odom_pub.publish(&odom);

  // odometry tf
  updateTF(odom_tf);
  odom_tf.header.stamp = stamp_now;
  tf_broadcaster.sendTransform(odom_tf);

  // joint states
  updateJointStates();
  joint_states.header.stamp = stamp_now;
  joint_states_pub.publish(&joint_states);
}

/*******************************************************************************
* Update TF Prefix                                                                //수정해야될지는 모르겠음 안해도 될것 같아보임
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)  {
    if (isChecked == false)    {
      nh.getParam("~tf_prefix", &get_tf_prefix);
      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");
        sprintf(imu_frame_id, "imu_link");
        sprintf(mag_frame_id, "mag_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }      else      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);
        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(mag_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);
        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");
        strcat(imu_frame_id, "/imu_link");
        strcat(mag_frame_id, "/mag_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }
      sprintf(log_msg, "Setup TF on Odometry [%s]", odom_header_frame_id);
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setup TF on MagneticField [%s]", mag_frame_id);
      nh.loginfo(log_msg);
      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg);
      isChecked = true;
    }
  }
  else  {    isChecked = false;  }
}

/*******************************************************************************
* Update the odometry                                                                     ////수정필요 linear y 추가 ////수정완료?
*******************************************************************************/
void updateOdometry(void)
{
  odom.header.frame_id = odom_header_frame_id;
  odom.child_frame_id  = odom_child_frame_id;
  odom.pose.pose.position.x = odom_pose[0];
  odom.pose.pose.position.y = odom_pose[1];
  odom.pose.pose.position.z = 0;
  odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);
  odom.twist.twist.linear.x  = odom_vel[0];
  odom.twist.twist.linear.y  = odom_vel[1];
  odom.twist.twist.angular.z = odom_vel[2];
}

/*******************************************************************************
* Update the joint states                                                               //수정필요 left right rear front 추가만 해주면됨 wheel}_num은 변경 되어있음///////////////////수정완료
*******************************************************************************/
void updateJointStates(void)
{
  static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
  static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
  joint_states_pos[LEFT_REAR]  = last_rad[LEFT_REAR];
  joint_states_pos[RIGHT_REAR] = last_rad[RIGHT_REAR];
  joint_states_pos[LEFT_FRONT]  = last_rad[LEFT_FRONT];
  joint_states_pos[RIGHT_FRONT] = last_rad[RIGHT_FRONT];
  joint_states_vel[LEFT_REAR]  = last_velocity[LEFT_REAR];
  joint_states_vel[RIGHT_REAR] = last_velocity[RIGHT_REAR];
  joint_states_vel[LEFT_FRONT]  = last_velocity[LEFT_FRONT];
  joint_states_vel[RIGHT_FRONT] = last_velocity[RIGHT_FRONT];
  joint_states.position = joint_states_pos;
  joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
* CalcUpdateulate the TF
*******************************************************************************/
void updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom.header;
  odom_tf.child_frame_id = odom.child_frame_id;
  odom_tf.transform.translation.x = odom.pose.pose.position.x;
  odom_tf.transform.translation.y = odom.pose.pose.position.y;
  odom_tf.transform.translation.z = odom.pose.pose.position.z;
  odom_tf.transform.rotation      = odom.pose.pose.orientation;
}

/*******************************************************************************
* Update motor information
*******************************************************************************/
void updateMotorInfo(int32_t left_rear_tick, int32_t right_rear_tick, int32_t left_front_tick, int32_t right_front_tick)  ////수정완료
{
  int32_t current_tick = 0;
  static int32_t last_tick[WHEEL_NUM] = {0, 0};
  if (init_encoder)  {
    for (int index = 0; index < WHEEL_NUM; index++)    {
      last_diff_tick[index] = 0;
      last_tick[index]      = 0;
      last_rad[index]       = 0.0;
      last_velocity[index]  = 0.0;
    }
    last_tick[LEFT_REAR] = left_rear_tick;
    last_tick[RIGHT_REAR] = right_rear_tick;
    last_tick[LEFT_FRONT] = left_front_tick;
    last_tick[RIGHT_FRONT] = right_front_tick;
    init_encoder = false;
    return;
  }
  current_tick = left_rear_tick;
  last_diff_tick[LEFT_REAR] = current_tick - last_tick[LEFT_REAR];
  last_tick[LEFT_REAR]      = current_tick;
  last_rad[LEFT_REAR]       += TICK2RAD * (double)last_diff_tick[LEFT_REAR];
  current_tick = right_rear_tick;
  last_diff_tick[RIGHT_REAR] = current_tick - last_tick[RIGHT_REAR];
  last_tick[RIGHT_REAR]      = current_tick;
  last_rad[RIGHT_REAR]       += TICK2RAD * (double)last_diff_tick[RIGHT_REAR];
  current_tick = left_front_tick;
  last_diff_tick[LEFT_FRONT] = current_tick - last_tick[LEFT_FRONT];
  last_tick[LEFT_FRONT]      = current_tick;
  last_rad[LEFT_FRONT]       += TICK2RAD * (double)last_diff_tick[LEFT_FRONT];
  current_tick = right_front_tick;
  last_diff_tick[RIGHT_FRONT] = current_tick - last_tick[RIGHT_FRONT];
  last_tick[RIGHT_FRONT]      = current_tick;
  last_rad[RIGHT_FRONT]       += TICK2RAD * (double)last_diff_tick[RIGHT_FRONT];
}

/*******************************************************************************
* Calculate the odometry                                                            //수정필요 나중에 전부 다 뜯어고쳐야함
*******************************************************************************/
bool calcOdometry(double diff_time)
{
  float* orientation;
  double wheel_left_rear, wheel_right_rear, wheel_left_front, wheel_right_front;      // rotation value of wheel [rad]
  double delta_x, delta_y, theta, delta_theta;
  static double last_theta = 0.0;
  double v_x, v_y, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
  double step_time;

  wheel_left_rear = wheel_right_rear = wheel_left_front = wheel_right_front = 0.0;
  delta_x = delta_y = delta_theta = theta = 0.0;
  v_x = v_y = w = 0.0;
  step_time = 0.0;
  step_time = diff_time;
  if (step_time == 0)    return false;

  wheel_left_rear = TICK2RAD * (double)last_diff_tick[LEFT_REAR];
  wheel_right_rear = TICK2RAD * (double)last_diff_tick[RIGHT_REAR];
  wheel_left_front = TICK2RAD * (double)last_diff_tick[LEFT_FRONT];
  wheel_right_front = TICK2RAD * (double)last_diff_tick[RIGHT_FRONT];

  if (isnan(wheel_left_rear))
    wheel_left_rear = 0.0;

  if (isnan(wheel_right_rear))
    wheel_right_rear = 0.0;

  if (isnan(wheel_left_front))
    wheel_left_front = 0.0;

  if (isnan(wheel_right_front))
    wheel_right_front = 0.0;

  delta_x     = WHEEL_RADIUS * (wheel_left_rear + wheel_right_rear + wheel_left_front + wheel_right_front) / 4.0;
//  delta_x     = WHEEL_RADIUS * (wheel_left_rear - wheel_right_rear + wheel_left_front - wheel_right_front) / 4.0;
//  delta_x     = WHEEL_RADIUS * (wheel_left_rear - wheel_right_rear - wheel_left_front + wheel_right_front) / 4.0;
  delta_y     = WHEEL_RADIUS * (wheel_left_rear - wheel_right_rear - wheel_left_front + wheel_right_front) / 4.0;
//  delta_y     = WHEEL_RADIUS * (wheel_left_rear + wheel_right_rear - wheel_left_front - wheel_right_front) / 4.0;
//  delta_y     = WHEEL_RADIUS * (wheel_left_rear + wheel_right_rear + wheel_left_front + wheel_right_front) / 4.0;
  // theta = WHEEL_RADIUS * (- wheel_left_rear + wheel_right_rear - wheel_left_front + wheel_right_front) / (4.0 * WHEEL_SEPARATION);
  orientation = sensors.getOrientation();
  theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3],
                0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += (delta_x * cos(odom_pose[2] + (delta_theta / 2.0)) + delta_y * sin(odom_pose[2] + (delta_theta / 2.0)));
  odom_pose[1] += (delta_x * sin(odom_pose[2] + (delta_theta / 2.0)) + delta_y * cos(odom_pose[2] + (delta_theta / 2.0)));
  odom_pose[2] += delta_theta;

//  odom_pose[0] += delta_x;
//  odom_pose[1] += delta_y;
//  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  v_x = delta_x / step_time;
  v_y = delta_y / step_time;
  w = delta_theta / step_time;

  odom_vel[0] = v_x;
  odom_vel[1] = v_y;
  odom_vel[2] = w;

  last_velocity[LEFT_REAR]  = wheel_left_rear / step_time;
  last_velocity[RIGHT_REAR] = wheel_right_rear / step_time;
  last_velocity[LEFT_FRONT]  = wheel_left_front / step_time;
  last_velocity[RIGHT_FRONT] = wheel_right_front / step_time;
  last_theta = theta;

  return true;
}


void driveTest(uint8_t buttons)
{
  static bool move[2] = {false, false};
  static int32_t saved_tick[2] = {0, 0};
  static double diff_encoder = 0.0;
  int32_t current_tick[2] = {0, 0};
  motor_driver.readEncoder(current_tick[LEFT_REAR], current_tick[RIGHT_REAR],current_tick[LEFT_FRONT], current_tick[RIGHT_FRONT]);
  if (buttons & (1<<0))
  {
    move[LINEAR_X] = true;
    saved_tick[RIGHT_REAR] = current_tick[RIGHT_REAR];
    diff_encoder = TEST_DISTANCE / (0.207 / 4096); // (Circumference of Wheel) / (The number of tick per revolution)
    tTime[6] = millis();
  }
  else if (buttons & (1<<1))
  {
    move[ANGULAR] = true;
    saved_tick[RIGHT_REAR] = current_tick[RIGHT_REAR];

    diff_encoder = (TEST_RADIAN * TURNING_RADIUS) / (0.207 / 4096);
    tTime[6] = millis();
  }

  if (move[LINEAR_X])
  {
    if (abs(saved_tick[RIGHT_REAR] - current_tick[RIGHT_REAR]) <= diff_encoder)
    {
      goal_velocity_from_button[LINEAR_Y]  = 0.05;
      tTime[6] = millis();
      goal_velocity_from_button[LINEAR_Y]  = -0.05;
      tTime[6] = millis();
    }
    else
    {
      goal_velocity_from_button[LINEAR_Y]  = 0.0;
      move[LINEAR_X] = false;
    }
  }
  else if (move[ANGULAR])
  {
    if (abs(saved_tick[RIGHT_REAR] - current_tick[RIGHT_REAR]) <= diff_encoder)
    {
      goal_velocity_from_button[ANGULAR]= 0.7;
      tTime[6] = millis();
      goal_velocity_from_button[ANGULAR]= -0.7;
      tTime[6] = millis();
    }
    else
    {
      goal_velocity_from_button[ANGULAR]  = 0.0;
      move[ANGULAR] = false;
    }
  }
}

/*******************************************************************************
* Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;
  if (isConnected)  {
    if (variable_flag == false)    {
      sensors.initIMU();
      initOdom();
      variable_flag = true;
    }
  }
  else{variable_flag = false;}
}
/*******************************************************************************
* Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  if (isConnected){
    if (wait_flag == false){
      delay(10);
      wait_flag = true;
    }
  } else  {wait_flag = false;}
}

/*******************************************************************************
* Update the base time for interpolation
*******************************************************************************/
void updateTime(){
  current_offset = millis();
  current_time = nh.now();
}
/*******************************************************************************
* ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow(){return nh.now();}
/*******************************************************************************
* Time Interpolation function (deprecated)
*******************************************************************************/
ros::Time addMicros(ros::Time & t, uint32_t _micros){
  uint32_t sec, nsec;
  sec  = _micros / 1000 + t.sec;
  nsec = _micros % 1000000000 + t.nsec;
  return ros::Time(sec, nsec);
}
/*******************************************************************************
* Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected){
  static bool isEnded = false;
  char log_msg[50];
  (void)(isConnected);
  if (nh.connected()){
    if (isEnded == false){
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);
      sensors.calibrationGyro();
      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);
      isEnded = true;
    }
  }
  else{isEnded = false;}
}

/*******************************************************************************
* Send log message
*******************************************************************************/
void sendLogMsg(void)
{
  static bool log_flag = false;
  char log_msg[100];
  String name             = NAME;
  String firmware_version = FIRMWARE_VER;
  String bringup_log      = "This core(v" + firmware_version + ") is compatible with TB3 " + name;
  const char* init_log_data = bringup_log.c_str();
  if (nh.connected()){
    if (log_flag == false){
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);
      sprintf(log_msg, "Connected to OpenCR board!");
      nh.loginfo(log_msg);
      sprintf(log_msg, init_log_data);
      nh.loginfo(log_msg);
      sprintf(log_msg, "--------------------------");
      nh.loginfo(log_msg);

      nh.loginfo(debug_buf);
      log_flag = true;
    }
  }
  else{log_flag = false;}
}

/*******************************************************************************
* Initialization odometry data
*******************************************************************************/
void initOdom(void)
{
  init_encoder = true;
  for (int index = 0; index < 3; index++)  {odom_pose[index] = 0.0;odom_vel[index]  = 0.0;}
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;
  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.linear.y  = 0.0;
  odom.twist.twist.angular.z = 0.0;
}

/*******************************************************************************
* Initialization joint states data
*******************************************************************************/
void initJointStates(void)
{
  static char *joint_states_name[] = {(char*)"wheel_left_rear_joint", (char*)"wheel_right_rear_joint", (char*)"wheel_left_front_joint", (char*)"wheel_right_front_joint"};       ////수정필요
  joint_states.header.frame_id = joint_state_header_frame_id;
  joint_states.name            = joint_states_name;
  joint_states.name_length     = WHEEL_NUM;
  joint_states.position_length = WHEEL_NUM;
  joint_states.velocity_length = WHEEL_NUM;
  joint_states.effort_length   = WHEEL_NUM;
}

/*******************************************************************************
* Update Goal Velocity
*******************************************************************************/              ////수정필요  goal_velocity  goal_velocity_from_button  goal_velocity_from_cmd  goal_velocity_from_rc100 LINEAR X,Y 분리
void updateGoalVelocity(void)
{
  goal_velocity[LINEAR_X]  = goal_velocity_from_button[LINEAR_X]  + goal_velocity_from_cmd[LINEAR_X]  + goal_velocity_from_rc100[LINEAR_X];
  goal_velocity[LINEAR_Y]  = goal_velocity_from_button[LINEAR_Y]  + goal_velocity_from_cmd[LINEAR_Y]  + goal_velocity_from_rc100[LINEAR_Y];
  goal_velocity[ANGULAR] = goal_velocity_from_button[ANGULAR] + goal_velocity_from_cmd[ANGULAR] + goal_velocity_from_rc100[ANGULAR];
  sensors.setLedPattern(goal_velocity[LINEAR_X], goal_velocity[ANGULAR]);         ///////////////////////수정해야되는데 우짜누
}

/*******************************************************************************
* Send Debug data
*******************************************************************************/
void sendDebuglog(void)
{
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("EXTERNAL SENSORS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.print("Bumper : "); DEBUG_SERIAL.println(sensors.checkPushBumper());
  DEBUG_SERIAL.print("Cliff : "); DEBUG_SERIAL.println(sensors.getIRsensorData());
  DEBUG_SERIAL.print("Sonar : "); DEBUG_SERIAL.println(sensors.getSonarData());
  DEBUG_SERIAL.print("Illumination : "); DEBUG_SERIAL.println(sensors.getIlluminationData());

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("OpenCR SENSORS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.print("Battery : "); DEBUG_SERIAL.println(sensors.checkVoltage());
  DEBUG_SERIAL.println("Button : " + String(sensors.checkPushButton()));

  float* quat = sensors.getOrientation();

  DEBUG_SERIAL.println("IMU : ");
  DEBUG_SERIAL.print("    w : "); DEBUG_SERIAL.println(quat[0]);
  DEBUG_SERIAL.print("    x : "); DEBUG_SERIAL.println(quat[1]);
  DEBUG_SERIAL.print("    y : "); DEBUG_SERIAL.println(quat[2]);
  DEBUG_SERIAL.print("    z : "); DEBUG_SERIAL.println(quat[3]);

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("DYNAMIXELS");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Torque : " + String(motor_driver.getTorque()));

  int32_t encoder[WHEEL_NUM] = {0, 0};

  motor_driver.readEncoder(encoder[LEFT_REAR], encoder[RIGHT_REAR], encoder[LEFT_FRONT], encoder[RIGHT_FRONT]);        ////  motor_driver.readEncoder(encoder[LEFT], encoder[RIGHT]);        ////수정필요 rear front 추가//수정완료

  DEBUG_SERIAL.println("Encoder(left_rear) : " + String(encoder[LEFT_REAR]));     ////  DEBUG_SERIAL.println("Encoder(left) : " + String(encoder[LEFT]));
  DEBUG_SERIAL.println("Encoder(right_rear) : " + String(encoder[RIGHT_REAR]));   ////  DEBUG_SERIAL.println("Encoder(right) : " + String(encoder[RIGHT]));   ////여기까지
  DEBUG_SERIAL.println("Encoder(left_front) : " + String(encoder[LEFT_FRONT]));
  DEBUG_SERIAL.println("Encoder(right_front) : " + String(encoder[RIGHT_FRONT]));

  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("TurtleBot3");
  DEBUG_SERIAL.println("---------------------------------------");
  DEBUG_SERIAL.println("Odometry : ");
  DEBUG_SERIAL.print("         x : "); DEBUG_SERIAL.println(odom_pose[0]);
  DEBUG_SERIAL.print("         y : "); DEBUG_SERIAL.println(odom_pose[1]);
  DEBUG_SERIAL.print("     theta : "); DEBUG_SERIAL.println(odom_pose[2]);
}
