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

/* Authors: Yoonseok Pyo, Leon Jung, Darby Lim, HanCheol Cho */

//MODIFIED FOR MECANUM MOTOR SETUP


#include "../../include/turtlebot3_mecanum_assy/turtlebot3_mecanum_assy_motor_driver.h"

Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION),
  left_rear_wheel_id_(DXL_LEFT_REAR_ID), right_rear_wheel_id_(DXL_RIGHT_REAR_ID),
  left_front_wheel_id_(DXL_LEFT_FRONT_ID), right_front_wheel_id_(DXL_RIGHT_FRONT_ID)

{
  torque_ = false;
  dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver(){  close();}
bool Turtlebot3MotorDriver::init(String turtlebot3, int baudrate, char* debug_buf){
  DEBUG_SERIAL.begin(57600);
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort() == false)  {
    DEBUG_SERIAL.println("Failed to open port(Motor Driver)");
    sprintf(debug_buf,"motordriver: failed to open port");
    return false;
  }
  // Set port baudrate
  // if (portHandler_->setBaudRate(baudrate_) == false)  {
  if (portHandler_->setBaudRate(baudrate) == false)  {
    sprintf(debug_buf,"motordriver: failed to set baud rate");
    DEBUG_SERIAL.println("Failed to set baud rate(Motor Driver)");
    return false;
  }
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
  setTorque(true);  // Enable Dynamixel Torque 바퀴만 토크 걸어주는 부분
  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
  groupSyncReadEncoder_   = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  
  if (turtlebot3 == "Burger") dynamixel_limit_max_velocity_ = BURGER_DXL_LIMIT_MAX_VELOCITY;
  else if (turtlebot3 == "Waffle or Waffle Pi") dynamixel_limit_max_velocity_ = WAFFLE_DXL_LIMIT_MAX_VELOCITY;
  else dynamixel_limit_max_velocity_ = WAFFLE_DXL_LIMIT_MAX_VELOCITY;
  DEBUG_SERIAL.println("Success to init Motor Driver");
  sprintf(debug_buf,"motordriver: Success to init motor driver");
  return true;
}

// 여기에서 바퀴, 로봇암, 그리퍼 전부 토크 걸어준다.
// 그리퍼 부분 제거하고 로봇암 부분 workbench 스타일로 하면 될 듯.
bool Turtlebot3MotorDriver::setTorque(bool onoff){
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  torque_ = onoff;
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_LEFT_REAR_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS){
	  Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
	  return false;
  }
  else if (dxl_error != 0)  {
	  Serial.println(packetHandler_->getRxPacketError(dxl_error));
	  return false;
  }
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_RIGHT_REAR_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)  {
	  Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
	  return false;
  }
  else if (dxl_error != 0)  {
	  Serial.println(packetHandler_->getRxPacketError(dxl_error));
	  return false;
  }
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_LEFT_FRONT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)  {
	  Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
	  return false;
  }  else if (dxl_error != 0)  {
	  Serial.println(packetHandler_->getRxPacketError(dxl_error));
	  return false;
  }
  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, DXL_RIGHT_FRONT_ID, ADDR_X_TORQUE_ENABLE, onoff, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)  {
	  Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
	  return false;
  }  else if (dxl_error != 0)  {
	  Serial.println(packetHandler_->getRxPacketError(dxl_error));
	  return false;
  }
  
  // Set torque robot arm using workbench
  for (int cnt = 0; cnt < 8; cnt++){
    result = dxl_wb.ping(dxl_arm_id[cnt], &model_number, &log);
    result = dxl_wb.jointMode(dxl_arm_id[cnt], 0, 0, &log);
  }
  result = dxl_wb.initBulkWrite(&log);
  result = dxl_wb.initBulkRead(&log);
  for (int cnt = 0; cnt < 8; cnt++){
    result = dxl_wb.addBulkReadParam(dxl_arm_id[cnt], "Present_Position", &log);
  }
  
  return true;
}

bool Turtlebot3MotorDriver::getTorque(){return torque_;}

void Turtlebot3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  setTorque(false);
  // Close port
  portHandler_->closePort();
  DEBUG_SERIAL.end();
}

// 바퀴위치 읽음
bool Turtlebot3MotorDriver::readEncoder(int32_t& left_rear_value, int32_t& right_rear_value, int32_t& left_front_value, int32_t& right_front_value)
{
  int dxl_comm_result = COMM_TX_FAIL;              // Communication result
  bool dxl_addparam_result = false;                // addParam result
  bool dxl_getdata_result = false;                 // GetParam result

  // Set parameter
  dxl_addparam_result = groupSyncReadEncoder_->addParam(left_rear_wheel_id_);
  // if (dxl_addparam_result != true)	  return false;
  dxl_addparam_result = groupSyncReadEncoder_->addParam(right_rear_wheel_id_);
  // if (dxl_addparam_result != true)	  return false;
  dxl_addparam_result = groupSyncReadEncoder_->addParam(left_front_wheel_id_);
  // if (dxl_addparam_result != true)	  return false;
  dxl_addparam_result = groupSyncReadEncoder_->addParam(right_front_wheel_id_);
  // if (dxl_addparam_result != true)	  return false;

  // Syncread present position
  dxl_comm_result = groupSyncReadEncoder_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)  return false;
   // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
  // if (dxl_comm_result != COMM_SUCCESS)   Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
  // Check if groupSyncRead data of Dynamixels are available
  // dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  // if (dxl_getdata_result != true)	  return false;
  // dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  // if (dxl_getdata_result != true)	  return false;
  // dxl_getdata_result = groupSyncReadEncoder_->isAvailable(left_front_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  // if (dxl_getdata_result != true)	  return false;
  // dxl_getdata_result = groupSyncReadEncoder_->isAvailable(right_front_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  // if (dxl_getdata_result != true)	  return false;

  // Get data
  left_rear_value = groupSyncReadEncoder_->getData(left_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  right_rear_value = groupSyncReadEncoder_->getData(right_rear_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  left_front_value = groupSyncReadEncoder_->getData(left_front_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  right_front_value = groupSyncReadEncoder_->getData(right_front_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
  groupSyncReadEncoder_->clearParam();
  return true;
}

// 바퀴 속도값 명령주는 부분 인듯
bool Turtlebot3MotorDriver::writeVelocity(int64_t left_rear_value, int64_t right_rear_value, int64_t left_front_value, int64_t right_front_value)
{
  bool dxl_addparam_result;
  int8_t dxl_comm_result;

  uint8_t left_rear_data_byte[4] = { 0, };
  uint8_t right_rear_data_byte[4] = { 0, };
  uint8_t left_front_data_byte[4] = { 0, };
  uint8_t right_front_data_byte[4] = { 0, };
  left_rear_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(left_rear_value));
  left_rear_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(left_rear_value));
  left_rear_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(left_rear_value));
  left_rear_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(left_rear_value));
  dxl_addparam_result = groupSyncWriteVelocity_->addParam(left_rear_wheel_id_, (uint8_t*)& left_rear_data_byte);
  if (dxl_addparam_result != true)	  return false;
  right_rear_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(right_rear_value));
  right_rear_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(right_rear_value));
  right_rear_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(right_rear_value));
  right_rear_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(right_rear_value));
  dxl_addparam_result = groupSyncWriteVelocity_->addParam(right_rear_wheel_id_, (uint8_t*)& right_rear_data_byte);
  if (dxl_addparam_result != true)	  return false;
  left_front_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(left_front_value));
  left_front_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(left_front_value));
  left_front_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(left_front_value));
  left_front_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(left_front_value));
  dxl_addparam_result = groupSyncWriteVelocity_->addParam(left_front_wheel_id_, (uint8_t*)& left_front_data_byte);
  if (dxl_addparam_result != true)	  return false;
  right_front_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(right_front_value));
  right_front_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(right_front_value));
  right_front_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(right_front_value));
  right_front_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(right_front_value));
  dxl_addparam_result = groupSyncWriteVelocity_->addParam(right_front_wheel_id_, (uint8_t*)& right_front_data_byte);
  if (dxl_addparam_result != true)	  return false;
  dxl_comm_result = groupSyncWriteVelocity_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)  {
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
    return false;
  }
  groupSyncWriteVelocity_->clearParam();
  return true;
}

// 바퀴 속도값 명령주는 부분
bool Turtlebot3MotorDriver::controlMotorMecanum(const float wheel_radius, const float wheel_separation, float* value){
	bool dxl_comm_result = false;
	float wheel_velocity_cmd[4];

	float lin_x_vel = value[LINEAR_X];	float lin_y_vel = value[LINEAR_Y];	float ang_vel = value[ANGULAR];
  wheel_velocity_cmd[LEFT_FRONT] = (lin_x_vel - lin_y_vel - wheel_separation * ang_vel)* VELOCITY_CONSTANT_VALUE / wheel_radius;
  wheel_velocity_cmd[RIGHT_FRONT] = (lin_x_vel + lin_y_vel + wheel_separation * ang_vel)* VELOCITY_CONSTANT_VALUE / wheel_radius;
	wheel_velocity_cmd[LEFT_REAR] = (lin_x_vel + lin_y_vel - wheel_separation * ang_vel)* VELOCITY_CONSTANT_VALUE / wheel_radius;
	wheel_velocity_cmd[RIGHT_REAR] = (lin_x_vel - lin_y_vel + wheel_separation * ang_vel)* VELOCITY_CONSTANT_VALUE / wheel_radius;

  float max_velocity_cmd = std::max(
     std::max(     abs(wheel_velocity_cmd[0]),     abs(wheel_velocity_cmd[1])),
     std::max(  abs(wheel_velocity_cmd[2]),abs(wheel_velocity_cmd[3])      )
   );
  if (max_velocity_cmd>dynamixel_limit_max_velocity_){
    wheel_velocity_cmd[LEFT_FRONT]=wheel_velocity_cmd[LEFT_FRONT] * dynamixel_limit_max_velocity_/max_velocity_cmd;
    wheel_velocity_cmd[RIGHT_FRONT]=wheel_velocity_cmd[RIGHT_FRONT] * dynamixel_limit_max_velocity_/max_velocity_cmd;
    wheel_velocity_cmd[LEFT_REAR]=wheel_velocity_cmd[LEFT_REAR] * dynamixel_limit_max_velocity_/max_velocity_cmd;
    wheel_velocity_cmd[RIGHT_REAR]=wheel_velocity_cmd[RIGHT_REAR] * dynamixel_limit_max_velocity_/max_velocity_cmd;
  }
	dxl_comm_result = writeVelocity((int64_t)wheel_velocity_cmd[LEFT_REAR], (int64_t)wheel_velocity_cmd[RIGHT_REAR], (int64_t)wheel_velocity_cmd[LEFT_FRONT], (int64_t)wheel_velocity_cmd[RIGHT_FRONT]);
	if (dxl_comm_result == false)		return false;
	return true;
}

bool Turtlebot3MotorDriver::controlArm(int32_t *arm_target_pos){
	bool result = false;

  int32_t arm_current_pos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  result = dxl_wb.bulkRead(&log);
  result = dxl_wb.getBulkReadData(&arm_current_pos[0], &log);

  int32_t diff_pos[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  for (int i=0; i<8; ++i){
    diff_pos[i] = arm_target_pos[i] - arm_current_pos[i];
  }

  // Goal Position range 0 ~ 4095
  // Profile Acceleration range 0 ~ 32737
  // Profile Velocity range 0 ~ 32737
  // Profile Acceleration will not exceed 50% of Profile Velocity

  float coeff_1 = 0.1;
  float coeff_2 = 0.5;
  for (int i=1; i<=8; ++i){
    int32_t p_vel = floor(abs(coeff_1*diff_pos[i-1]/4096*32737));
    int32_t p_acc = floor(coeff_2*p_vel);
    dxl_wb.addBulkWriteParam(i, "Profile_Velocity", p_vel, &log);
    dxl_wb.addBulkWriteParam(i, "Profile_Acceleration", p_acc, &log);
  }
  result = dxl_wb.bulkWrite(&log);

  for (int i=1; i<=8; ++i){
    dxl_wb.addBulkWriteParam(i, "Goal_Position", arm_target_pos[i-1], &log);
  }
  result = dxl_wb.bulkWrite(&log);

  if (result) {
    return true;
  }
  return false;
  
}
