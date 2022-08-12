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

#ifndef TURTLEBOT3_MOTOR_DRIVER_H_
#define TURTLEBOT3_MOTOR_DRIVER_H_

#include "variant.h"
#include "math.h"
#include <DynamixelSDK.h>
#include <DynamixelWorkbench.h>

// use for DynamixelWorkbench init
#define DEVICE_NAME ""

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE            64
#define ADDR_X_GOAL_CURRENT             102
#define ADDR_X_GOAL_VELOCITY            104
#define ADDR_X_GOAL_POSITION            116
#define ADDR_X_REALTIME_TICK            120
#define ADDR_X_PRESENT_VELOCITY         128
#define ADDR_X_PRESENT_POSITION         132

// Limit values (XM430-W210-T and XM430-W350-T)
#define BURGER_DXL_LIMIT_MAX_VELOCITY            265     // MAX RPM is 61 when XL is powered 12.0V
#define WAFFLE_DXL_LIMIT_MAX_VELOCITY            330     // MAX RPM is 77 when XM is powered 12.0V

// Data Byte Length
#define LEN_X_TORQUE_ENABLE             1
#define LEN_X_GOAL_VELOCITY             4
#define LEN_X_GOAL_POSITION             4
#define LEN_X_GOAL_CURRENT              2
#define LEN_X_REALTIME_TICK             2
#define LEN_X_PRESENT_VELOCITY          4
#define LEN_X_PRESENT_POSITION          4

#define PROTOCOL_VERSION                2.0     // Dynamixel protocol version 2.0


#define DXL_LEFT_FRONT_ID               1       // ID of left front motor
#define DXL_RIGHT_FRONT_ID              2       // ID of right front motor
#define DXL_LEFT_REAR_ID                3       // ID of left rear motor
#define DXL_RIGHT_REAR_ID               4       // ID of right rear motor

#define DXL_ARM1_ID                     11       // ID of Left sholder
#define DXL_ARM2_ID                     12       // ID of Left uppper arm
#define DXL_ARM3_ID                     13       // ID of Left elbow
#define DXL_ARM4_ID                     14       // ID of Left lower arm
#define DXL_ARM5_ID                     15       // ID of Right sholder
#define DXL_ARM6_ID                     16       // ID of Right uppper arm
#define DXL_ARM7_ID                     17       // ID of Right elbow
#define DXL_ARM8_ID                     18       // ID of Right lower arm

#define BAUDRATE                        1000000 // baurd rate of Dynamixel, not used
#define DEVICENAME                      ""      // no need setting on OpenCR

#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define LEFT	                        0
#define RIGHT	                        1

#define LEFT_REAR                       0
#define RIGHT_REAR                      1
#define LEFT_FRONT                      2
#define RIGHT_FRONT                     3

#define LINEAR_X                        0
#define LINEAR_Y                        1
#define ANGULAR                         2


#define VELOCITY_CONSTANT_VALUE         41.69988758  // V = r * w = r     *        (RPM             * 0.10472)
                                                     //           = r     * (0.229 * Goal_Velocity) * 0.10472
                                                     //
                                                     // Goal_Velocity = V / r * 41.69988757710309
#define POSITION_CONSTANT_VALUE         651.898647649 //2pi rad : 4096 clicks


#define DEBUG_SERIAL  SerialBT2

class Turtlebot3MotorDriver
{
 public:
  Turtlebot3MotorDriver();
  ~Turtlebot3MotorDriver();
  bool init(String turtlebot3, int baudrate, char* debug_buf);
  void close(void);
  bool setTorque(bool onoff);
  bool getTorque();
  bool readEncoder(int32_t &left_rear_value, int32_t &right_rear_value, int32_t& left_front_value, int32_t& right_front_value);
  bool writeVelocity(int64_t left_rear_value, int64_t right_rear_value, int64_t left_front_value, int64_t right_front_value);
  bool controlMotorMecanum(const float wheel_radius, const float wheel_separation, float* value);
  bool controlArm(int32_t* arm_target_pos);
  
 private:
  uint32_t baudrate_;
  float  protocol_version_;
  uint8_t left_rear_wheel_id_, right_rear_wheel_id_;
  uint8_t left_front_wheel_id_, right_front_wheel_id_;
  bool torque_;

  uint16_t dynamixel_limit_max_velocity_;

  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
  dynamixel::GroupSyncRead *groupSyncReadEncoder_;
  
  DynamixelWorkbench dxl_wb;
  const char *log;
  bool result = false;
  uint16_t model_number = 0;
  uint8_t dxl_arm_id[8] = {DXL_ARM1_ID, DXL_ARM2_ID, DXL_ARM3_ID, DXL_ARM4_ID, DXL_ARM5_ID, DXL_ARM6_ID, DXL_ARM7_ID, DXL_ARM8_ID};
};

#endif // TURTLEBOT3_MOTOR_DRIVER_H_
