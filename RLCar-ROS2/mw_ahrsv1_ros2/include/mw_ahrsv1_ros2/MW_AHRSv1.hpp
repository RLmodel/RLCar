#pragma once

#include <tuple>
#include <iostream>
#include <string>
#include <unistd.h>

#include "mw_ahrsv1_ros2/Serial.h"
#include "mw_ahrsv1_ros2/Const.hpp"

typedef struct {
  // ang
  float roll;
  float pitch;
  float yaw;

  // acc
  float linear_acc_x;
  float linear_acc_y;
  float linear_acc_z;

  // gyr
  float angular_vel_x;
  float angular_vel_y;
  float angular_vel_z;
} IMUMsg;

class MW_AHRSv1 {

private:
  // Device Name
  int serial_id = 0;

  // Data buffer
  char buffer[120];
  char small_buffer[10];

  // Serperate Euler Angle Variable
  uint ang_count = 0;
  
  std::string device_name = "/dev/MWAHRs";
  
  uint baud_rate = 115200;

public:
  MW_AHRSv1(const std::string& device_name = "/dev/MWAHRs", const uint& baud_rate = 115200);

  void set_device_name(const std::string& device_name){
    this->device_name = device_name;
  }

  void set_baud_rate(const int& baud_rate){
    this->baud_rate = baud_rate;
  }

  void start_serial();

  void reset_angle();

  void reset_imu();

  void speed_setup();

  void start_data_stream();

  void get_angle_data(IMUMsg &msg_in);

  void parse_ss_data(IMUMsg &msg_in);

  ~MW_AHRSv1();
};
