#include "mw_ahrsv1_ros2/MW_AHRSv1.hpp"

MW_AHRSv1::MW_AHRSv1(const std::string& device_name, const uint& baud_rate)
      : device_name(device_name), baud_rate(baud_rate){

  for (size_t i = 0; i < sizeof(buffer); i++)
    buffer[i] = 0;
}

void MW_AHRSv1::start_serial() {
  // int open_serial(char *dev_name, int baud, int vtime, int vmin);
  serial_id =
      open_serial(const_cast<char *>(device_name.c_str()), 115200, 0, 0);
}

void MW_AHRSv1::reset_angle() {
  write(serial_id, reset_angle_cmd, 5);
  read(serial_id, &buffer, sizeof(buffer));

  buffer[sizeof(buffer)] = '\0';

  if ((int)buffer[0] != 0)
    std::cout << "IMU Angle Reset Done..." << std::endl;
}

void MW_AHRSv1::reset_imu() {
  write(serial_id, reset_cmd, 5);
  read(serial_id, &buffer, sizeof(buffer));

  buffer[sizeof(buffer)] = '\0';

  if ((int)buffer[0] != 0)
    std::cout << "IMU Reset Done..." << std::endl;
}

void MW_AHRSv1::speed_setup() {
  write(serial_id, speed_cmd, 8);
  read(serial_id, &buffer, sizeof(buffer));

  buffer[sizeof(buffer)] = '\0';

  if ((int)buffer[0] != 0)
    std::cout << "Speed Setup Done..." << std::endl;
}

void MW_AHRSv1::start_data_stream() {
  write(serial_id, ros_data_cmd, 6);
  read(serial_id, &buffer, sizeof(buffer));

  buffer[sizeof(buffer)] = '\0';
}

void MW_AHRSv1::get_angle_data(IMUMsg &msg_in) {
  write(serial_id, angle_cmd, 5);
  read(serial_id, &buffer, sizeof(buffer));

  if (buffer[0] == 'a' && buffer[1] == 'n' && buffer[2] == 'g') {
    char *rest;
    char *token;
    char *ptr = buffer;

    ang_count = 0;

    while ((token = strtok_r(ptr, " ", &rest))) {
      ang_count++;

      // ang=   -3.59    -0.61    -0.26
      if (ang_count == 2) {
        msg_in.roll = atof(token) * -convertor_d2r;
      } else if (ang_count == 3) {
        msg_in.pitch = atof(token) * -convertor_d2r;
      } else if (ang_count == 4) {
        msg_in.yaw = atof(token) * -convertor_d2r;
      }
      ptr = rest;
    }
  }
}

void MW_AHRSv1::parse_ss_data(IMUMsg &msg_in) {
  read(serial_id, &buffer, sizeof(buffer));

  if (int(buffer[0]) < 97) {
    char *rest;
    char *token;
    char *ptr = buffer;

    ang_count = 0;

    while ((token = strtok_r(ptr, " ", &rest))) {
      ang_count++;

      if (ang_count == 1) {
        msg_in.linear_acc_x = atof(token);
      } else if (ang_count == 2) {
        msg_in.linear_acc_y = atof(token);
      } else if (ang_count == 3) {
        msg_in.linear_acc_z = atof(token);
      } else if (ang_count == 4) {
        msg_in.angular_vel_x = atof(token);
      } else if (ang_count == 5) {
        msg_in.angular_vel_y = atof(token);
      } else if (ang_count == 6) {
        msg_in.angular_vel_z = atof(token);
      } else if (ang_count == 7) {
        msg_in.roll = atof(token) * -convertor_d2r;
      } else if (ang_count == 8) {
        msg_in.pitch = atof(token) * -convertor_d2r;
      } else if (ang_count == 9) {
        msg_in.yaw = atof(token) * -convertor_d2r;
      }
      ptr = rest;
    }
  }
}

MW_AHRSv1::~MW_AHRSv1() { close_serial(serial_id); }
