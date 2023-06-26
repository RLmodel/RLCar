#pragma once

#include <math.h>

// utils for serial cmd
const unsigned char angle_cmd[5] = {0x61, 0x6E, 0x67, 0x0D, 0x0A};       // ang Enter
const unsigned char gyr_cmd[5] = {0x67, 0x79, 0x72, 0x0D, 0x0A};         // ang Enter
const unsigned char acc_cmd[5] = {0x61, 0x63, 0x63, 0x0D, 0x0A};         // ang Enter
const unsigned char reset_angle_cmd[5] = {0x7A, 0x72, 0x6F, 0x0D, 0x0A}; // zro Enter
const unsigned char reset_cmd[5] = {0x7A, 0x72, 0x6F, 0x0D, 0x0A};       // zro Enter
const unsigned char av_cmd[7] = {0x61, 0x76, 0x3D, 0x31, 0x30, 0x0D, 0x0A}; // av = 10
const unsigned char speed_cmd[8] = {0x73, 0x70, 0x3D, 0x31,
                              0x30, 0x30, 0x0D, 0x0A}; // sp=100 Enter
const unsigned char ros_data_cmd[6] = {0x73, 0x73, 0x3D,
                                 0x37, 0x0D, 0x0A}; // ss=7 Enter

// Unit converting constants
const double convertor_g2a = 9.80665; // for linear_acceleration (g to m/s^2)
const double convertor_d2r =
    M_PI / 180.0; // for angular_velocity (degree to radian)
const double convertor_r2d =
    180.0 / M_PI; // for easy understanding (radian to degree)
const double convertor_ut2t = 1000000; // for magnetic_field (uT to Tesla)
const double convertor_c = 1.0;        // for temperature (celsius)
