#pragma once

#include "../include/keejLib/lib.h"

lib::chassis::chassis(lib::diffy& mtrs, pros::Imu& imu, std::vector<int> encoderPorts, lib::robotConstants constants, lib::accelConstants linear, lib::accelConstants angular)
{
  chass = &mtrs;
  this -> imu = &imu;
  this -> constants = constants;
  this -> linear = linear;
  this -> angular = angular;
  
  // auto temp1 = pros::ADIEncoder(std::abs(encoderPorts[0]), std::abs(encoderPorts[0]) + 1, encoderPorts[0] < 0 ? true : false);
  // auto temp2 = pros::ADIEncoder(std::abs(encoderPorts[1]), std::abs(encoderPorts[1]) + 1, encoderPorts[1] < 0 ? true : false);
  // horizTracker = &temp1;
  // vertTracker = &temp2;
} 