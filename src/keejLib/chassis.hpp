#pragma once

#include "lib.h"

lib::chassis::chassis(lib::diffy& mtrs, pros::Imu& imu, std::vector<int> encoderPorts, lib::robotConstants constants)
{
  chass = &mtrs;
  this -> imu = &imu;
  this -> constants = constants;
  
  auto temp1 = pros::ADIEncoder(std::abs(encoderPorts[0]), std::abs(encoderPorts[0]) + 1, encoderPorts[0] < 0 ? true : false);
  auto temp2 = pros::ADIEncoder(std::abs(encoderPorts[1]), std::abs(encoderPorts[1]) + 1, encoderPorts[1] < 0 ? true : false);
  horizTracker = &temp1;
  vertTracker = &temp2;
}

void lib::chassis::drive(double target, double timeout, lib::pidConstants constants)
{

  double error = target;
  lib::timer timer;
  lib::pid pidController(constants, target);
  chass -> reset();

  while(timer.time() < timeout)
  {
    error = target - chass -> getRotation();
    chass -> spin(pidController.out(error));
  }

  chass -> stop('b');
}
