#pragma once
#include "../include/keejLib/lib.h"

lib::pid lib::chassis::pidDrive(double target, double timeout, lib::pidConstants constants, char brake = 'b')
{
  lib::timer timer;
  lib::pid pidController(constants, target);
  chass -> reset();
  while(timer.time() < timeout) chass -> spin(pidController.out(target - chass -> getRotation()));
  chass -> stop(brake);
  return (pidController);
}

lib::pid lib::chassis::pidTurn(double target, double timeout, lib::pidConstants constants, char brake = 'b')
{
  lib::timer timer;
  lib::pid pidController(constants, target);
  while(timer.time() < timeout)
  {
    double vel = pidController.out(lib::minError(target, imu -> get_heading()));
    chass -> spinDiffy(vel, -vel);
  }
  chass -> stop(brake);
  return (pidController);
}

lib::pid lib::chassis::pidDrive(double target, double timeout, char brake, lib::pid cont)
{
  lib::timer timer;
  chass -> reset();
  while(timer.time() < timeout) chass -> spin(cont.out(target - chass -> getRotation()));
  chass -> stop(brake);
  return (cont);
}

lib::pid lib::chassis::pidTurn(double target, double timeout, char brake, lib::pid cont)
{
  lib::timer timer;
  while(timer.time() < timeout)
  {
    double vel = cont.out(lib::minError(target, imu -> get_heading()));
    chass -> spinDiffy(vel, -vel);
  }
  chass -> stop(brake);
  return (cont);
}

//TODO: make radius actual units lol
void lib::chassis::arcTurn(double target, double radius, double timeout, int dir, lib::pidConstants constants)
{
  lib::timer timer;

  double curr = imu -> get_heading();
  double theta = lib::dtr(lib::minError(target, curr));
  double sl = theta * (radius + this -> constants.vertTrack);
  double sr = theta * (radius - this -> constants.vertTrack);

  double ratio = sl/sr;
  lib::pid controller(constants, 0);

  while (timer.time() < timeout)
  {
    curr = imu -> get_heading();
    double vel = controller.out(lib::minError(target, curr));
    double rvel = (2 * vel) / (ratio+1);
    rvel = std::abs(rvel) >= 127 ? (127 * lib::sign(rvel)) : rvel;
    double lvel = ratio * rvel;

    if(lib::sign(dir) == 1) chass -> spinDiffy(rvel, lvel);
    else chass -> spinDiffy(-lvel, -rvel);

    pros::delay(10);
  }
  chass -> stop('b');
}

void lib::chassis::eulerTurn(double theta, double rate, double timeout, int dir, lib::pidConstants constants)
{
  double curvature = 0;
  lib::timer timer;
  lib::pid controller(constants, 0);

  while(timer.time() < timeout)
  {
    curvature += rate;
    double curr = imu -> get_heading();
    double sl = theta * (1/curvature + this -> constants.vertTrack);
    double sr = theta * (1/curvature - this -> constants.vertTrack);
    double ratio = sl/sr;

    double vel = controller.out(lib::minError(theta, curr));
    vel = std::abs(vel) >= 127 ? (127 * lib::sign(vel)) : vel;
    double rvel = (2 * vel) / (ratio+1);
    double lvel = ratio * rvel;

    if(lib::sign(dir) == 1) chass -> spinDiffy(rvel, lvel);
    else chass -> spinDiffy(-lvel, -rvel);

    pros::delay(10);
  }
  chass -> stop('b');
}