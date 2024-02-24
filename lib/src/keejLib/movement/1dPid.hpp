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
    chass -> spinDiffy(-vel, vel);
    pros::delay(10);
  }
  chass -> stop(brake);
  return (pidController);
}

// lib::pid lib::chassis::keejTurn(double target, int timeout, lib::pidConstants constants, char brake = 'b')
// {
//   lib::timer timer;
//   lib::pid pidController(constants, target);
//   int state = 1;
//   double vel = 0;
//   while (timer.time() < timeout)
//   {
//     double error = lib::minError(target, imu -> get_heading());
//     switch (state) {
//       case 1:
//         if (vel >= angular.maxSpeed) {
//           vel = angular.maxSpeed;
//           state = 2;
//         }
//       case 2:
        
//       case 3:
//     }
//   }
//   return pidController;
//   /*
//     thought process,
//     split it into three states. 
//     case 1: if there is distance to deccelerate from current velocity, accelerate, if current velocity exceeds the max, set it to the max
//     case 2: if there isn't distance to deccelerate from current velocity, then decelerate otherwise max. 
//     case 3: decelerate, pid correction aat the very end
//   */
// }

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
void lib::chassis::arcTurn(double target, double radius, double timeout, int dir, lib::pidConstants constants, double min = 0, int endTime = 0,char brake = 'b')
{
  lib::timer timer;
  lib::timer end;

  double curr = imu -> get_heading();
  double theta = lib::dtr(lib::minError(target, curr));
  double sl = theta * (radius + this -> constants.vertTrack);
  double sr = theta * (radius - this -> constants.vertTrack);
  double ratio = sl/sr;
  lib::pid controller(constants, 0);
  int last = lib::sign(controller.out(lib::minError(target, curr)));
  bool ending = false;

  while (timer.time() < timeout)
  {
    curr = imu -> get_heading();
    // double temp = lib::minError(target, curr);
    // std::cout << temp << std::endl;
    double vel = controller.out(lib::minError(target, curr));
    vel = std::max(std::abs(vel), min) * lib::sign(vel);
    double rvel = (2 * vel) / (ratio+1);
    rvel = std::abs(rvel) >= 127 ? (127 * lib::sign(rvel)) : rvel;
    double lvel = ratio * rvel;

    if(lib::sign(dir) == 1) chass -> spinDiffy(rvel, lvel);
    else chass -> spinDiffy(-lvel, -rvel);
    if((lib::sign(vel) != last && min > 0) || ending) {
      ending = true;
      if(end.time() > endTime) {
        return;
      }
    }
    else {
      end.reset();
    }
    pros::delay(10);
  }
  chass -> stop(brake);
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
    double sl = theta * (1/curvature + this -> constants.horizTrack);
    double sr = theta * (1/curvature - this -> constants.horizTrack);
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