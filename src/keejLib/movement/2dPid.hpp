#pragma once
#include "lib.h"

void lib::chassis::driveAngle(double target, double heading, double timeout, lib::pidConstants lCons, lib::pidConstants acons)
{
  lib::timer timer;

  double currHeading = imu -> get_heading();
  double sgn = sign(target);
  lib::pid linearController(lCons, 0);
  lib::pid angularController(acons,target);

  chass -> reset();

  while (timer.time() <= timeout)
  {
    currHeading = imu -> get_heading();
    double angularError = lib::minError(heading, currHeading);

    if (angularError < acons.tolerance)
    {
        angularError = 0;
    }

    double va = angularController.out(angularError);
    double vl = linearController.out(target - chass -> getRotation());

    if (vl + std::abs(va) > 127)
    {
      vl = 127 - std::abs(va);
    }

    chass -> spinDiffy(vl + (va * sgn),  vl - (va * sgn));
    pros::delay(10);
  }
  chass -> stop('b');
}

void lib::chassis::pidMoveTo(lib::coordinate target, double timeout, lib::pidConstants lConstants, lib::pidConstants rConstants, double rotationBias)
{
  //init
  lib::timer timeoutTimer;
  lib::pid linearController(lConstants, 0);
  lib::pid rotationController(rConstants, 0);

  while (timeoutTimer.time() < timeout)
  {
    double linearError = dist(pos,target);

    double currHeading = imu -> get_heading();
    double targetHeading = absoluteAngleToPoint(pos, target);
    double rotationError = lib::minError(targetHeading,currHeading);
    double cre = abs(rotationError) > 90 ? 0.1 : cos(lib::dtr(rotationError));

    double angularVel = rotationController.out(rotationError);
    double linearVel = cre * linearController.out(linearError);

    double rVel = (linearVel - (fabs(angularVel) * rotationBias)) + angularVel;
    double lVel = (linearVel - (fabs(angularVel) * rotationBias)) - angularVel;

    chass -> spinDiffy(rVel,lVel);
  }
  chass -> stop('b');
}