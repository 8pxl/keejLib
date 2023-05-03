#pragma once
#include "../include/keejLib/lib.h"

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

std::vector<double> lib::chassis::pidMTPVel(lib::coordinate target, double rotationBias, lib::pid* lCont, lib::pid* rCont)
{
    double linearError = dist(pos,target);
    double currHeading = imu -> get_heading();
    double targetHeading = absoluteAngleToPoint(pos, target);
    double rotationError = lib::minError(targetHeading,currHeading);
    double cre = abs(rotationError) > 90 ? 0.1 : cos(lib::dtr(rotationError));
    double angularVel = rCont -> out(rotationError);
    double linearVel = cre * lCont -> out(linearError);
    double rVel = (linearVel - (fabs(angularVel) * rotationBias)) + angularVel;
    double lVel = (linearVel - (fabs(angularVel) * rotationBias)) - angularVel;
    return(std::vector<double> {rVel, lVel});
} 

void lib::chassis::pidMoveTo(lib::coordinate target, double timeout, lib::pidConstants lConstants, lib::pidConstants rConstants, double rotationBias)
{
  lib::timer timeoutTimer;
  lib::pid linearController(lConstants, 0);
  lib::pid rotationController(rConstants, 0);

  while (timeoutTimer.time() < timeout) chass -> spinDiffy(lib::chassis::pidMTPVel(target, rotationBias, &linearController, &rotationController));
  chass -> stop('b');
}


void lib::chassis::boomerang(lib::coordinate target, double timeout, double dLead, double thetaEnd, double rotationBias, lib::pidConstants lConstants, lib::pidConstants rConstants)
{
  lib::timer timeoutTimer;
  lib::pid linearController(lConstants, 0);
  lib::pid rotationController(rConstants, 0);

  while (timeoutTimer.time() < timeout)
  {
    double h = hypot(pos.x - target.x, pos.y - target.y);
    lib::coordinate carrot = {target.x - (h * sin(thetaEnd) * dLead), target.y - (h * cos(thetaEnd) * dLead)};
    chass -> spinDiffy(lib::chassis::pidMTPVel(carrot, rotationBias, &linearController, &rotationController));
  }
}