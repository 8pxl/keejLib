#include "../include/keejLib/lib.h"

std::vector<double> lib::chassis::asymTrapezoidalProfile(double dist, double maxSpeed, double accel, double decel)
{
  double max = std::min(std::sqrt((2 * accel * decel * dist) / accel + decel), maxSpeed);
  double accelTime = max / accel;
  double decelTime = max / decel;
  double coastDist = (dist / max) - (max / (2 * accel)) - (max / (2*decel));
  double coastTime = coastDist / max;
  double totalTime = accelTime + decelTime + coastTime;
  double vel = 0;
  double diff;
  std::vector<double> profile;
  for (int i = 0; i < std::ceil(totalTime); i++)
  {
    if (i < std::floor(accelTime))
    {
      profile.push_back(vel);
      vel += accel;
    }

    else if (i < coastTime + accelTime)
    {
      profile.push_back(max);
    }

    else
    {
      profile.push_back(vel);
      vel -= decel;
    }
  }
  return profile;
}

void lib::chassis::profiledDrive(double target, int endDelay = 500)
{
  //kv: rpm -> voltage
  //sf: in/ms -> rpm
  int sign = lib::sign(target);
  target = fabs(target);
  std::vector<double> profile;
  if(sign > 0) profile = asymTrapezoidalProfile(target, constants.maxSpeed, constants.fwdAccel, constants.fwdDecel);
  else profile = asymTrapezoidalProfile(target, constants.maxSpeed, constants.revAccel, constants.revDecel);
  chass -> reset();
  for (int i = 0; i < profile.size(); i++)
  {
    chass -> spin(profile[i] * constants.ip10mstomvolt * sign);
    pros::delay(10);
  }
  chass -> stop('b');
  pros::delay(endDelay);
}