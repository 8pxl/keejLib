#pragma once
#include "../include/keejLib/lib.h"

void lib::chassis::updatePos()
{
  double currRotation = imu -> get_heading();
  double deltaRotation = lib::minError(prevRotation, currRotation);

  prevRotation = currRotation;

  deltaRotation = lib::dtr(deltaRotation);
  currRotation = lib::dtr(currRotation);

  double deltaVert = (constants.trackDia / 360) * vertTracker -> get_value();
  double deltaHoriz = (constants.trackDia / 360) * horizTracker -> get_value();
  double deltaY, deltaX, localX, localY, pAngle, pRadius;

  if (deltaRotation == 0)
  {
    localX = deltaHoriz;
    localY = deltaVert;
  }

  else
  {
    localX = (2 * sin(deltaRotation / 2)) * ((deltaHoriz / deltaRotation) + constants.horizTrack);
    localY = (2 * sin(deltaRotation / 2)) * ((deltaVert / deltaRotation) + constants.vertTrack);
  }

  if (localX == 0 && localY == 0)
  {
    pAngle = 0;
    pRadius = 0;
  } 

  else 
  {
    pAngle = atan2(localY, localX); 
    pRadius = hypot(localX, localY);
  }

  pAngle = pAngle - lib::dtr(prevRotation) - (deltaRotation / 2);
  deltaX = pRadius * cos(pAngle);
  deltaY = pRadius * sin(pAngle);

  pos = {pos.x + deltaX, pos.y + deltaY};

  // reset encoders
  horizTracker -> reset();
  vertTracker -> reset();
}

void lib::chassis::initTracking()
{
  odomTask = new pros::Task {[=] 
  {
    while (true) 
    {
      updatePos();
      pros::delay(10);
    }
  }};
}