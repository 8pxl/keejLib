#pragma once
#include "../include/keejLib/lib.h"

// - timer
lib::timer::timer()
{
    start();
}

void lib::timer::start()
{
    startTime = pros::millis();
}

int lib::timer::time()
{
    return (pros::millis() - startTime);
}

// - pid
double lib::pid::out(double error)
{
    if(std::fabs(error) < constants.tolerance) integral = 0;
    else if(std::fabs(error) < constants.integralThreshold) integral += error;
    if(integral > constants.maxIntegral) integral = constants.maxIntegral;
    derivative = error - prevError;
    prevError = error;
    return(error * constants.p  + integral * constants.i + derivative * constants.d);
}

// - util functions
double lib::dtr(double input)
{
  return(PI * input/180);
}

double lib::rtd(double input)
{
  return(input * 180/PI);
}

int lib::dirToSpin(double target,double currHeading)
{
    double d = (target - currHeading);
    double diff = d < 0 ? d + 360 : d;
    return(diff > 180 ? 1 : -1);
}

double lib::minError(double target, double current)
{
    double b = std::max(target,current);
    double s = std::min(target,current);
    double diff = b - s;
    
    return((diff <= 180 ? diff : (360-b) + s) * dirToSpin(target, current));
}


double lib::imuToRad(double heading)   
{
    // could be like shifted over? idk
    return (heading < 180) ? dtr(heading) : dtr(-(heading - 180));
}

double lib::sign(double a)
{
    return(a > 0 ? 1 : -1);
}

double lib::hypot(double a, double b)
{
    return(sqrt(pow(a, 2) + pow(b, 2)));
}

double lib::hypot(lib::point a)
{
    return(sqrt(pow(a.x, 2) + pow(a.y, 2)));
}

double lib::dist(const point& a, const point& b)
{
    return(hypot(b.x - a.x, b.y - a.y));
}

double lib::absoluteAngleToPoint(const point& pos, const point& point)
{
    double t;

    try
    { 
        t = atan2(point.x - pos.x, point.y - pos.y);
    }

    catch(...)
    {
        t = PI/2;
    }
    
    t = lib::rtd(t);

    // -270 - 90
    
    // if(t < -180)
    // {
    //     t = 90 + (270 - fabs(t));
    // }

    //-180 - 180

    t = -t;
    t = t >= 0 ? t :  180 + 180+t;
    return (t);
}