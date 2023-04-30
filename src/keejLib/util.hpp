#pragma once

#include "../../include/keejLib/util.h"

#define ALLBUTTONS {pros::E_CONTROLLER_DIGITAL_L1, pros::E_CONTROLLER_DIGITAL_L2, pros::E_CONTROLLER_DIGITAL_R1, pros::E_CONTROLLER_DIGITAL_R2, pros::E_CONTROLLER_DIGITAL_UP, pros::E_CONTROLLER_DIGITAL_DOWN, pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT, pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_B, pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A}
typedef void(*fptr)();

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

// - controller
enum lib::controller::driveMode
{
    arcade,
    tank
};

int lib::controller::select(int num, std::vector<std::string> names)
{
    int curr = 0;
    cont -> clear();
    while(1)
    {   
        if(cont -> get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            if (curr != num-1)
            {
                curr++;
            }
            
            else
            {
                curr = 0;
            }
        }

        if(cont -> get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT))
        {
            if (curr != 0)
            {
                curr--;
            }

            else
            {
                curr = num-1;
            }
        }

        if(cont -> get_digital(pros::E_CONTROLLER_DIGITAL_A))
        {
            pros::delay(200);
            return(curr);
        }

        cont -> print(0, 0, "%s         ", names[curr]);
        pros::delay(50);
    }
}

std::vector<bool> lib::controller::getAll(std::vector<pros::controller_digital_e_t> buttons)
{
    std::vector<bool> out;
    for (pros::controller_digital_e_t button : buttons)
    {
        out.push_back(cont -> get_digital(button));
        out.push_back(cont -> get_digital_new_press(button));
    }
    return(out);
}

//https://www.desmos.com/calculator/puepnlubzh
double lib::controller::curve(double x, double scale) 
{
    return scale == 0 ? x : (pow(2.718, (scale * ((std::fabs(x) - 127))) / 1000 ) * x);
}

std::vector<double> lib::controller::drive(int direction, lib::controller::driveMode mode)
{   
    double lStick = curve(cont -> get_analog(ANALOG_LEFT_Y) * direction, leftCurve);
    double rStick;
    switch(mode)
    {
        case arcade:
            rStick = curve(cont ->get_analog(ANALOG_RIGHT_X), rightCurve);
            return(std::vector<double>{lStick + rStick, lStick - rStick});
        
        case tank:
            rStick = curve(cont -> get_analog(ANALOG_RIGHT_Y), rightCurve);
            return(std::vector<double>{lStick, rStick});
    }

    //you shoudlnt be here !
    return(std::vector<double>{0,0});
}

void lib::controller::setCurves(double left, double right)
{
    leftCurve = left;
    rightCurve = right;
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
    
    return(diff <= 180 ? diff : (360-b) + s);
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