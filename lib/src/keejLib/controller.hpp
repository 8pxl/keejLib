#pragma once
#include "lib.hpp"


int lib::controller::select(std::vector<std::string> names)
{
    int num = names.size();
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
    if (scale != 0)
    {
        return(pow(2.718, (scale * ((std::fabs(x) - 127))) / 1000 ) * x);
    }
    return x;
}

std::vector<double> lib::controller::drive(int direction, driveMode mode)
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

        case curvature:
            rStick = curve(cont ->get_analog(ANALOG_RIGHT_X), rightCurve);
            if (lStick == 0) {
                return(std::vector<double>{lStick + rStick, lStick - rStick});
            }
            double left = lStick + (std::abs(lStick) * rStick) / 127.0;
            double right = lStick - (std::abs(lStick) * rStick) / 127.0;
            return (std::vector<double>{left, right});
    }

    //you shoudlnt be here !
    return(std::vector<double>{0,0});
}

void lib::controller::setCurves(double left, double right)
{
    leftCurve = left;
    rightCurve = right;
}