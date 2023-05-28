#pragma once
#include "main.h"
#include "lib.hpp"

pros::motor_brake_mode_e lib::mtrs::returnBrakeType(char brakeMode) 
{
    return brakeMode == 'c' ? pros::E_MOTOR_BRAKE_COAST : brakeMode == 'b' ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_HOLD;
}

lib::mtrs::mtrs(const std::vector<int> ports)
{
    for (int i : ports)
    {
        pros::Motor temp(std::abs(i), i < 0 ? true : false);
        motors.push_back(temp);
    }
    size = ports.size();
}

void lib::mtrs::spin(double volts) 
{
    for (int i=0; i < size; i++)
    {
        motors[i].move(volts);
    }
}

void lib::mtrs::stop(char brakeMode) 
{
    pros::motor_brake_mode_e brakeType = returnBrakeType(brakeMode);

    for (int i=0; i < size; i++)
    {
        motors[i].set_brake_mode(brakeType);
        motors[i].brake();
    }
}

void lib::mtrs::setBrake(char brakeMode) 
{
    pros::motor_brake_mode_e brakeType = returnBrakeType(brakeMode);

    for (int i=0; i < size; i++)
    {
        motors[i].set_brake_mode(brakeType);
    }
}

double lib::mtrs::getSpeed() 
{
    double vel = 0;

    for (int i=0; i < size; i++)
    {
        vel += motors[i].get_actual_velocity();
    }
    
    return(vel/size);
}

double lib::mtrs::getRotation() 
{
    double rotation = 0;

    for (int i=0; i < size; i++)
    {
        rotation += motors[i].get_position();
    }
    
    return(rotation/size);
}

void lib::mtrs::reset() 
{
    for (int i=0; i < size; i++)
    {
        motors[i].set_zero_position(0);
    }
}

void lib::diffy::spinDiffy(std::vector<int> ports, std::vector<int> lookup_){
    for (int i : ports)
    {
        pros::Motor temp(std::abs(i), i < 0 ? true : false);
        motors.push_back(temp);
    }
    lookup = lookup_;
}

void lib::diffy::spinDiffy(std::vector<double> voltages) 
{
    for (int i=0; i < voltages.size(); i++)
    {
        motors[i].move(voltages[lookup[i]]);
    }
}

std::vector<double> lib::diffy::getDiffy() 
{
    double dl = 0;
    double dr = 0;
    int half = size/2;
    
    for (int i=0; i < half; i++)
    {
        dl += motors[i].get_position();
        dr += motors[i + half].get_position();
    }
    
    return(std::vector<double> {dr/half, dl/half});
}

lib::pis::pis(std::vector<pros::ADIDigitalOut> p, bool s) : pistons(p), state(s)
{
    setState(s);
}

void lib::pis::toggle() 
{
    state = !state;

    for(int i = 0; i < pistons.size(); i++)
    {
        pistons[i].set_value(state);
    }
}

void lib::pis::setState(bool iState) 
{
    state = iState;

    for(int i = 0; i < pistons.size(); i++)
    {
        pistons[i].set_value(state);
    }
}

bool lib::pis::getState()
{
    return(state);
}
