#pragma once
#include "main.h"
#include "lib.hpp"

pros::motor_brake_mode_e lib::mtrs::returnBrakeType(char brakeMode) 
{
    return brakeMode == 'c' ? pros::E_MOTOR_BRAKE_COAST : brakeMode == 'b' ? pros::E_MOTOR_BRAKE_BRAKE : pros::E_MOTOR_BRAKE_HOLD;
}

lib::mtrs::mtrs(const std::vector<int> & ports)
{
    for (int i : ports)
    {
        pros::Motor temp(std::abs(i), pros::E_MOTOR_GEARSET_06, i < 0 ? true : false);
        motors.push_back(temp);
        temp.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
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

double lib::mtrs::getEfficiency() 
{
    double result = 0;
    for (auto motor: motors)
    {
        result += motor.get_efficiency();
    }
    return result / size;
}

void lib::mtrs::reset() 
{
    for (int i=0; i < size; i++)
    {
        motors[i].set_zero_position(0);
    }
}

void lib::diffy::spinDiffy(double rvolt, double lvolt) 
{
    int half = size/2;

    for (int i=0; i < half; i++)
    {
        motors[i].move(rvolt);
        motors[i + half].move(lvolt);
    }
}

void lib::diffy::spinDiffy(std::vector<double> voltages) 
{
    int half = size/2;

    for (int i=0; i < half; i++)
    {
        motors[i].move(voltages[0]);
        motors[i + half].move(voltages[1]);
    }
}

std::vector<double> lib::diffy::getDiffy() 
{
    double dl = 0;
    double dr = 0;
    int half = size/2;
    
    for (int i=0; i < half; i++)
    {
        std::cout << motors[0].get_encoder_units() << std::endl;
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