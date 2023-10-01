#pragma once
#include "robot.hpp"

using namespace robot;
using namespace lib;

//example turn constants
pidConstants ninety
{
    .p = 5.9, 
    .i = 0.6, 
    .d = 70, 
    .tolerance = 0.05, 
    .integralThreshold = 1.1, 
    .maxIntegral = 20
}; 

pidConstants fortyfive
{
    .p = 5.9, 
    .i = 0.6, 
    .d = 70, 
    .tolerance = 0.05, 
    .integralThreshold = 1.1, 
    .maxIntegral = 20
}; 


void test1()
{
    chass.profiledDrive(10, 100);
}

void test2()
{
    chass.pidTurn(90, 1500, ninety);
    chass.pidTurn(45, 1500, fortyfive);
    auto c = chass.pidDrive(1000, 500, ninety, 'c');
    intake.spin(127);
    chass.pidDrive(1000,500, 'b', c);
}

//creates list of auton function pointers and names - useful for auton selector
lib::atns autons = 
{
    {test1, test2}, 
    {"test", "test2"}
};