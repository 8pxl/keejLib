#pragma once
#include "robot.hpp"

using namespace robot;
using namespace lib;

//turns
pidConstants ninety
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
}

lib::atns autons = 
{
    {test1, test2}, 
    {"test", "test2"}
};