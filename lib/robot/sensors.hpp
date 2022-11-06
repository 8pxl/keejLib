#ifndef __SENSORS__
#define __SENSORS__

#include "main.h"
#include "util/util.hpp"

namespace lib 
{
    class imu
    {
        private:
            pros::IMU inertial;
            double initHeading;
        
        public:
            imu(pros::IMU imu, double heading): inertial(imu), initHeading(heading) {}

            double degHeading();
            double radHeading();
            void init(double heading);
    };
}

double lib::imu::degHeading() //NOLINT
{
    double t = inertial.get_heading() + initHeading;
    return(t <= 360 ? t : 0 + (t-360));
}

double lib::imu::radHeading() //NOLINT
{
    double t = inertial.get_heading() + initHeading;
    return(t <= 360 ? util::dtr(t) : util::dtr((t-360)));
}

void lib::imu::init(double heading) //NOLINT
{
    initHeading = heading;
}

#endif