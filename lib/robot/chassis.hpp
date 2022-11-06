#ifndef __CHASSIS__
#define __CHASSIS__

#include "groups.hpp"
#include "sensors.hpp"
#include "util/util.hpp"

namespace lib
{
    class chassis
    {
        private:
            lib::diffy chass;
            lib::imu imu;
            util::coordinate pos;

        public:
            chassis(lib::diffy mtrs, lib::imu inertial, util::coordinate position) : chass(mtrs), imu(inertial), pos(position){}

            void updatePos(double rx, double ry);
            void drive(double target, double timeout, util::pidConstants constants);

            //add chassis funcs here! (drive pid and the sorts)
    };
}

void lib::chassis::updatePos(double rx, double ry) 
{
  pos.x += rx;
  pos.y += ry;
}

void lib::chassis::drive(double target, double timeout, util::pidConstants constants)
{
  double error = target;
  util::timer timer;
  util::pid pidController(constants, target);
  chass.reset();

  while(timer.time() < timeout)
  {
    error = target - chass.getRotation();
    chass.spin(pidController.out(error));
  }

  chass.stop('b');
}

#endif