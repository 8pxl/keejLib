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
            
            //add chassis funcs here! (drive pid and the sorts)
    };
}

#endif