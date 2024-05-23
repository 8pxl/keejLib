#include "keejlib/units.h"
#include "main.h"
#include "units.h"

using namespace units::length;
using namespace units::angular_velocity;

namespace keejLib {
    class VelocityController {
        private:
            pros::MotorGroup* mtrs;
        public:
            void setVelocity(revolutions_per_minute_t v);
            void applyVoltage();
    };
}