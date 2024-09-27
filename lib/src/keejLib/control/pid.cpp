#include "keejLib/lib.h"

namespace keejLib {
    
PID::PID(PIDConstants constants) : constants(constants), integral(0){};

double PID::out(double error) {
    prevTime = pros::millis();
    if(std::fabs(error) < constants.integralThreshold) integral += error;
    else integral = 0;
    if(std::fabs(error) < constants.tolerance) integral = 0;
    if (sign(error) != sign(prevError)) integral = 0;
    // if(integral > constants.maxIntegral) integral = constants.maxIntegral;
    integral = std::clamp(integral, -constants.maxIntegral, constants.maxIntegral);
    derivative = (error - prevError);
    prevError = error;
    return(error * constants.kp + integral * constants.ki + derivative * constants.kd + (error * constants.kf));
}

double PID::getError() {
    return error;
}

double PID::getDerivative() {
    return derivative;
}

}