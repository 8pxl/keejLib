#include "keejLib/lib.h"

using namespace keejLib;
PID::PID(PIDConstants constants) : constants(constants), integral(0){};

double PID::out(double error) {
    if(std::fabs(error) < constants.tolerance) integral = 0;
    else if(std::fabs(error) < constants.integralThreshold) integral += error;
    if(integral > constants.maxIntegral) integral = constants.maxIntegral;
    derivative = error - prevError;
    prevError = error;
    return(error * constants.kp + integral * constants.ki + derivative * constants.kd + (error * constants.kf));
}

double PID::getError() {
    return error;
}

double PID::getDerivative() {
    return derivative;
}
