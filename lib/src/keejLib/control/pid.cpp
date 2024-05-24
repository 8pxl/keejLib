#include "keejLib/lib.hpp"

namespace keejLib {
    Pid::Pid(PidConstants constants) : constants(constants), integral(0){};
    
    double Pid::out(double error) {
        if(std::fabs(error) < constants.tolerance) integral = 0;
        else if(std::fabs(error) < constants.integralThreshold) integral += error;
        if(integral > constants.maxIntegral) integral = constants.maxIntegral;
        derivative = error - prevError;
        prevError = error;
        return(error * constants.p  + integral * constants.i + derivative * constants.d);
    }
    
    double Pid::getError() {
        return error;
    }
    
    double Pid::getDerivative() {
        return derivative;
    }
}