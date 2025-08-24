#include "keejLib/chassis.h"
#include "keejLib/control.h"
#include "keejLib/lib.h"
#include "keejLib/util.h"
#include "pros/rtos.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>

namespace keejLib {

    void Chassis::moveWithin(Pt target, double dist, MotionParams params, double angle) {
        // chassMutex.take();
        if (params.async) {
            params.async = false;
            pros::Task task([&]() { moveWithin(target, dist, params);});
            pros::delay(10);
            return;
        }
        this -> waitUntilSettled();

        if (params.exit == nullptr) {
            params.exit = new exit::Range(5, 50);
        }

        moving = true;
        if (clr == blue) target = translate(target);
        Exit* timeout = new exit::Timeout(params.timeout);
        
        PID linCont(mtpLin);
        // PID angCont(angConsts);
        PID angCont(mtpAng);
        
        double linError = pose.pos.dist(target);
        Angle heading;
        if (angle == -1) heading = Angle(imu->get_heading(), HEADING);
        else heading = Angle(angle, HEADING);
        double prev = 0;
        while (!params.exit -> exited({.error = fabs(linError)}) && !timeout -> exited({})) {
            linError = pose.pos.dist(target) - dist;
            if (linError < 0 && params.vMin != 0) break;
            double angularError = heading.error(Angle(imu -> get_heading(), HEADING));
        
            if (std::abs(angularError) < this -> angConsts.tolerance) {
                angularError = 0;
            }
            double va = angCont.out(angularError);
            double vl = linCont.out(linError);
            if (std::abs(vl) < params.vMin) {
                vl = params.vMin * sign(vl);
            }
            
            if (std::abs(vl) + std::abs(va) > 127) {
              vl = (127 - std::abs(va)) * sign(vl);
            }
            //only slews when accelerating
            if (params.slew != 0) vl = std::min(prev + params.slew, vl);
            prev = vl;
            if (params.reverse) vl = -vl;
            
            this -> dt -> spinVolts({vl + va, vl - va});
            pros::delay(10);
        }
        this -> dt -> spinAll(0);
        moving = false;
        // chassMutex.give()
    }
    

void Chassis::driveAngle(double dist, double angle, MotionParams params = { .vMin = 0, .slew = 2}, bool absolute) {
    // chassMutex.take();
    if (params.async) {
        params.async = false;
        pros::Task task([&]() { driveAngle(dist, angle, params);});
        pros::delay(10);
        return;
    }
    if (params.exit == nullptr) {
        params.exit = new exit::Range(20, 20);
    }
    this -> waitUntilSettled();
    // double prev = dt -> getAvgVelocity();
    moving = true;
    if (clr == blue) angle = neg(angle);
    Angle targ = Angle(angle, HEADING);
    Exit* timeout = new exit::Timeout(params.timeout);
    PID linCont = PID(this -> linConsts);
    PID angCont = PID(this -> angConsts);
    double linError = dist;
    double traveled = 0;
    if (!absolute) this -> dt -> tare_position();
    // double prev = chassConsts.wheelDia * M_PI * (vertEnc->get_position()/36000.0);
    double prev =  dt -> getLastCommanded();
    while (!params.exit -> exited({.error = fabs(linError)}) && !timeout -> exited({})) {
        linError = dist - (this -> dt -> getAvgPosition());
        if (params.vMin > 0 && linError * sign(dist) < 0) {
            break;
        }
        double angularError = targ.error(Angle(imu -> get_heading(), HEADING));
    
        if (std::abs(angularError) < this -> angConsts.tolerance) {
            angularError = 0;
        }
        double va = angCont.out(angularError);
        double vl = linCont.out(linError);
        if (std::fabs(vl) < params.vMin) {
            // vl = params.vMin * sign(vl);
            // if (params.slew != 0) vl = std::min(params.vMin, std::fabs(vl) + params.slew) * sign(vl);
            vl = params.vMin * sign(vl);
        }
        if (std::abs(vl) > params.vMax) {
            vl = params.vMax * sign(vl);
        }
        
        if (std::abs(vl) + std::abs(va) > 127) {
          vl = (127 - std::abs(va)) * sign(vl);
        }
        if (params.slew !=0) vl = sign(vl) * std::min(fabs(prev) + params.slew, fabs(vl));
        prev = vl;
        
        this -> dt -> spinVolts({vl + va, vl - va});
        pros::delay(20);
    }
    if (params.vMin == 0) this -> dt -> spinAll(0);
    moving = false;
    // chassMutex.give();
}

void Chassis::mtpoint(Pt target, MotionParams params = {.slew = 4}) {
    if (params.async) {
        params.async = false;
        pros::Task task([&]() { mtpoint(target, params);});
        pros::delay(10);
        return;
    }
    this -> waitUntilSettled();
    if (params.exit == nullptr) params.exit = new exit::Range(3, 20);
    moving = true;

    Exit* timeout = new exit::Timeout(params.timeout);
    exit::Perp* perp = new exit::Perp(target);
    PID linCont(mtpLin);
    PID angCont(mtpAng);
    double dist = pose.pos.dist(target);
    bool close = false;
    VelocityManager velCalc(dt -> getLastCommanded(), 0, params.slew, params.vMin, params.vMax, params.angMin, params.angMax);
    //https://www.desmos.com/calculator/cnp2vnubnx
    while (!timeout -> exited({}) && !params.exit -> exited({.error = dist, .pose = pose })) {  

        //restrict angular velocity when close
        if (close) velCalc.setAngMax(0);
        else velCalc.setAngMax(params.angMax);
        
        if (params.vMin != 0) {
            if (perp->exited({.pose = pose, .targetHeading = pose.heading})) {
                break;
            }
        }
        
        //calculate direction based on side
        int side = perp->computeSide({.pose = pose, .targetHeading = pose.heading});
        int dir = side * (params.reverse ? -1 : 1);
        
        //compute errors
        double linearError = pose.pos.dist(target);
        dist = linearError; //sets the true error before using cosine scaling
        double angularError = mtpAngleError(pose, target, dir);
        if (params.within > 0) linearError -= params.within;
        
        linearError *= cos(toRad(angularError));
        
        if (fabs(dist) < params.settleRange) close = true;
        else close = false;
        
        //calculate output velocities
        double linearVel = dir * linCont.out(linearError);
        double angularVel = angCont.out(angularError);
        
        //calc max slip speed
        double maxSlipSpeed = calculateMaxSlipSpeed(pose, target, params.drift);
        
        //update limits
        velCalc.setLinMax(std::min(maxSlipSpeed, params.vMax));
        
        auto vels = velCalc.update({linearVel,angularVel});
        dt -> spinVolts(vels);
        pros::delay(10);
        std::cout << vels.left << ", " << vels.right << std::endl;
        if (params.debug) {
            std::cout << "linear vel: " << linearVel << " angular vel: " << angularVel << std::endl;
            std::cout << linearError << std::endl;
            // std::cout <<"linmin" <<
            // std::cout << maxSlipSpeed << std::endl;
            // std::printf("curr: (%.3f, %.3f, %.3f) \n", pose.pos.x, pose.pos.y, pose.heading.heading());
            // std::printf(" target: (%.3f, %.3f, %.3f) \n", target.x, target.y, targetHeading.heading());
            // std::printf(" angularError: %.3f \n", angularError);
        }
    }
    if (params.vMin != 0) dt -> spinAll(0);
    moving = false;
    // chassMutex.give();
}

void Chassis::mtpose(Pose target, double dLead, MotionParams params, double gLead, double gSettle) {
    if (params.async) {
        params.async = false;
        pros::Task task([&]() { mtpose(target, dLead, params, gLead);});
        pros::delay(10);
        return;
    }
    this -> waitUntilSettled();
    if (params.exit == nullptr) params.exit = new exit::Range(1, 20);
    moving = true;

    Exit* timeout = new exit::Timeout(params.timeout);
    exit::Perp* perp = new exit::Perp(target.pos, target.heading);
    PID linCont(mtposeLin);
    PID angCont(mtposeAng);
    double dist = pose.pos.dist(target.pos);
    bool close = false;
    Pt targetPoint;
    
    VelocityManager velCalc(dt -> getLastCommanded(), 0, params.slew, params.vMin, params.vMax, params.angMin, params.angMax);
    // https://www.desmos.com/calculator/cnp2vnubnx
    while (!timeout -> exited({}) && !params.exit -> exited({.error = dist, .pose = pose })) {  
        
        //restrict angular and adjust target point when close
        if (close) {
            // velCalc.setAngMax(0);
            targetPoint = target.pos;
        }
        else {
            velCalc.setAngMax(params.angMax);
            double h = std::hypot(pose.pos.x - target.pos.x, pose.pos.y - target.pos.y);
            targetPoint = {target.pos.x - (h * sin(toRad(target.heading.heading())) * dLead), target.pos.y - (h * cos(toRad(target.heading.heading())) * dLead)}; //carrot point
            if (gLead != -1) {
                Pt carrot = {targetPoint.x * gLead, targetPoint.y * gLead};
                MotionParams gParams = params;
                gParams.vMin = mtposeLin.kp * (gSettle + targetPoint.dist(target.pos));
                gParams.settleRange = params.settleRange;
                gParams.exit = new exit::Range(gSettle, 10);
                gParams.timeout = params.timeout / 1.8;
                gParams.async = false;
                gParams.debug = true;
                std::cout << "starting mtp!" << std::endl;
                moving = false;
                this->mtpoint(carrot, gParams);
                moving = true;
                gLead = -1;
            }
        }
    
        //perp line exit
        if (params.vMin != 0) {
            if (perp->exited({.pose = pose, .targetHeading = pose.heading})) {
                break;
            }
        }
        
        //calculate direction based on side
        int side = perp->computeSide({.pose = pose, .targetHeading = pose.heading});
        int dir = side * (params.reverse ? -1 : 1);
        
        //compute errors
        double linearError = pose.pos.dist(target.pos);
        dist = linearError; //sets the true error before using cosine scaling
        double angularError;
        if (close) angularError = target.heading.error(pose.heading);
        else angularError = mtpAngleError(pose, targetPoint, dir);
        if (params.within > 0) linearError -= params.within;
        // if (close) linearError *= cos(toRad(angularError));
        linearError *= cos(toRad(angularError));
        
        if (fabs(dist) < params.settleRange) close = true;
        else close = false;
        
        //calculate output velocities
        double linearVel = dir * linCont.out(linearError);
        double angularVel = angCont.out(angularError);
        
        //calc max slip speed
        double maxSlipSpeed = calculateMaxSlipSpeed(pose, targetPoint, params.drift);
        
        //update limits
        velCalc.setLinMax(std::min(maxSlipSpeed, params.vMax));
        
        dt -> spinVolts(velCalc.update({linearVel, angularVel}));
        pros::delay(10);
        
        if (params.debug) {
            std::cout << targetPoint.x << " " << targetPoint.y << std::endl;
            // std::printf("curr: (%.3f, %.3f, %.3f) \n", pose.pos.x, pose.pos.y, pose.heading.heading());
            // std::printf(" target: (%.3f, %.3f, %.3f) \n", target.x, target.y, targetHeading.heading());
            // std::printf(" angularError: %.3f \n", angularError);
        }
    }
    if (params.vMin != 0) dt -> spinAll(0);
    moving = false;
    // chassMutex.give();
}

void Chassis::holdPos(double angle, double offset, double timeout, MotionParams params) {
    this -> waitUntilSettled();
    moving = true;
    double start = vertEnc -> get_position() / 100.0;
    Exit* timer = new exit::Timeout(timeout);
    PID angCont = PID(this -> angConsts);
    double error = 1000;
    
    while (!timer -> exited({}) && !params.exit -> exited({.error = error})) {
        encMutex.take();
        error = (angError(start + offset, vertEnc -> get_position() / 100.0) * M_PI * chassConsts.vertDia) / 360.0;
        encMutex.give();
        std::cout << error << std::endl;
        
        //scale error via regressed exponential graph https://www.desmos.com/calculator/zjzo4sdlnr
        double vl = 0.233427 * (pow(66.72679, fabs(error)) - 1) * sign(error);
        
        double angularError = Angle(angle, HEADING).error(Angle(imu -> get_heading(), HEADING));
        double va = angCont.out(angularError);
        if (std::abs(vl) + std::abs(va) > 127) {
          vl = (127 - std::abs(va)) * sign(vl);
        }
        
        this -> dt -> spinVolts({vl + va, vl - va});
        pros::delay(10);

        // dt ->spinal)
    }
    moving = false;
}
}