#include "chassis.h"
#include "keejLib/lib.h"
#include "keejLib/util.h"
#include "locolib/distance.h"
#include <cstdio>
#include <ostream>

namespace keejLib {
    
void Chassis::startTracking() {
    if (odomTask == nullptr) {
        vertEnc -> reset_position();
        horizEnc -> reset_position();
        prev = {0,0, Angle(imu -> get_heading(), HEADING)};
        odomTask = new pros::Task{[=] {
            Stopwatch sw;
            while (true) {
                // std::cout << "hi1" << std::endl;
                updateOdom();
                // if (sw.elapsed() > 50) {
                //     sw.reset();
                    // std::cout << dt -> getAvgVelocity(true) << std::endl;
                // }
                    // std::cout << "hi" << std::endl;
                    // std::cout << "x: " << pose.pos.x << " y: " << pose.pos.y << " theta: " << pose.heading.heading() << std::endl;
        // std::cout << "hi2" << std::endl;
                // mcl.update([this](){
                //     return updateOdom();
                // });
                // auto p = mcl.getPrediction();
                // auto particles = mcl.getParticles();
                // // std::printf("[");
                // for (auto q: particles) {
                //     // std::printf("(%.3f, %.3f), ", q.x(), q.y());
                // }
                // std::printf("],\n");
                // pose = {p.x() * 39.3701, p.y() * 39.3701, Angle(p.z(), keejLib::HEADING)};
                // std::cout << "x: " << pose.pos.x << " y: " << pose.pos.y << " theta: " << pose.heading.heading() << std::endl;
                // if (pose.heading.heading() < 0) {
                    // std::cout << "heading: " << pose.heading.heading() << std::endl;
                    // std::cout << "standard: " << pose.heading.rad() << std::endl;
                // }
                                // std::cout << "x: " << pose.pos.x << " y: " << pose.pos.y << " theta: " <<horizDist -> get() << std::endl;
                 // std::printf("(%.3f, %.3f, %.3f),", pose.pos.x, pose.pos.y, pose.heading.heading());
                pros::delay(10);
            }
        }};
        pros::delay(10);
    }
}

void Chassis::updateOdom() {
    // chassMutex.take();
    double rot = imu -> get_heading();
    Angle currTheta;
    if (rot == PROS_ERR_F) {
        currTheta= prev.theta;
    }
    else currTheta = Angle(rot, AngleType::HEADING);
    encMutex.take();
    double currVert = (vertEnc -> get_position() / 100.0);
    double currHoriz = (horizEnc -> get_position() / 100.0);
    encMutex.give();
    // std::cout << "currVert: " << currVert << " currHoriz: " << currHoriz << std::endl;
    double dTheta = toRad(currTheta.error(prev.theta));
    // std::cout << rot << std::endl;
    // std::cout << dTheta << std::endl;
    
    // alternateMutex.take();
    double dVert;
    double dHoriz;
    if (useAltOffsets) {
        dVert = (angError(currVert, prev.vert) * M_PI * alternateOffsets.first) / 360.0;
        dHoriz = (angError(currHoriz, prev.horiz) * M_PI * alternateOffsets.second) / 360.0;
    }
    else {
        dVert = (angError(currVert, prev.vert) * M_PI * chassConsts.vertDia) / 360.0;
        dHoriz = (angError(currHoriz, prev.horiz) * M_PI * chassConsts.horizDia) / 360.0;   
    }
    // alternateMutex.give();
    // std::cout << "dVert: " << dVert << " dHoriz: " << dHoriz << std::endl;
    // std::cout << "dTheta: " << dTheta*10 << std::endl;
    prev.theta = currTheta;
    prev.vert = currVert;
    prev.horiz = currHoriz;
    
    double avgHeading = (toRad(prev.theta.heading()) + dTheta / 2);
    // std::cout << "avgHeading: " << avgHeading << std::endl;
    double dx = dHoriz;
    double dy = dVert;
    
    
    // std::cout << "dx: " << dx << " dy: " <<dy << std::endl;
    double locX = 0;
    double locY = 0;
    
    if (dTheta == 0) {
        locX = dx;
        locY = dy;
    } else {
        locX = 2 * sin(dTheta / 2) * (dx / dTheta + chassConsts.horizWidth);
        locY = 2 * sin(dTheta / 2) * (dy / dTheta + chassConsts.vertWidth);
    }
    // if (fabs(dx) > 0.01 )
    // {
    //     double drift = (dx - (dTheta * chassConsts.horizWidth)) / (dTheta * chassConsts.horizWidth);
    // // if (fabs(drift) > 0.001) std::cout << drift <<std::endl;
    //     std::cout << drift <<std::endl;
    // }
        // std::cout << (dTheta * chassConsts.horizWidth) - dx << std::endl;
    // if (fabs(locX) > 0.1) {
    // }
    
    // std::cout << "x: " << dHoriz << " y: " << locY << std::endl;
    // update globals
    // std::cout <<  2 * sin(dTheta / 2) << std::endl;
    pose.pos.x += locY * sin(avgHeading) - (locX * cos(avgHeading));
    pose.pos.y += locY * cos(avgHeading) + (locX * sin(avgHeading));
    pose.heading = currTheta;
    // velEMA.out(hypot(dx, dy));
    // return { 39.3701 * locY * sin(avgHeading) - (locX * cos(avgHeading)), 39.3701 * locY * cos(avgHeading) + (locX * sin(avgHeading))};
    // chassMutex.give()
}

void Chassis::wallReset(int wall, int numReadings, bool createTask) {
    if (createTask) {
        pros::Task task([&]() { wallReset(wall, numReadings, false);});
        pros::delay(10);
        return;
    }
    double weightedDist = 0;
    double weights = 0;
    // const double imuConfInterval = 4;
    const double mmtoinch = 0.0393701;
    const double offset = 30.0;
    while (numReadings--) {
        double distReading = (horizDist->get_distance() - offset) * mmtoinch;
        double imuReading = imu->get_rotation();
        int target = wall * 90;
        // double imuConfidence = 1 - (fabs(angError(target, imuReading)) / 4);
        // imuConfidence = imuConfidence < 0 ? 0.1 : imuConfidence;
        double confidence = horizDist -> get_confidence();
        weights += confidence;
        weightedDist += distReading * confidence;
        pros::delay(10);
    }
    double avgDist = weightedDist / weights;
    
    const double maxX = 40.1124;
    const double maxY = 5.7999;
    const double minX = -98.527;
    const double minY = -133.19;
    std::cout << "prev x: " << pose.pos.x << " prev y: " << pose.pos.y << std::endl;
    // if (fabs(avgDist) <= 8) {
        switch (wall) {
            case 0:
                pose.pos.x = minX + avgDist;
                break;
            case 1:
                pose.pos.y = maxY - avgDist;
                break;
            case 2:
                pose.pos.x = maxX - avgDist;
                break;
            case 3:
                pose.pos.y = minY + avgDist;
                break;
        }
    // }
    std::cout << "new x: " << pose.pos.x << " new y: " << pose.pos.y << std::endl;
}
}