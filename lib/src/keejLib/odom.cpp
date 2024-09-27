#include "keejLib/lib.h"
#include "keejLib/util.h"
#include <cstdio>

namespace keejLib {
    
void Chassis::startTracking() {
    if (odomTask == nullptr) {
        vertEnc -> reset_position();
        horizEnc -> reset_position();
        odomTask = new pros::Task{[=] {
            while (true) {
                update();
                // std::cout << "x: " << pose.pos.x << " y: " << pose.pos.y << " theta: " << pose.heading.deg() << std::endl;
                // std::printf("(%.3f, %.3f, %.3f),", pose.pos.x, pose.pos.y, pose.heading.heading());
                pros::delay(5);
            }
        }};
    }
}

void Chassis::update() {
    double rot = imu -> get_rotation();
    Angle currTheta;
    if (rot == PROS_ERR_F) {
        currTheta= prev.theta;
        std::cout << "imu disconnected!" << std::endl;
    }
    else currTheta = Angle(imu -> get_rotation(), AngleType::HEADING);
    // std::cout << "heading: " << currTheta.heading() << std::endl;
    double currVert = (vertEnc -> get_position() / 100.0);
    double currHoriz = (horizEnc -> get_position() / 100.0);
    // std::cout << "currVert: " << currVert << " currHoriz: " << currHoriz << std::endl;
    double dTheta = toRad(currTheta.error(prev.theta));
    // std::cout << toDeg(dTheta) << std::endl;
    
    double dVert = (angError(currVert, prev.vert) * M_PI * chassConsts.wheelDia) / 360.0;
    double dHoriz = (angError(currHoriz, prev.horiz) * M_PI * chassConsts.wheelDia) / 360.0;
    // std::cout << "dVert: " << dVert << " dHoriz: " << dHoriz << std::endl;
    // std::cout << "dTheta: " << dTheta*10 << std::endl;
    prev.theta = currTheta;
    prev.vert = currVert;
    prev.horiz = currHoriz;
    
    double avgHeading = (toRad(prev.theta.heading()) + dTheta / 2);
    // std::cout << "avgHeading: " << avgHeading << std::endl;
    double dx = dHoriz;
    double dy = dVert;
    
    
    // std::cout << "dx: " << dx << " dy: " << dy << std::endl;
    double locX = 0;
    double locY = 0;
    
    if (dTheta == 0) {
        locX = dx;
        locY = dy;
    } else {
        locX = 2 * sin(dTheta / 2) * (dx / dTheta + chassConsts.horizWidth);
        locY = 2 * sin(dTheta / 2) * (dy / dTheta + chassConsts.vertWidth);
    }
    
    // std::cout << "x: " << locX << " y: " << locY << std::endl;
    // update globals
    pose.pos.x += locY * sin(avgHeading) - (locX * cos(avgHeading));
    pose.pos.y += locY * cos(avgHeading) + (locX * sin(avgHeading));
    pose.heading = currTheta;
}

}