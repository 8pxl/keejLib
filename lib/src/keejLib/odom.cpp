#include "keejLib/lib.h"
#include "keejLib/util.h"

using namespace keejLib;
void Chassis::startTracking() {
    if (odomTask == nullptr) {
        odomTask = new pros::Task{[=] {
            while (true) {
                update();
                pros::delay(10);
            }
        }};
    }
}

void Chassis::update() {
    Angle currTheta = Angle(imu -> get_rotation(), AngleType::HEADING);
    double currVert = (vertEnc -> get_position()) * chassConsts.trackDia * M_PI / 360;
    double currHoriz = (horizEnc -> get_position()) * chassConsts.trackDia * M_PI / 360;
    
    Angle dTheta = currTheta - prev.theta;
    double dVert = currVert - prev.vert;
    double dHoriz = currHoriz - prev.horiz;
    
    prev.theta = currTheta;
    prev.vert = currVert;
    prev.horiz = currHoriz;
    
    Angle avgHeading = (prev.theta + currTheta) / 2;
    pose.heading += dTheta;
    
    double dx = dVert;
    double dy = dHoriz;
    
    double locX = 0;
    double locY = 0;
    
    if (dTheta == 0) {
        locX = dx;
        locY = dy;
    } else {
        locX = 2 * sin(dTheta.rad() / 2) * (dx / dTheta.rad() + chassConsts.horizWidth);
        locY = 2 * sin(dTheta.rad() / 2) * (dy / dVert + chassConsts.vertWidth);
    }
    Pose prevPose = pose;

    // update globals
    pose.pos.x += locY * sin(avgHeading.rad()) + locX * -cos(avgHeading.rad());
    pose.pos.y += locY * cos(avgHeading.rad()) + locX * sin(avgHeading.rad());
    pose.heading = currTheta;
}