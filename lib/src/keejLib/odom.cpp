#include "keejLib/lib.h"

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
    double currTheta = imu -> get_rotation();
    double currVert = (vertEnc -> get_position()) * constants.trackDia * M_PI / 360;
    double currHoriz = (horizEnc -> get_position()) * constants.trackDia * M_PI / 360;
    
    double dTheta = degToRad(currTheta - prev.theta);
    double dVert = currVert - prev.vert;
    double dHoriz = currHoriz - prev.horiz;
    
    prev.theta = currTheta;
    prev.vert = currVert;
    prev.horiz = currHoriz;
    
    double avgHeading = (prev.theta + currTheta) / 2;
    pose.heading += dTheta;
    
    double dx = dVert;
    double dy = dHoriz;
    
    double locX = 0;
    double locY = 0;
    
    if (dTheta == 0) {
        locX = dx;
        locY = dy;
    } else {
        locX = 2 * sin(dTheta / 2) * (dx / dTheta + constants.horizWidth);
        locY = 2 * sin(dTheta / 2) * (dy / dVert + constants.vertWidth);
    }
    Pose prevPose = pose;

    // update globals
    pose.pos.x += locY * sin(avgHeading);
    pose.pos.y += locY * cos(avgHeading);
    pose.pos.x += locX * -cos(avgHeading);
    pose.pos.y += locX * sin(avgHeading);
    pose.heading = currTheta;
}