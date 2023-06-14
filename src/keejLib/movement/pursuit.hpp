#pragma once
#include "../include/keejLib/lib.h"

lib::point lib::chassis::targetPoint(lib::linearPath path, int lookAhead, int lineLookAhead, int lineIndex){

    point targetPoint;
    double closestDist = 1000000000;
    
    int a = lineLookAhead < (path.len() - lineIndex) ? lineIndex+lineLookAhead : path.len() -1;
    for (int i = lineIndex; i <= a; i++)
    {    
        // curr points
        lib::point p1 = path.at(i);
        lib::point p2 = path.at(i+1);

        // offset to origin
        p1 -= this -> pos;
        p2 -= this -> pos;

        // set up to find intersection using this method 
        // https://mathworld.wolfram.com/Circle-LineIntersection.html
        double dx = p2.x-p1.x;
        double dy = p2.y-p1.y;
        double dr = sqrt(pow(dx,2)+pow(dy,2));
        double D = p1.x*p2.y - p2.x * p1.y;

        double discriminant = pow(lookAhead,2)  *  pow(dr,2) - pow(D,2);

        // calculate solutions
        if (discriminant >= 0)
        {
            double sDiscriminant = sqrt(discriminant);
            double dxdy = D * dy;
            double dxdx = D*dx;
            double sdyxdxxsd = sign(dy) * dx * sDiscriminant;
            double dr2 = pow(dr,2);
            double adyxsd = std::abs(dy) * sDiscriminant;
            point farthestPoint = path.at(a-1);

            int minX = std::min(p1.x,p2.x);
            int maxX = std::max(p1.x,p2.x);
            int minY = std::min(p1.y,p2.y);
            int maxY = std::max(p1.y,p2.y);

            double sx1 = (dxdy + sdyxdxxsd) / dr2;
            double sy1 = (-dxdx + adyxsd) / dr2;
            double sx2 = (dxdy - sdyxdxxsd) / dr2;
            double sy2 = (-dxdx - adyxsd) / dr2;

            point s1 = {sx1 + this -> pos.x, sy1 + this -> pos.y};
            point s2 = {sx2+ this-> pos.x, sy2 + this -> pos.y};
            
            bool s1Valid = s1.x >= minX && s1.x <= maxX && s1.y >= minY && s1.y <= maxY;
            bool s2Valid = s2.x >= minX && s2.x <= maxX && s2.y >= minY && s2.y <= maxY;

            double s1Dist = dist(s1, farthestPoint);
            double s2Dist = dist(s2,farthestPoint);

            if (s1Valid && s1Dist < closestDist)
            {
                targetPoint = s1;
                closestDist = s1Dist;
            }

            if (s2Valid && s2Dist < closestDist)
            {
                targetPoint = s2;
                closestDist = s2Dist;
            }

        }

    } 

    return(targetPoint);
}

// distToPoint(x, y, point.getX(), point.getY()) < radius

void lib::chassis::moveToPurePursuit(lib::linearPath path, double lookAhead, int lineLookAhead, int finalTimeout, lib::pidConstants lConstants, lib::pidConstants rConstants, double rotationBias)
{

    bool targetReached = false;
    int lineIndex = 0;
    point target;
    lib::pid linearController(lConstants, 0);
    lib::pid rotationController(rConstants, 0);

    while (!targetReached)
    {

        if (dist(this -> pos, path.at(lineIndex + lineLookAhead)) < lookAhead)
        {
            lineIndex += 1;
        }

        if (dist(this -> pos, path.last()) < lookAhead)
        {
            targetReached = true;
        }

        target = targetPoint(path,lookAhead, lineLookAhead, lineIndex);

        chass -> spinDiffy(lib::chassis::pidMTPVel(target, rotationBias, &linearController, &rotationController));
    }

   this -> pidMoveTo(target, finalTimeout, lConstants, rConstants, rotationBias);
}   