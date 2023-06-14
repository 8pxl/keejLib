#pragma once
#include "../include/keejLib/lib.h"

lib::cubicBezier::cubicBezier(const point& p0, const point& p1, const point& p2, const point& p3) : p0(p0), p1(p1), p2(p2), p3(p3) {}

lib::point lib::cubicBezier::evaluate(double t) 
{
    double omt = 1 - t;
    double omt2 = omt * omt;
    double omt3 = omt2 * omt;
    double t2 = t * t;
    double t3 = t2 * t;
    double x = omt3 * p0.x + 3 * omt2 * t * p1.x + 3 * omt * t2 * p2.x + t3 * p3.x;
    double y = omt3 * p0.y + 3 * omt2 * t * p1.y + 3 * omt * t2 * p2.y + t3 * p3.y;
    return {x,y};
}

lib::vec lib::cubicBezier::evaluateDerivative(double t) 
{
    double omt = 1 - t; 
    double omt2 = omt * omt;
    double t2 = t * t;
    double x = 3 * omt2 * (p1.x - p0.x) + 6 * omt * t * (p2.x - p1.x) + 3 * t2 * (p3.x - p2.x);
    double y = 3 * omt2 * (p1.y - p0.y) + 6 * omt * t * (p2.y - p1.y) + 3 * t2 * (p3.y - p2.y);
    return {x,y};
}

double lib::cubicBezier::length(int reso = 100) 
{
    double result = 0;
    point prev = p0;
    for (int i = 1; i <= reso; ++i) {
        double t = static_cast<double>(i) / reso;
        point curr = evaluate(t);
        result += dist(curr, prev);
        prev = curr;
    }
    return result;
}

//assumes the patlinearPathb.
//either that or add these coefficens to the cubic solver (i do not want to do that)
//https://www.desmos.com/calculator/kbbuvzn3xb
//https://arc.net/e/8C306DBF-9273-4476-B4AF-A33BE6ADB8EF
double lib::cubicBezier::maxDeriv()
{
    return (std::max(evaluateDerivative(0.00000001).mag(), evaluateDerivative(1).mag()));
}

lib::linearPath::linearPath(std::vector<point> points)
{
    this -> points = points;
    this -> length = points.size();
}

int lib::linearPath::len()
{
    return (this -> length);
}

lib::point lib::linearPath::at(int index)
{
    return(this -> points[index]);
}

lib::point lib::linearPath::last()
{
    return(this -> points[(this -> length) -1]);
}