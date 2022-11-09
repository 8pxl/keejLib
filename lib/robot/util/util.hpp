#ifndef __UTIL__
#define __UTIL__

#include "main.h"
#include <cmath>
#include <vector>

#define PI 3.14159265358979323846

namespace util
{
    class timer;
    class coordinate;
    class pose;
    class bezier;
    class pidConstants;
    class pid;
    class movingAverage;
    double dtr(double input);
    double rtd(double input);
    double distToPoint(util::coordinate p1, util::coordinate p2);
    double mod(double a, double b);
    double absoluteAngleToPoint(util::coordinate pos, util::coordinate point);
    double imuToRad(double heading);
}

class util::timer
{

    public:
    
        int startTime = 0;

        timer()
        {
            start();
        }

        timer(int a){}

        void start()
        {
            startTime = pros::millis();
        }

        int time()
        {
            return (pros::millis() - startTime);
        }

};

class util::coordinate
{
    public:
        double x;
        double y;

        coordinate(double px, double py)
        {
            x = px;
            y = py;
        }

        coordinate();
};

class util::pose
{
    public:

        util::coordinate pos;
        double heading;

        pose(util::coordinate p, double h) : pos(p), heading(h){}
};

class util::bezier
{
    private:
        coordinate p0;
        coordinate p1;
        coordinate p2;
        coordinate p3;
        double initialWeight;
        double finalWeight;
        double initialHeading;
        double finalHeading;

    public:
        bezier(coordinate first, coordinate last, double initialWeight, double finalWeight, double initialHeading, double finalHeading)
        {
            p0 = first;
            p1 = coordinate(first.x + sin(initialHeading) * initialWeight, first.y + cos(initialHeading) * initialWeight);
            p2 = coordinate(last.x + sin(PI/2 + (PI/2-finalHeading)) * -1 * finalWeight, last.y + cos(PI/2 + (PI/2-finalHeading)) * -1*finalWeight);
            p3 = last;
        }
        
        coordinate solve(double t)
        {
            double omt = 1-t;
            double x0 = p0.x;
            double x1 = p1.x;
            double x2 = p2.x;
            double x3 = p3.x;
            double y0 = p0.y;
            double y1 = p1.y;
            double y2 = p2.y;
            double y3 = p3.y;
            return coordinate( pow(omt,3) * x0 + 3 * pow(omt,2) * t * x1 + 3*omt * pow(t,2) * x2 + pow(t,3) * x3, pow(omt,3) * y0 + 3 * pow(omt,2) * t * y1 + 3*omt * pow(t,2) * y2 + pow(t,3) * y3);
        }
        

        // double tangentLineAngle(double t)
        // {
        //     double x0 = p0.x;
        //     double x1 = p1.x;
        //     double x2 = p2.x;
        //     double x3 = p3.x;
        //     double y0 = p0.y;
        //     double y1 = p1.y;
        //     double y2 = p2.y;
        //     double y3 = p3.y;

        //     ex = symbols('x')
        //     omt = 1-ex
        //     # bx = pow(omt,3) * x0 + 3 * pow(omt,2) * t * x1 + 3*omt * pow(t,2) * x2 + pow(t,3) * x3
        //     # by = pow(omt,3) * y0 + 3 * pow(omt,2) * t * y1 + 3*omt * pow(t,2) * y2 + pow(t,3) * y3

        //     bx = pow(omt,3) * x0 + 3 * pow(omt,2) * ex * x1 + 3*omt * pow(ex,2) * x2 + pow(ex,3) * x3
        //     by = pow(omt,3) * y0 + 3 * pow(omt,2) * ex * y1 + 3*omt * pow(ex,2) * y2 + pow(ex,3) * y3
            

        //     bpx = diff(bx,ex).evalf(subs={ex: t})
        //     bpy = diff(by,ex).evalf(subs={ex: t})

        //     m = bpy/bpx

        //     return(atan2(1,m))
        // }

        std::vector<coordinate> createLUT(double resolution)
        {
            std::vector<coordinate> points;
            // std::vector<double> angles;

            for (int i=0; i < resolution; i++)
            {
                points.push_back(solve(i/resolution));
                // angles.push_back(tangentLineAngle(i/resolution));
            }

            return (points);
        }

        double approximateLength(std::vector<coordinate> lut, double resolution)
        {
            double length = 0;

            for (int i=0; i < resolution; i++)
            {
                coordinate first = lut[i];
                coordinate second = lut[i+1];
                length += distToPoint(first,second);
            }

            return length;
        }
};

class util::pidConstants
{
    public:
        double p,i,d,tolerance,integralThreshold, maxIntegral;
        pidConstants(double kp, double ki, double kd, double deviation, double threshold, double maxI) : p(kp), i(ki), d(kd), tolerance(deviation), integralThreshold(threshold), maxIntegral(maxI) {}
};

class util::pid
{
    private:

        double prevError,error,derivative,integralThreshold;
        double integral = 0;
        util::pidConstants constants;

    public:

        pid(util::pidConstants cons, double error) : constants(cons), prevError(error){}

        double out(double error)
        {
            //eyes
            integral = error <= constants.tolerance ? 0 : error < integralThreshold ? integral + error : integral;

            if(integral > constants.maxIntegral)
            {
                integral = 0;
            }

            //dee
            derivative = error - prevError;
            prevError = error;

            return(error * constants.p  + integral * constants.i + derivative * constants.d);
        }

        void update(util::pidConstants cons)
        {
            constants = cons;
        }
};

class util::movingAverage
{
    private:
        int size;
        double sum;
        double integral;
        std::vector<double> window;

    public:
        movingAverage(int Size) : size(Size)
        {
            for (int i = 0; i < size; i++)
            {
                window.push_back(0);
                integral += pow(i * 1.0/size * 1.0,2);
            }
        }
        
        void push(double val)
        {
            for (int i = 0; i < size-1; i++)
            {
                window[i] = window[i+1];
            }

            window[size - 1] = val;
            
        }

        double simpleAverage()
        {
            double average = 0;
            for(int i = 0; i < size; i++)
            {
                average += window[i];
            }
            
            return(average/size);
        }

        double expAverage()
        {
            double average = 0;
            for(int i = 1; i != size; i++)
            {
                average += window[i] * pow((i * 1.0)/(size * 1.0),2);
            }
            
            return(average/integral);
        }
};

double util::dtr(double input) //NOLINT
{
  return(PI * input/180);
}

double util::rtd(double input) //NOLINT
{
  return(input * 180/PI);
}

double util::distToPoint(util::coordinate p1, util::coordinate p2) //NOLINT
{
    return( sqrt( pow((p2.x-p1.x),2) + pow((p2.y-p1.y), 2)));
}

double util::mod(double a, double b) //NOLINT
{
  return fmod(360-std::abs(a), b);
}

double util::absoluteAngleToPoint(util::coordinate pos, util::coordinate point) //NOLINT
{
    double t;

    try
    { 
        // t = -atan2(pos.y-point.y,pos.x - point.x) - PI/2;
        t = atan2(point.x - pos.x, point.y - pos.y);
    }

    catch(...)
    {
        t = PI/2;
    }
    
    t = util::rtd(t);

    // -270 - 90
    
    // if(t < -180)
    // {
    //     t = 90 + (270 - fabs(t));
    // }

    //-180 - 180

    t = -t;
    t = t >= 0 ? t :  180 + 180+t;
    return (t);
}

double util::imuToRad(double heading) //NOLINT
{
    // could be like shifted over? idk
    return (heading < 180) ? dtr(heading) : dtr(-(heading - 180));
}

#endif
