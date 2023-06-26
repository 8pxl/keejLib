/*
    MIT License

    Copyright (c) 2020 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#pragma once

#define _USE_MATH_DEFINES
#include <cfloat>
#include <cmath>
#include <set>

std::set<double> solveCub(double a, double b, double c, double d)
// Calculate all roots of a*x^3 + b*x^2 + c*x + d = 0
{
    std::set<double> roots;

    constexpr double cos120 = -0.50;
    constexpr double sin120 = 0.866025403784438646764;

    if (fabs(d) < DBL_EPSILON)
    {
        // First solution is x = 0
        roots.insert(0.0);

        // Converting to a quadratic equation
        d = c;
        c = b;
        b = a;
        a = 0.0;
    }

    if (fabs(a) < DBL_EPSILON)
    {
        if (fabs(b) < DBL_EPSILON)
        {
            // Linear equation
            if (fabs(c) > DBL_EPSILON)
                roots.insert(-d / c);
        }
        else
        {
            // Quadratic equation
            double discriminant = c * c - 4.0 * b * d;
            if (discriminant >= 0)
            {
                double inv2b = 1.0 / (2.0 * b);
                double y = sqrt(discriminant);
                roots.insert((-c + y) * inv2b);
                roots.insert((-c - y) * inv2b);
            }
        }
    }
    else
    {
        // Cubic equation
        double inva = 1.0 / a;
        double invaa = inva * inva;
        double bb = b * b;
        double bover3a = b * (1.0 / 3.0) * inva;
        double p = (3.0 * a * c - bb) * (1.0 / 3.0) * invaa;
        double halfq = (2.0 * bb * b - 9.0 * a * b * c + 27.0 * a * a * d) * (0.5 / 27.0) * invaa * inva;
        double yy = p * p * p / 27.0 + halfq * halfq;

        if (yy > DBL_EPSILON)
        {
            // Sqrt is positive: one real solution
            double y = sqrt(yy);
            double uuu = -halfq + y;
            double vvv = -halfq - y;
            double www = fabs(uuu) > fabs(vvv) ? uuu : vvv;
            double w = (www < 0) ? -pow(fabs(www), 1.0 / 3.0) : pow(www, 1.0 / 3.0);
            roots.insert(w - p / (3.0 * w) - bover3a);
        }
        else if (yy < -DBL_EPSILON)
        {
            // Sqrt is negative: three real solutions
            double x = -halfq;
            double y = sqrt(-yy);
            double theta;
            double r;
            double ux;
            double uyi;
            // Convert to polar form
            if (fabs(x) > DBL_EPSILON)
            {
                theta = (x > 0.0) ? atan(y / x) : (atan(y / x) + M_PI);
                r = sqrt(x * x - yy);
            }
            else
            {
                // Vertical line
                theta = M_PI / 2.0;
                r = y;
            }
            // Calculate cube root
            theta /= 3.0;
            r = pow(r, 1.0 / 3.0);
            // Convert to complex coordinate
            ux = cos(theta) * r;
            uyi = sin(theta) * r;
            // First solution
            roots.insert(ux + ux - bover3a);
            // Second solution, rotate +120 degrees
            roots.insert(2.0 * (ux * cos120 - uyi * sin120) - bover3a);
            // Third solution, rotate -120 degrees
            roots.insert(2.0 * (ux * cos120 + uyi * sin120) - bover3a);
        }
        else
        {
            // Sqrt is zero: two real solutions
            double www = -halfq;
            double w = (www < 0.0) ? -pow(fabs(www), 1.0 / 3.0) : pow(www, 1.0 / 3.0);
            // First solution
            roots.insert(w + w - bover3a);
            // Second solution, rotate +120 degrees
            roots.insert(2.0 * w * cos120 - bover3a);
        }
    }
    return roots;
}