/****************************************************************************
 * MIT License
 *
 * Copyright (c) 2020 İsmail Çağdaş Yılmaz
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ****************************************************************************/

#pragma once

#include <cmath>

#define MASS 1700.0f // mass of the vehicle
#define I_Z  2900.0f // Inertia of the vehicle
#define L_f 1.5f // length from CoG to front axle in meters
#define L_r 1.4f // length from CoG to back axle in meters

/*
 * Tire Model Constants
 */
//#define A1 -22.1f
//#define A2 1011.0f
//#define A3 1078.0f
//#define A4 1.82.0f
//#define A5 0.208f
//#define A6 0.000f
//#define A7 -0.354f
//#define A8 0.707f

#define C_Y 1.30f
#define B_Y 1.0f

class TireCalculations
{
public:
    TireCalculations(const double delta_, const double  vx_, const double vy_, const double r_);

    ~TireCalculations();

    void setAlphaFront(const double &delta_, const double &vx_, const double &vy_, const double &r_);
    void setAlphaRear(const double &vx_, const double &vy_, const double &r_);
    void setFyFront(const double &alphaF_);
    void setFyRear(const double &alphaR_);

    double getAlphaFront() const;
    double getAlphaRear() const;
    double getFyFront() const;
    double getFyRear() const;
private:
    double delta;
    double vx;
    double vy;
    double r;
    double alphaF;
    double alphaR;
    double FyF;
    double FyR;
};
