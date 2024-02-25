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

#include "TireCalculations.h"

TireCalculations::TireCalculations(const double delta_, const double  vx_, const double vy_, const double r_)
    : delta(delta_), vx(vx_), vy(vy_), r(r_)
{
    setAlphaFront(delta, vx, vy, r);
    setAlphaRear(vx, vy, r);
}

TireCalculations::~TireCalculations()
{
}

void TireCalculations::setAlphaFront(const double &delta_, const double &vx_, const double &vy_, const double &r_)
{
    alphaF = delta_ - atan2( (vy_ + L_f * r_), vx_);
    setFyFront( alphaF );
}

void TireCalculations::setAlphaRear(const double &vx_, const double &vy_, const double &r_)
{
    alphaR = -atan2((vy_ - r_ * L_r), vx_ );
    setFyRear( alphaR );
}

void TireCalculations::setFyFront(const double &alphaF_)
{
    double D_Y = (1.0 * MASS * 9.81) / 2.0;
    FyF = D_Y  * sin( C_Y * atan( B_Y * alphaF_));
}

void TireCalculations::setFyRear(const double &alphaR_)
{
    double D_Y = (1.0 * MASS * 9.81) / 2.0;
    FyR = D_Y  * sin( C_Y * atan( B_Y * alphaR_));
}

double TireCalculations::getAlphaFront() const
{
    return alphaF;
}

double TireCalculations::getAlphaRear() const
{
    return alphaR;
}

double TireCalculations::getFyFront() const
{
    return FyF;
}

double TireCalculations::getFyRear() const
{
    return FyR;
}



