//
// Created by cagdas on 25.10.2020.
//

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



