//
// Created by cagdas on 25.10.2020.
//

#ifndef TIRECALCULATIONS_H
#define TIRECALCULATIONS_H

#include <math.h>

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

#endif //TIRECALCULATIONS_H
