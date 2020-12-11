//
// Created by cagdas on 25.10.2020.
//

#ifndef KINEMATICS_DYNAMICS_CALCULATIONS_H
#define KINEMATICS_DYNAMICS_CALCULATIONS_H

#include <math.h>

#include "TireCalculations.h"
#include "Eigen-3.3/Eigen/Core"

#define MASS 1700.0f // mass of the vehicle
#define I_Z  2900.0f // Inertia of the vehicle
#define L_f 1.5f // length from CoG to front axle in meters
#define L_r 1.4f // length from CoG to back axle in meters

// Return the next state.
Eigen::VectorXd globalKinematicsDynamics(const Eigen::VectorXd &state,
                         const Eigen::VectorXd &actuators, double dt, double vx);

#endif //KINEMATICS_DYNAMICS_CALCULATIONS_H
