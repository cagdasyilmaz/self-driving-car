//
// Created by cagdas on 25.10.2020.
//

#include "KinematicsDynamicsCalculations.h"

using Eigen::VectorXd;

VectorXd globalKinematicsDynamics(const VectorXd &state,
                                  const VectorXd &actuators, double dt, double vx) {
    // Create a new vector for the next state.
    VectorXd next_state(state.size());

    // NOTE: state is [x, y, psi, v]
    auto x = state(0);
    auto y = state(1);
    auto psi = state(2);
    auto vy = state(3);
    auto r = state(4);

    // NOTE: actuators is [delta, a]
    auto delta = actuators(0);

    TireCalculations tire(delta, vx, vy, r);
    double alphaF = tire.getAlphaFront();
    double alphaR = tire.getAlphaRear();
    double FyF = tire.getFyFront();
    double FyR = tire.getFyRear();

    // Recall the equations for the model:
    // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt

    next_state(0) = x + (vx * cos(psi) - vy * sin(psi)) * dt;
    next_state(1) = y + (vx * cos(psi) - vy * sin(psi)) * dt;
    next_state(2) = psi + r * dt;
    next_state(3) = vy + ((1 / MASS) * (FyF * cos(delta) + FyR) - vx * r) * dt;
    next_state(4) = r + (1 / I_Z) * (L_f * FyF * cos(delta) - L_r * FyR) * dt;

    return next_state;
}