/****************************************************************************
 * MIT License
 *
 * Copyright (c) 2019 İsmail Çağdaş Yılmaz
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

#include "MPC.h"

using CppAD::AD;
using Eigen::VectorXd;

// Horizon is set to 7
// and the timestep evaluation frequency or evaluation
// period to 0.05.
size_t N = 20;
double dt = 0.05f; // sampling time

/*
 * Physical parameters of Car
 */
const double mass = 1700.0f; // in Kg
const double Iz = 2900.0f;   // moment of inertia in yaw axis
const double Lf = 1.5f;    // length from CoG to front axle in meters
const double Lr = 1.4f;    // length from CoG to front axle in meters
const double vx = 10.0f;     // constant velocity in x directions


/*
 * Tire parameters
 */
const double CY = 1.30f;
const double DY = 1.0 * mass * 9.81 / 2.0;
const double BY = 1.0f;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make program easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t vy_start = psi_start + N;
size_t r_start = vy_start + N;
size_t delta_start = r_start + N - 1;

class FG_eval
{
public:
    VectorXd coeffs;
    // Coefficients of the fitted polynomial.
    FG_eval(VectorXd coeffs, double xPos)
    {
        this->coeffs = coeffs;
        Xposition = xPos;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    // `fg` is a vector containing the cost and constraints.
    // `vars` is a vector containing the variable values (state & actuators).

    void operator()(ADvector& fg, const ADvector& vars) {
        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;

        // The part of the cost based on the reference state.
        for (size_t t = 0; t < N; ++t) {

            // std::cout << "x position: " << Xposition << std::endl;
            std::vector<double> referenceValues = references( Xposition + vx * t * dt );

            //std::cout << "psi ref: " << referenceValues[1] << std::endl;
            fg[0] += 5.0f * CppAD::pow(vars[y_start + t] - referenceValues[0], 2);
            fg[0] += 100.0f * CppAD::pow(vars[psi_start + t] - referenceValues[1], 2);
        }
        // Minimize the use of actuators.
        //for (int t = 0; t < N - 1; ++t) {
        //  fg[0] += CppAD::pow(vars[delta_start + t], 2);
        //}

        // Minimize the value gap between sequential actuations.
        for (size_t t = 0; t < N - 2; ++t) {
          fg[0] += 0.05f * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
        }

        //
        // Setup Constraints
        //

        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + vy_start] = vars[vy_start];
        fg[1 + r_start] = vars[r_start];

        // The rest of the constraints
        for (size_t t = 1; t < N; ++t) {
          // The state at time t+1 .
          AD<double> x1 = vars[x_start + t];
          AD<double> y1 = vars[y_start + t];
          AD<double> psi1 = vars[psi_start + t];
          AD<double> vy1 = vars[vy_start + t];
          AD<double> r1 = vars[r_start + t];

          // The state at time t.
          AD<double> x0 = vars[x_start + t - 1];
          AD<double> y0 = vars[y_start + t - 1];
          AD<double> psi0 = vars[psi_start + t - 1];
          AD<double> vy0 = vars[vy_start + t - 1];
          AD<double> r0 = vars[r_start + t - 1];

          // Only consider the actuation at time t.
          AD<double> delta0 = vars[delta_start + t - 1];

          // Here's `x` to get you started.
          // The idea here is to constraint this value to be 0.
          //
          // Recall the equations for the model:
          // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
          // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt

          AD<double> alphaR = -1.0 * CppAD::atan( (vy0 - r0 * Lr) / vx );
          AD<double> FyR = DY * CppAD::sin(C_Y * CppAD::atan(B_Y * alphaR));

          fg[1 + x_start + t] = x1 - (x0 + (vx * CppAD::cos(psi0) - vy1 * CppAD::sin(psi0)) * dt);
          fg[1 + y_start + t] = y1 - (y0 + (vx * CppAD::sin(psi0) + vy1 * CppAD::cos(psi0)) * dt);
          fg[1 + psi_start + t] = psi1 - (psi0 + r0 * dt);
          fg[1 + vy_start + t] = vy1 - (vy0 + (((1 / mass) *
                  ( (DY * CppAD::sin(C_Y * CppAD::atan(BY * (delta0 - CppAD::atan((vy0 + Lf * r0)/ vx))))) * CppAD::cos(psi0) + FyR)
                  - vx * r0) * dt));
          fg[1 + r_start + t] = r1 - (r0 + ( (1 / Iz) *
                  ( (DY * CppAD::sin(CY * CppAD::atan(BY * (delta0 - CppAD::atan((vy0 + Lf * r0)/ vx))))) * CppAD::cos(psi0) * Lf - Lr * FyR) * dt ));
        }
    }

private:
    double Xposition;
};

//
// MPC class definition
//

MPC::MPC() {}
MPC::~MPC() {}

std::vector<double> MPC::Solve(const VectorXd &x0, const VectorXd &coeffs)
{
    typedef CPPAD_TESTVECTOR(double) Dvector;

    double x = x0[0];
    double y = x0[1];
    double psi = x0[2];
    double vy = x0[3];
    double r = x0[4];

    // number of independent variables
    // N timesteps == N - 1 actuations
    size_t n_vars = N * 5 + (N - 1) * 1;
    // Number of constraints
    size_t n_constraints = N * 5;

    // Initial value of the independent variables.
    // Should be 0 except for the initial values.
    Dvector vars(n_vars);
    for (size_t i = 0; i < n_vars; ++i)
    {
      vars[i] = 0.0;
    }
    // Set the initial variable values
    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[vy_start] = vy;
    vars[r_start] = r;

    // Lower and upper limits for x
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (size_t i = 0; i < delta_start; ++i) {
      vars_lowerbound[i] = -1.0e19;
      vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (size_t i = delta_start; i < n_vars; ++i) {
      vars_lowerbound[i] = -20.0 * M_PI / 180.0;
      vars_upperbound[i] = 20.0 * M_PI / 180.0;
    }

    // Lower and upper limits for constraints
    // All of these should be 0 except the initial
    // state indices.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (size_t i = 0; i < n_constraints; ++i) {
      constraints_lowerbound[i] = 0;
      constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[vy_start] = vy;
    constraints_lowerbound[r_start] = r;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[vy_start] = vy;
    constraints_upperbound[r_start] = r;

    // Object that computes objective and constraints
    FG_eval fg_eval(coeffs, getXposition());

    // options
    std::string options;
    options += "Integer print_level  0\n";
    //options += "String  sb           no\n";
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    //
    // Check some of the solution values
    //
    //bool ok = true;
    //ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;
    return {solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[vy_start + 1],
          solution.x[r_start + 1], solution.x[delta_start]};
}

void MPC::setXposition(const double &x)
{
    xPosition = x;
}

double MPC::getXposition() const
{
    return xPosition;
}