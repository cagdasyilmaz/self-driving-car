#include <math.h>
#include <iostream>
#include <vector>
#include <ctime>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "KinematicsDynamicsCalculations.h"
#include "helpers.h"
#include "matplotlibcpp_.h"
#include "MPC.h"
#include "TireCalculations.h"
#include "ReferenceGenerator.h"
namespace plt = matplotlibcpp;

using Eigen::VectorXd;
using std::cout;
using std::cin;
using std::endl;
using std::vector;

//
// Helper functions
//
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main() {
    MPC mpc;        // define Model Predective Control Class
    int iters = 50; // maximum iteration

    VectorXd ptsx(2);
    VectorXd ptsy(2);
    ptsx << -200, 200;
    ptsy << -5, -5;

    // The polynomial is fitted to a straight line so a polynomial with
    // order 1 is sufficient.
    auto coeffs = polyfit(ptsx, ptsy, 1);


    // NOTE: Initial values of the state equations
    double X = 0.0f;
    double Y = 0.0f;
    double psi = 0.0f;
    double vy = 0.0f;
    double r = 0.0f;

    const double vx = 10.0f;  // constant velocity in x directions

    VectorXd state(5);
    state << X, Y, psi, vy, r;


    TireCalculations tire(0.0f, vx, vy, r);
    vector<double> alphaF_vals = {rad2deg(tire.getAlphaFront())};
    vector<double> alphaR_vals = {rad2deg(tire.getAlphaRear())};
    vector<double> FyF_vals = {tire.getFyFront()};
    vector<double> FyR_vals = {tire.getFyRear()};


    vector<double> simTime = {0.0};
    vector<double> x_vals = {state[0]};
    vector<double> y_vals = {state[1]};
    vector<double> psi_vals = {state[2]};
    vector<double> vy_vals = {state[3]};
    vector<double> r_vals = {state[4]};
    vector<double> delta_vals = {0};

    vector<double> solverTime_vals = {0.0};

    vector<double> delta_y_vals = {0};
    vector<double> delta_psi_vals = {0};

    vector<double> ref_y_vals = {0.0f};
    vector<double> ref_psi_vals = {0.0f};

    for (int  i = 0; i < 240; ++i) {
        cout << "Iteration " << i << endl;

        mpc.setXposition( X );

        std::vector<double> referenceValues = references( X );
        ref_y_vals.push_back(referenceValues[0]);
        ref_psi_vals.push_back(rad2deg(referenceValues[1]));

        clock_t begin = std::clock();

        auto vars = mpc.Solve(state, coeffs);

        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        //cout << "Solver time: " << elapsed_secs << endl;
        solverTime_vals.push_back(elapsed_secs);

        x_vals.push_back(vars[0]);
        y_vals.push_back(vars[1]);
        psi_vals.push_back(rad2deg(vars[2]));
        vy_vals.push_back(vars[3]);
        r_vals.push_back(rad2deg(vars[4]));

        delta_vals.push_back(rad2deg(vars[5]));

        delta_y_vals.push_back( y_vals[i+1] - y_vals[i] );
        delta_psi_vals.push_back( psi_vals[i+1] - psi_vals[i] );

        simTime.push_back(0.05 * (i + 1));

        tire.setAlphaFront(vars[5], vx, vars[3], vars[4]);
        tire.setAlphaRear(vx, vars[3], vars[4]);

        alphaF_vals.push_back(rad2deg(tire.getAlphaFront()));
        alphaR_vals.push_back(rad2deg(tire.getAlphaRear()));
        FyF_vals.push_back(tire.getFyFront());
        FyR_vals.push_back(tire.getFyRear());

        state << vars[0], vars[1], vars[2], vars[3], vars[4];
        /*
         * Write variable and optimized output
         */
        // cout << "x = " << vars[0] << endl;
        // cout << "y = " << vars[1] << endl;
        // cout << "psi = " << vars[2] << endl;
        // cout << "vy = " << vars[3] << endl;
        // cout << "r = " << vars[4] << endl;

        // cout << "delta = " << vars[5] << endl;

        X = vars[0];

        cout << endl;
    }

    // Plot values
    plt::figure();
    plt::subplot(3, 2, 1);
    plt::title("$\\psi$");
    plt::plot(simTime , psi_vals);
    plt::plot(simTime , ref_psi_vals, "r--");
    plt::ylabel("$\\psi[deg]$");

    plt::subplot(3, 2, 2);
    plt::title("$\\Delta\\Psi$");
    plt::plot(simTime , delta_psi_vals);

    plt::subplot(3, 2, 3);
    plt::title("Y");
    plt::plot(simTime, y_vals);
    plt::plot(simTime , ref_y_vals, "r--");

    plt::subplot(3, 2, 4);
    plt::title("$\\Delta{Y}$");
    plt::plot(simTime, delta_y_vals);

    plt::subplot(3, 2, 5);
    plt::title("$\\delta_{f}$");
    plt::plot(simTime, delta_vals);
    plt::xlabel("$Time[s]$");

    plt::subplot(3, 2, 6);
    plt::title("Path Following");
    plt::plot(x_vals, y_vals);
    plt::plot(x_vals, ref_y_vals, "r--" );

    plt::show();

    plt::figure();

    plt::subplot(3, 2, 1);
    plt::title("IPOPT Computation Time (sec)");
    plt::plot(simTime , solverTime_vals);

    plt::subplot(3, 2, 2);
    plt::title("$\\dot{\\psi}$");
    plt::plot(simTime , r_vals);

    plt::subplot(3, 2, 3);
    plt::title("Front slip angle, $\\alpha_f$ ");
    plt::plot(simTime , alphaF_vals);

    plt::subplot(3, 2, 4);
    plt::title("Rear slip angle, $\\alpha_r$");
    plt::plot(simTime , alphaR_vals);

    plt::subplot(3, 2, 5);
    plt::title("Front Corner Force, $F_{c_{f}}$");
    plt::plot(simTime , FyF_vals);

    plt::subplot(3, 2, 6);
    plt::title("Rear Corner Force, $F_{c_{r}}$");
    plt::plot(simTime , FyR_vals);

    plt::show();

    return 0;
}