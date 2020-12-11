//
// Created by cagdas on 25.10.2020.
//

#include "ReferenceGenerator.h"
#include <cmath>

using namespace std;

vector<double> references( const double &xPosition )
{
    vector<double> references {};

    const double shape = 2.4f;
    const double d_x_1 = 25.0f;
    const double d_x_2 = 21.95f;
    const double d_y_1 = 4.05f;
    const double d_y_2 = 5.7f;
    const double X_s_1 = 27.19f;
    const double X_s_2 = 56.46f;

    double z_1 = 0.0f;
    double z_2 = 0.0f;

    z_1 = (shape / d_x_1) * (xPosition - X_s_1) - (shape / 2);
    z_2 = (shape / d_x_2) * (xPosition - X_s_2) - (shape / 2);

    double Y_ref = 0.0f;
    Y_ref = ((d_y_1 / 2) * (1 + tanh(z_1)) - (d_y_2 / 2) * (1 + tanh(z_2)));

    double psi_ref = 0.0f;

    psi_ref = atan( d_y_1 * pow((1 / cosh(z_1)), 2) * (1.2 / d_x_1) -
            d_y_2 * pow((1 / cosh(z_2)), 2) * (1.2 / d_x_2) );

    references.push_back(Y_ref);
    references.push_back(psi_ref);

    return references;
}