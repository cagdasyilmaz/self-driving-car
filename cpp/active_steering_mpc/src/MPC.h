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

#pragma once

#include "TireCalculations.h"
#include "ReferenceGenerator.h"
#include <eigen3/Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include <cmath>

class MPC {
public:
    MPC();
    virtual ~MPC();

    void setXposition(const double &x);
    double getXposition() const;

    // Solve the model given an initial state.
    // Return the next state and actuations as a vector.
    std::vector<double> Solve(const Eigen::VectorXd &x0,
                              const Eigen::VectorXd &coeffs);

private:
    double xPosition;
};

