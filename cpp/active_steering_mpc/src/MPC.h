#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

class MPC {
 public:
  MPC();

  virtual ~MPC();

  void setXposition( const double &x );
  double getXposition() const;

  // Solve the model given an initial state.
  // Return the next state and actuations as a vector.
  std::vector<double> Solve(const Eigen::VectorXd &x0,
                            const Eigen::VectorXd &coeffs);

private:
    double xPosition;
};

#endif  // MPC_H
