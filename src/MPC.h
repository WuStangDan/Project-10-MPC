#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();
  vector<double> Get_x_Vals();
  vector<double> Get_y_Vals();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

private:
  vector<double> x_vals;
  vector<double> y_vals;
};

#endif /* MPC_H */
