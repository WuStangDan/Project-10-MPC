#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 15.0;
double dt = 0.05; // 2 seconds prediction horizon.

double ref_v = 60.0; // Reference velocity that controller should obtain.

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;


// The solver has to take all state variables and actuator variables in one
// vector. In one long vector all variables will have a start and end position.
// The start positions are given below, the end positions will be the start
// positions + N-1 (the number of steps to the prediction horizon).
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
// The only exception are the two control inputs, since they will have one less
// step than the state (since we start and end prediction horizon on states).
// 1 - state-control, ... 19 - state, control, 20 - state.
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

    fg[0] = 0; // Cost value.

    // For all costs use variable*variable to punish straying far from
    // reference. The solver will try to pull fg[0] to zero.

    // Add cost for error for entire prediction horizon.
    for (int i = 0; i < N; i++) {
      // Cross track error and error psi references are zero.
      fg[0] += CppAD::pow(vars[cte_start + i], 2)*0.15;
      fg[0] += CppAD::pow(vars[epsi_start + i], 2)*150;
      fg[0] += CppAD::pow(vars[v_start + i] - ref_v, 2);
    }

    // Add cost for actuator inputs. Do not calculate control inputs for final
    // state since they will not be used.
    for (int i = 0; i < N - 1; i++) {
      fg[0] += CppAD::pow(vars[delta_start + i], 2)*100;
      fg[0] += CppAD::pow(vars[a_start + i], 2);
    }

    // Add cost function for change in actuator inputs (derivative) to punish
    // rapid changes of input.
    for (int i = 1; i < N - 1; i++) {
      fg[0] += CppAD::pow(vars[delta_start + i] - vars[delta_start + (i-1)], 2)*1300;
      fg[0] += CppAD::pow(vars[a_start + i] - vars[a_start + (i-1)], 2)*40;
    }

    // Setup global kinematic model.

    // Initial conditions. 1 + since 0 is the total cost.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The solver will try to pull fg[2 + ] values to zero. So all model
    // equations must be set equal to zero.
    for (int i = 0; i < N - 1; i++) {
      // State values at time t.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> cte0 = vars[cte_start + i]; // Not used.
      AD<double> epsi0 = vars[epsi_start + i];

      // State values at time t + 1.
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      // Actuators at time t (will act upon time t + 1).
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      // Reference track of f(x) (what the controller is trying to follow).
      // y = c0 + c1*x + c2*x^2 + c3*x^3.
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0*x0
                      + coeffs[3] * x0*x0*x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] +  2 * coeffs[2] * x0
                              + 3 * coeffs[3] * x0*x0);


      // State equations for global kinematic model (set to zero for solver):
      // 0 = x_[t+1] - (x[t] + v[t] * cos(psi[t]) * dt)
      // 0 = y_[t+1] - (y[t] + v[t] * sin(psi[t]) * dt)
      // 0 = v_[t+1] - (v[t] + a[t] * dt)
      // 0 = psi_[t+1] - (psi[t] + v[t] / Lf * delta[t] * dt)
      // Errors
      // 0 = cte[t+1] - (f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt)
      // 0 = epsi[t+1] - (psi[t] - psides[t] + v[t] * delta[t] / Lf * dt)
      fg[2 + x_start + i] = x1 - (x0 + (v0 * CppAD::cos(psi0) * dt));
      fg[2 + y_start + i] = y1 - (y0 + (v0 * CppAD::sin(psi0) * dt));
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + (v0 * delta0 * dt / Lf));

      fg[2 + cte_start + i] = cte1 - (f0 - y0 + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] = epsi1 - (psi0 - psides0 + (v0 * delta0 * dt / Lf));
    }
    // TODO: implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = N * 6 + (N-1) * 2;
  // Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Set the initial state varibles.
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;


  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // x, y, v, psi, cte, epsi, should have no contraints other than maximum
  // value of data type.
  for (int i = 0; i < (epsi_start + N); i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // Steering upper and lower limits. 15 degrees = 0.261799 radians.
  for (int i = delta_start; i < (delta_start + N-1); i++) {
    vars_lowerbound[i] = -0.139626;
    vars_upperbound[i] = 0.139626;
  }

  // Acceleration upper and lower limits.
  for (int i = a_start; i < (a_start + N-1); i++) {
    vars_lowerbound[i] = -1.0; // Maximum braking.
    vars_upperbound[i] = 1.0; // 70% of max throttle.
  }


  // Lower and upper limits for the constraints.
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  // Initial state.
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;



  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // save x and y states.
  x_vals.clear();
  y_vals.clear();

  for (int i = 0; i < N-1; i++) {
    x_vals.push_back(solution.x[x_start+1+i]);
    y_vals.push_back(solution.x[y_start+1+i]);
  }

  return {solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start],   solution.x[a_start]};
}

vector<double> MPC::Get_x_Vals()
{
  return x_vals;
}

vector<double> MPC::Get_y_Vals()
{
  return y_vals;
}
