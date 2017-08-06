#ifndef MPC_H
#define MPC_H

#include <vector>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

//
// Helper functions
//
// constexpr double pi() { return M_PI; }
// double deg2rad(double x) { return x * pi() / 180; }
// double rad2deg(double x) { return x * 180 / pi(); }

class MPC {
 public:

  // Letancy
  int latency;

  // predicted trajectory
  vector<double> trajectory_xs;
  vector<double> trajectory_ys;

  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(const Eigen::VectorXd &state, const Eigen::VectorXd &coeffs);

  // Calculate new_state after latency
  Eigen::VectorXd StateAfterLatency(const Eigen::VectorXd &state, double delta, double a);
};

#endif /* MPC_H */
