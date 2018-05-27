#ifndef MPC_H
#define MPC_H

#include <vector>
#include "common/Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
private:

    vector<unsigned int> WEIGHTS_;
    unsigned int V_REF_;
    size_t AVG_N_;
    double MPH_2_MS_;
    double LF_;

public:

  MPC(
      const vector<unsigned int> &WEIGHTS,
      const vector<unsigned int> &MPC_PARAMS,
      const double MPH_2_MS,
      const double LF
  );

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the average of the first N actuations.
  vector<double> solve(
      const Eigen::VectorXd state,
      const Eigen::VectorXd coeffs
  );

};

#endif /* MPC_H */
