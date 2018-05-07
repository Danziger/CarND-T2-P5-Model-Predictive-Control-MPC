#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <iostream>


using CppAD::AD;


// CONSTANTS:

// We want T to be 1 seconds, so we could use DT = 0.1 seconds and N = 10;
const double DT = 0.1; // DT >= LAG
const size_t N = 10;

// The solver takes all the state variables and actuator variables in a
// single vector. Thus, we should establish where one variable starts and
// another ends to make our lifes easier:
const size_t START_X = 0;
const size_t START_Y = START_X + N;
const size_t START_PSI = START_Y + N;
const size_t START_V = START_PSI + N;
const size_t START_CTE = START_V + N;
const size_t START_EPSI = START_CTE + N;
const size_t START_DELTA = START_EPSI + N;
const size_t START_A = START_DELTA + N - 1;


// FG_eval CLASS DEFINITION:

class FG_eval {

public:

    // Coefficients of the fitted polynomial:
    Eigen::VectorXd coeffs_;

    // Weights for the cost calculation:
    unsigned int W_CTE_;
    unsigned int W_EPSI_;
    unsigned int W_SPEED_;
    unsigned int W_DELTA_;
    unsigned int W_ACC_;
    unsigned int W_DDELTA_;
    unsigned int W_DACC_;

    // Reference speed:
    unsigned int V_REF_;

    // LF param:
    double LF_;

    FG_eval(
        Eigen::VectorXd coeffs,
        const vector<unsigned int> &WEIGHTS,
        const unsigned int V_REF,
        const size_t N,
        const double DT,
        const double LF
    ) {
        // Coefficients of the fitted polynomial:
        coeffs_ = coeffs;

        // Weights for the cost calculation:
        W_CTE_ = WEIGHTS[0];
        W_EPSI_ = WEIGHTS[1];
        W_SPEED_ = WEIGHTS[2];
        W_DELTA_ = WEIGHTS[3];
        W_ACC_ = WEIGHTS[4];
        W_DDELTA_ = WEIGHTS[5];
        W_DACC_ = WEIGHTS[6];

        // Reference speed:
        V_REF_ = V_REF;

        // LF param:
        LF_ = LF;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator() (
        ADvector& fg,
        const ADvector& vars
    ) {
        // `fg` is a vector containing the cost and constraints.
        // `vars` is a vector containing the variable values (state & actuators).

        CppAD::AD<double> cost = 0;

        // COSTS:
        // Define the cost related to the reference state and anything you think may be beneficial:

        // Reference State Cost:

        for (size_t t = 0; t < N; ++t) {
            cost += W_CTE_ * CppAD::pow(vars[START_CTE + t], 2);
            cost += W_EPSI_ * CppAD::pow(vars[START_EPSI + t], 2);

            // IDEA/TODO: Adjust V_REF_ dinamically based on the average of the M last
            // steering angles, although this should not be necessary.
            cost += W_SPEED_ * CppAD::pow(vars[START_V + t] - V_REF_, 2);
        }

        // Control Cost (minimize the use of actuators):

        for (size_t t = 0; t < N - 1; ++t) {
            cost += W_DELTA_ * CppAD::pow(vars[START_DELTA + t], 2);
            cost += W_ACC_ * CppAD::pow(vars[START_A + t], 2);
        }

        // Control Cost (minimize the value gap between sequential actuations to achieve temporal smoothness):

        for (size_t t = 0; t < N - 2; ++t) {
            cost += W_DDELTA_ * CppAD::pow(vars[START_DELTA + t + 1] - vars[START_DELTA + t], 2);
            cost += W_DACC_ * CppAD::pow(vars[START_A + t + 1] - vars[START_A + t], 2);
        }

        // The cost is stored is the first element of fg, fg[0]:
        fg[0] = cost;

        // CONSTRAINTS:
        // In this section you'll setup the model constraints.

        // Initial constraints:
        // We add 1 to each of the starting indices due to cost being located at index 0 of fg:
        fg[1 + START_X] = vars[START_X];
        fg[1 + START_Y] = vars[START_Y];
        fg[1 + START_PSI] = vars[START_PSI];
        fg[1 + START_V] = vars[START_V];
        fg[1 + START_CTE] = vars[START_CTE];
        fg[1 + START_EPSI] = vars[START_EPSI];

        // The rest of the constraints:
        for (size_t t = 1; t < N; ++t) {
            // State at t + 1:
            AD<double> x1 = vars[START_X + t];
            AD<double> y1 = vars[START_Y + t];
            AD<double> psi1 = vars[START_PSI + t];
            AD<double> v1 = vars[START_V + t];
            AD<double> cte1 = vars[START_CTE + t];
            AD<double> epsi1 = vars[START_EPSI + t];

            // State at t:
            AD<double> x0 = vars[START_X + t - 1];
            AD<double> y0 = vars[START_Y + t - 1];
            AD<double> psi0 = vars[START_PSI + t - 1];
            AD<double> v0 = vars[START_V + t - 1];
            AD<double> cte0 = vars[START_CTE + t - 1];
            AD<double> epsi0 = vars[START_EPSI + t - 1];

            // Actuators at t:
            AD<double> delta0 = vars[START_DELTA + t - 1];
            AD<double> a0 = vars[START_A + t - 1];

            // The idea here is to constraint this value to be 0.
            // NOTE: The use of `AD<double>` and use of `CppAD`!
            // This is also CppAD can compute derivatives and pass these to the solver.

            // For a polynomial of 2nd degree:
            // AD<double> f0 = coeffs_[0] + coeffs_[1] * x0;
            // AD<double> psides0 = CppAD::atan(coeffs_[1]);

            // For a polynomial of nth degree:

            AD<double> f0 = 0.0; // f(x0)

            for (int i = 0; i < coeffs_.size(); i++) {
                f0 += coeffs_[i] * CppAD::pow(x0, i);
            }

            AD<double> psides0 = 0.0; // f'(x0)

            for (int i = 1; i < coeffs_.size(); i++) {
                psides0 += i * coeffs_[i] * CppAD::pow(x0, i - 1);
            }

            psides0 = CppAD::atan(psides0);

            fg[1 + START_X + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * DT);
            fg[1 + START_Y + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * DT);
            fg[1 + START_PSI + t] = psi1 - (psi0 + v0/LF_ * delta0 * DT);
            fg[1 + START_V + t] = v1 - (v0 + a0 * DT);
            fg[1 + START_CTE + t] = cte1 - (f0 - y0 + v0 * CppAD::sin(epsi0) * DT);

            // A sign has ben changed in this list one ( + v0/LF_) to work properly
            // on the emulator:
            fg[1 + START_EPSI + t] = epsi1 - (psi0 - psides0 - v0/LF_ * delta0 * DT);
        }
    }
};


// MPC CLASS DEFINITION:

MPC::MPC(
    const vector<unsigned int> &WEIGHTS,
    const vector<unsigned int> &MPC_PARAMS,
    const double MPH_2_MS,
    const double LF
) {

    WEIGHTS_ = WEIGHTS;
    V_REF_ = MPC_PARAMS[0];
    AVG_N_ = MPC_PARAMS[1];
    MPH_2_MS_ = MPH_2_MS;
    LF_ = LF;

}

MPC::~MPC() {}

vector<double> MPC::solve(
    const Eigen::VectorXd state,
    const Eigen::VectorXd coeffs
) {
    typedef CPPAD_TESTVECTOR(double) Dvector;

    const double x = state[0];
    const double y = state[1];
    const double psi = state[2];
    const double v = state[3];
    const double cte = state[4];
    const double epsi = state[5];

    // Number of independent variables (includes both states and inputs):
    // For example, if the state is a 4 element vector, the actuators is a 2 element vector
    // and there are 10 timesteps. The number of variables is: 4 * 10 + 2 * 9

    // N timesteps = N - 1 actuations:
    size_t n_vars = N * 6 + (N - 1) * 2;

    // Number of constraints:
    size_t n_constraints = N * 6;

    // Initial value of the independent variables.
    Dvector vars(n_vars);

    // Should be 0 except for the initial values:
    for (size_t i = 0; i < n_vars; ++i) {
        vars[i] = 0.0;
    }

    // Set the initial variable values:
    vars[START_X] = x;
    vars[START_Y] = y;
    vars[START_PSI] = psi;
    vars[START_V] = v;
    vars[START_CTE] = cte;
    vars[START_EPSI] = epsi;

    // Lower and upper limits for x:
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values:
    for (size_t i = 0; i < START_DELTA; ++i) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // IDEA/TODO: Set max allowed CTE to drive safely:

    /*
    const double max_cte = min(1.75, abs(cte) + 0.75);

    for (size_t i = START_CTE; i < START_EPSI; ++i) {
        vars_lowerbound[i] = -max_cte;
        vars_upperbound[i] = max_cte;
    }
    */

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians):
    // NOTE: Feel free to change this to something else.
    for (size_t i = START_DELTA; i < START_A; ++i) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Acceleration/decceleration upper and lower limits:
    // NOTE: Feel free to change this to something else.
    for (size_t i = START_A; i < n_vars; ++i) {
        vars_lowerbound[i] = -MPH_2_MS_;
        vars_upperbound[i] = MPH_2_MS_;
    }

    // Lower and upper limits for constraints:
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);

    // Should be 0 except for the initial values:
    for (size_t i = 0; i < n_constraints; ++i) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    // Set the initial variable values:

    constraints_lowerbound[START_X] = x;
    constraints_lowerbound[START_Y] = y;
    constraints_lowerbound[START_PSI] = psi;
    constraints_lowerbound[START_V] = v;
    constraints_lowerbound[START_CTE] = cte;
    constraints_lowerbound[START_EPSI] = epsi;

    constraints_upperbound[START_X] = x;
    constraints_upperbound[START_Y] = y;
    constraints_upperbound[START_PSI] = psi;
    constraints_upperbound[START_V] = v;
    constraints_upperbound[START_CTE] = cte;
    constraints_upperbound[START_EPSI] = epsi;

    // Object that computes objective and constraints:
    FG_eval fg_eval(coeffs, WEIGHTS_, V_REF_, N, DT, LF_);

    // Options:
    // NOTE: You don't have to worry about these options.

    string options;

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
    // options += "Numeric max_cpu_time          1.5\n";

    // Solve the problem:

    CppAD::ipopt::solve_result<Dvector> solution;

    CppAD::ipopt::solve<Dvector, FG_eval>(
        options,
        vars,
        vars_lowerbound,
        vars_upperbound,
        constraints_lowerbound,
        constraints_upperbound,
        fg_eval,
        solution
    );

    // Check some of the solution values:

    const bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    const auto cost = solution.obj_value;

    if (!ok) {
        cout << endl << "There was an error calculating the solution." << endl << endl;
    }

    double total_steering = 0;
    double total_throttle = 0;

    for (size_t i = 0; i < AVG_N_; ++i) {
        total_steering += solution.x[START_DELTA + i];
        total_throttle += solution.x[START_A + i];
    }

    // {...} is shorthand for creating a vector:

    vector<double> result = {
        cost,
        total_steering / AVG_N_,
        total_throttle / AVG_N_,
    };

    for (size_t i = 0; i < N - 1; ++i) {
        result.push_back(solution.x[START_X + i]);
        result.push_back(solution.x[START_Y + i]);
    }

    return result;
}
