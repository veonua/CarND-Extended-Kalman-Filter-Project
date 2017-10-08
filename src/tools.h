#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

namespace Tools {
    /**
    * A helper method to calculate RMSE.
    */
    VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

    /**
    * A helper method to calculate Jacobians.
    */
    MatrixXd CalculateJacobian(const VectorXd &x_state);

    /**
    * A helper method to convert polar coordinates from radar to cartesian coordinates + cartesian speed.
    */
    const VectorXd PolarToCartesian(double rho, double phi, double rhodot);

    /**
    * A helper method to convert carthesian coordinates + carthesian speed from state to polar coordinates + rho_dot
    */
    VectorXd CartesianToPolar(double px, double py, double vx, double vy);
};

#endif /* TOOLS_H_ */
