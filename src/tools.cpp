#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

namespace Tools {

    VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
                           const vector<VectorXd> &ground_truth) {
        VectorXd rmse(4);
        rmse << 0, 0, 0, 0;

        // check the validity of the following inputs:
        //  * the estimation vector size should not be zero
        //  * the estimation vector size should equal ground truth vector size
        if (estimations.size() != ground_truth.size()
            || estimations.empty()) {
            cout << "Invalid estimation or ground_truth data" << endl;
            return rmse;
        }

        //accumulate squared residuals
        for (unsigned int i = 0; i < estimations.size(); ++i) {
            VectorXd residual = estimations[i] - ground_truth[i];
            residual = residual.array() * residual.array();
            rmse += residual;
        }
        rmse = rmse / estimations.size(); //calculate the mean
        rmse = rmse.array().sqrt(); //calculate the squared root
        return rmse;
    }

    MatrixXd CalculateJacobian(const VectorXd &x_state) {
        MatrixXd Hj(3, 4);
        //recover state parameters
        float px = x_state(0);
        float py = x_state(1);
        float vx = x_state(2);
        float vy = x_state(3);

        //pre-compute a set of terms to avoid repeated calculation
        float c1 = px * px + py * py;
        float c2 = sqrt(c1);
        float c3 = (c1 * c2);

        //check division by zero
        if (fabs(c1) < 0.0001) {
            cout << "CalculateJacobian () - Error - Division by Zero" << endl;
            return Hj;
        }

        //compute the Jacobian matrix
        Hj << (px / c2), (py / c2), 0, 0,
                -(py / c1), (px / c1), 0, 0,
                py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

        return Hj;
    }

    const VectorXd PolarToCartesian(double rho, double phi, double rhodot) {
        double px = rho * cos(phi);
        double py = rho * sin(phi);
        double vx = rhodot * sin(phi);
        double vy = rhodot * cos(phi);

        VectorXd result(4);
        result << px, py, vx, vy;
        return result;
    }

    VectorXd CartesianToPolar(double px, double py, double vx, double vy) {
        double rho = sqrt(px * px + py * py);
        double phi = atan2(py, px);
        double rho_dot = (fabs(rho) < 0.0001) ? 0 : (px * vx + py * vy) / rho;

        VectorXd polar(3);
        polar << rho, phi, rho_dot;
        return polar;
    }

}
