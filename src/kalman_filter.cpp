#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() = default;

KalmanFilter::~KalmanFilter() = default;

void KalmanFilter::Predict(const Eigen::MatrixXd &F_, const Eigen::MatrixXd &Q_) {
    x_ = F_ * x_;
    auto Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::UpdateState(const Eigen::VectorXd &y, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R) {
    auto Ht = H.transpose();
    MatrixXd S = H * P_ * Ht + R;
    MatrixXd K = (P_ * Ht) * S.inverse();

    // Update state + state covariance
    x_ = x_ + (K * y);
    P_ = (I_ - K * H) * P_;
}

void KalmanFilter::Update(const VectorXd &z, const  Eigen::MatrixXd &H, const  Eigen::MatrixXd &R) {
    auto z_pred = H * x_;
    auto y = z - z_pred;

    UpdateState(y, H, R);
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R) {
    auto z_pred = Tools::CartesianToPolar(x_(0), x_(1), x_(2), x_(3));
    VectorXd y = z - z_pred;

    y[1] = atan2(sin(y[1]), cos(y[1])); // angle normalizaton
    UpdateState(y, H, R);
}

void KalmanFilter::Init(Eigen::MatrixXd x, Eigen::MatrixXd p) {
    x_ = x;
    auto x_size = x.size();
    I_ = MatrixXd::Identity(x_size, x_size);

    P_ = p;
}
