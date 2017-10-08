#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
public:
    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    /**
     * Constructor
     */
    KalmanFilter();

    /**
     * Destructor
     */
    virtual ~KalmanFilter();

    /**
     * Prediction Predicts the state and the state covariance
     * using the process model
     * @param delta_T Time between k and k+1 in s
     */
    void Predict(const Eigen::MatrixXd &F_, const Eigen::MatrixXd &Q_);

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
     */
    void Update(const Eigen::VectorXd &z,const  Eigen::MatrixXd &H,const Eigen::MatrixXd &R);

    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     */
    void UpdateEKF(const Eigen::VectorXd &z, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);

    void Init(Eigen::MatrixXd x, Eigen::MatrixXd p);

private:
    void inline UpdateState(const Eigen::VectorXd &y, const Eigen::MatrixXd &H, const Eigen::MatrixXd &R);

    // identity X
    Eigen::MatrixXd I_;

};

#endif /* KALMAN_FILTER_H_ */
