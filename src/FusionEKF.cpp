#include "FusionEKF.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);

    //measurement covariance matrix - laser
    R_laser_ <<
             0.0225, 0,
            0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ <<
             0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

    F_ = MatrixXd(4, 4);

    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;

    /**
    TODO:
      * Finish initializing the FusionEKF.
      * Set the process and measurement noises
    */
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() = default;

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        // first measurement
        auto x = VectorXd(4);

        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            x << Tools::PolarToCartesian(measurement_pack.raw_measurements_[0],
                                         measurement_pack.raw_measurements_[1],
                                         measurement_pack.raw_measurements_[2]);
        } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            x << measurement_pack.raw_measurements_[0], // px
                    measurement_pack.raw_measurements_[1], // py
                    0,                                     // vx = unknown
                    0;                                     // vy = unknown
        }


        auto P_ = MatrixXd(4, 4);
             P_ << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1000, 0,
                0, 0, 0, 1000;

        ekf_.Init(x,P_);

        // done initializing, no need to predict or update
        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/


    const auto dT = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0f;
    previous_timestamp_ = measurement_pack.timestamp_;

    const float noise_ax = 9;
    const float noise_ay = 9;

    //Modify the F matrix so that the time is integrated
    F_(0, 2) = dT;
    F_(1, 3) = dT;

    auto dt_2 = dT * dT;
    auto dt_3 = dt_2 * dT;
    auto dt_4 = dt_3 * dT;

    MatrixXd Q_ = MatrixXd(4, 4);
    Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
            0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
            dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
            0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;


    ekf_.Predict(F_, Q_);

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        auto H_ = Tools::CalculateJacobian(ekf_.x_);
        ekf_.UpdateEKF(measurement_pack.raw_measurements_, H_, R_radar_);
    } else {
        ekf_.Update(measurement_pack.raw_measurements_, H_laser_, R_laser_);
    }

}
