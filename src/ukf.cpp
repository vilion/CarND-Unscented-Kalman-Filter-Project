#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI/8;

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  /***************************************************************
  *      Inittialization structure similar to EKF project        *
  ****************************************************************/

  if(!is_initialized){
    // TODO Initialize x_, P_, previous_time, anything else needed.
    
    if(measurement_pack.sensor_type_ == MeasurementPackage::LASER){
      // TODO Initialize here
    } else if(measurement_pack.sensor_type_ == MeasurementPackage::RADER){
      // TODO Initialize here
    }
    
    // Initialize anything else here (e.g P_, anything else needed)
    previous_t = meas_package.timestamp_;
    is_initialized = true;
    return;
  }

  /***************************************************************
  *      Control structure similar to EKF project        *
  ****************************************************************/

  delta_t = (meas_package.timestamp_ - previous_t) / 1000000.0;
  Prediction(delta_t);
      
  if(meas_package.sensor_type_ == MeasurementPackage::LASER){
    UpdateLidar(meas_package);
  } else if(meas_package.sensor_type_ == MeasurementPackage::LASER){
    UpdateRader(meas_package);
  }
  previous_t = measurement.timestamp_;

}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  /***************************************************************
  *      Create Augmented Sigma Point                            *
  ****************************************************************/

  // Lesson 7, section 18: Agmenteation Assignment 2

  // create augmented mean state
  int n_aug = 7:

  VectorXd x_aug(7);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  MatrixXd P_aug(7,7);
  P_aug.fill(0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a*std_a;
  P_aug(6,6) = std_yawdd*std_yawdd;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  MatrixXd Xsig_aug(n_aug,2*n_aug+1);
  Xsig_aug.col(0) = x_aug;
  int lambda = 3 - n_aug;
  for(int ii = 0; ii < n_aug; ii++){
    Xsig_aug(ii+1)       = x_aug + sqrt(lambda+n_aug) * L;
    Xsig_aug(ii+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L;
  }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
}
