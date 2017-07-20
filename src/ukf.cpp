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

  // Lesson 7, section 18: Generate Augmented Sigma point Assignment 2

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
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  // create augmented sigma points
  MatrixXd Xsig_aug(n_aug,2*n_aug+1);
  Xsig_aug.col(0) = x_aug;
  int lambda = 3 - n_aug;
  for(int ii = 0; ii < n_aug; ii++){
    Xsig_aug(ii+1)       = x_aug + sqrt(lambda+n_aug) * L.col(i);
    Xsig_aug(ii+1+n_aug) = x_aug - sqrt(lambda+n_aug) * L.col(i);
  }


  /***************************************************************
  *      Predict Sigma Point                            *
  ****************************************************************/

  // Lesson 7, section 21: Predict Augmented Sigmapoint Assignment 2
  // predict sigma points
  for(int jj =0; jj < 2*n_aug+1; jj++){
    // extract values for better readability
    double p_x = Xsig_aug(0,jj);
    double p_x = Xsig_aug(1,jj);
    double v = Xsig_aug(2,jj);
    double yaw = Xsig_aug(3,jj);
    double yawd = Xsig_aug(4,jj);
    double nu_a = Xsig_aug(5,jj);
    double nu_yawdd = Xsig_aug(6,jj);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if(fabs(yawd)>0.001){
      px_p = p_x + v/yawd * (sin(yaw + yawd*delta_t) - sin(yaw));
      py_p = p_y + v/yawd * (cos(yaw) - cos(yaw+yawd*delta_t));
    } else {
      px_p = p_x + v*delta_t*cos(yaw);
      py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    // write predicted sigma point into right column
    Xsig_pred(0,jj) = px_p;
    Xsig_pred(1,jj) = py_p;
    Xsig_pred(2,jj) = v_p;
    Xsig_pred(3,jj) = yaw_p;
    Xsig_pred(4,jj) = yawd_p;
  }

  /***************************************************************
  *      Predict mean and covariance
  ****************************************************************/

  // Lesson 7, section 24: Predict Mean and Covariance Assignment 2
  
  // set weights
  double weight_0 = lambda/(lambda+n_aug);
  weights(0) = weight_0;
  for(int kk=1; kk<2*n_aug+1; kk++){ // 2n+1 weights
    double weight = 0.5/(n_aug+lambda);
    weights(kk) = weight;
  }

  // predicted state mean
  x_.fill(0.0);
  for(int ll = 0; ll < 2 * n_aug + 1; ll++){
    x_ = x_ + weights(ll)* Xsig_pred.col(ll);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for(int mm = 0; mm < 2 * n_aug + 1; mm++){
    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    // angle normalization
    while(x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
    while(x_diff(3)<M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights(mm) * x_diff * x_diff.transpose();
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
