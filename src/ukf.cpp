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
  n_aug_ = 7;

  n_x_ = 5;

  lambda_ = 3 - n_aug_;

  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1,0,0,0,0,
      0,1,0,0,0,
      0,0,1,0,0,
      0,0,0,1,0,
      0,0,0,0,1;

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

  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  H_laser_ = MatrixXd(2, 5);
  H_laser_ << 1, 0, 0, 0, 0,
             0, 1, 0, 0, 0;

  // set weights
  weights_ = VectorXd(2*n_aug_+1);
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for(int kk=1; kk<2*n_aug_+1; kk++){ // 2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(kk) = weight;
  }

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

  if(!is_initialized_){
    // TODO Initialize x_, P_, previous_time, anything else needed.
    
    if(meas_package.sensor_type_ == MeasurementPackage::LASER){
      // TODO Initialize here
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
      x_(2) = 0;
      x_(3) = 0;
      x_(4) = 0;
    } else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
      // TODO Initialize here
      double ro = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      x_(0) = ro * cos(phi);
      x_(1) = ro * sin(phi);
      x_(2) = 0;
      x_(3) = 0;
      x_(4) = 0;
    }
    
    // Initialize anything else here (e.g P_, anything else needed)
    previouse_t_ = meas_package.timestamp_;
    is_initialized_ = true;
    P_ << 0.1,0,0,0,0,
        0,0.1,0,0,0,
        0,0,0.1,0,0,
        0,0,0,0.1,0,
        0,0,0,0,0.1;

    return;
  }

  /***************************************************************
  *      Control structure similar to EKF project        *
  ****************************************************************/

  double delta_t = (meas_package.timestamp_ - previouse_t_) / 1000000.0;
  Prediction(delta_t);
      
  if(meas_package.sensor_type_ == MeasurementPackage::LASER){
    UpdateLidar(meas_package);
  } else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
    UpdateRadar(meas_package);
  }
  previouse_t_ = meas_package.timestamp_;

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
  MatrixXd Xsig_aug(n_aug_,2*n_aug_+1);
  Xsig_aug.col(0) = x_aug;
  for(int ii = 0; ii < n_aug_; ii++){
    Xsig_aug.col(ii+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(ii);
    Xsig_aug.col(ii+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(ii);
  }


  /***************************************************************
  *      Predict Sigma Point                            *
  ****************************************************************/

  // Lesson 7, section 21: Predict Augmented Sigmapoint Assignment 2
  // predict sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  for(int jj =0; jj < 2*n_aug_+1; jj++){
    // extract values for better readability
    double p_x = Xsig_aug(0,jj);
    double p_y = Xsig_aug(1,jj);
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
    Xsig_pred_(0,jj) = px_p;
    Xsig_pred_(1,jj) = py_p;
    Xsig_pred_(2,jj) = v_p;
    Xsig_pred_(3,jj) = yaw_p;
    Xsig_pred_(4,jj) = yawd_p;
  }

  /***************************************************************
  *      Predict mean and covariance
  ****************************************************************/

  // Lesson 7, section 24: Predict Mean and Covariance Assignment 2
  

  // predicted state mean
  x_.fill(0.0);
  for(int ll = 0; ll < 2 * n_aug_ + 1; ll++){
    x_ = x_ + weights_(ll)* Xsig_pred_.col(ll);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for(int mm = 0; mm < 2 * n_aug_ + 1; mm++){
    // state difference
    VectorXd x_diff = Xsig_pred_.col(mm) - x_;
    // angle normalization
    while(x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
    while(x_diff(3)<M_PI) x_diff(3)+=2.*M_PI;

    P_ = P_ + weights_(mm) * x_diff * x_diff.transpose();
  }

  cout << "Predicted" << endl;

  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
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
  //set measurement dimension
  int n_z_ = 2;

  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z_, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z_);
  
  //measurement covariance matrix S
  MatrixXd S_ = MatrixXd(n_z_,n_z_);

  //prediction noise matrix
  MatrixXd R_ = MatrixXd(n_z_, n_z_);

  //cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z_);

  //ground truth measurement
  VectorXd z = meas_package.raw_measurements_;

  //transform sigma points into measurement space
  for (int i=0; i<2*n_aug_+1; i++)
  {
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    Zsig.col(i) << p_x, p_y;
  }

  //calculate mean predicted measurement
  z_pred.fill(0.0);
  for (int k=0; k<2*n_aug_+1; k++)
  {
    z_pred += weights_(k) * Zsig.col(k);
  }

  //calculate measurement covariance matrix S
  S_.fill(0.0);
  for (int l=0; l<2*n_aug_+1; l++)
  {
    VectorXd z_diff = Zsig.col(l) - z_pred;
    S_ += weights_(l) * z_diff * z_diff.transpose();
  }

  //measurement noise covariance
  R_ <<  std_laspx_*std_laspx_, 0,
         0, std_laspy_*std_laspy_;

  //add measurement noise covariance matrix
  S_ += R_;

  //calculate cross correlation matrix
  Tc.fill(0);
  for (int j=0; j<2 * n_aug_ + 1; j++)
  {
    VectorXd x_diff = Xsig_pred_.col(j) - x_;
    
    VectorXd z_diff = Zsig.col(j) - z_pred;
    Tc += weights_(j) * x_diff * z_diff.transpose();
  }

  //calculate Kalman gain K
  MatrixXd K = Tc * S_.inverse();
  
  //update state mean and covariance matrix
  VectorXd z_diff_ = z - z_pred;
  x_ += K*(z_diff_);
  P_ -= K * S_ * K.transpose();

  cout << "Lidar updated" << endl;

  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
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


  /***************************************************************
  *      Predict Rader Sigma Points                            *
  ****************************************************************/

  // Lesson 7, section 27: Predict Radar Masurement Assignment 2
  
  //create matrix for sigma points in measurement space
  //

  //cout << "Radar update start" << endl;

  int n_z = 3;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  // transform sigma points into measurement space
  for(int ii = 0; ii < 2 * n_aug_ + 1; ii++){ // 2n+1 sigma points
    // extract values for better readability
    double p_x = Xsig_pred_(0,ii);
    double p_y = Xsig_pred_(1,ii);
    double v = Xsig_pred_(2,ii);
    double yaw = Xsig_pred_(3,ii);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;
    
    // measurement model
    double rho = sqrt(p_x*p_x + p_y*p_y);  // r
    if(fabs(rho)<0.001){
      if(rho>0) rho = 0.001;
      if(rho<0) rho = -0.001;
    }
    Zsig(0,ii) = rho;
    if(fabs(p_x)<0.001){
      if(p_x>0) p_x = 0.001;
      if(p_x<0) p_x = -0.001;
    }
    if(fabs(p_y)<0.001){
      if(p_y>0) p_y = 0.001;
      if(p_y<0) p_y = -0.001;
    }
    Zsig(1,ii) = atan2(p_y,p_x);          // phi
    Zsig(2,ii) = (p_x*v1 + p_y*v2) / rho; // r_dot
  }

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(int jj = 0; jj < 2*n_aug_+1; jj++){
    z_pred = z_pred + weights_(jj) * Zsig.col(jj);
  }

  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for(int kk = 0; kk < 2*n_aug_+1;kk++){
    // residual
    VectorXd z_diff = Zsig.col(kk) - z_pred;

    // angle normalization
    while(z_diff(1) > M_PI) z_diff(1)-=2.*M_PI;
    while(z_diff(1) < -M_PI) z_diff(1)+=2.*M_PI;

    S = S + weights_(kk) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_radr_ * std_radr_, 0, 0,
      0, std_radphi_*std_radphi_, 0,
      0, 0, std_radrd_*std_radrd_;
  S = S+R;

  /***************************************************************
  *      Update Radar                                            *
  ****************************************************************/

  // Lesson 7, section 30: UKF Update Assignment 2
  // calculate cross correlation matrix
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for(int ll = 0; ll < 2 * n_aug_+1;ll++){ // 2n+1 sigmapoints
    
    // residual
    VectorXd z_diff = Zsig.col(ll) - z_pred;

    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1) < -M_PI) z_diff(1)+=2.*M_PI;
    
    // state difference
    VectorXd x_diff = Xsig_pred_.col(ll) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights_(ll) * x_diff * z_diff.transpose();
  }

  // update state mean and covariance matrix
  //Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  //residual
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;

  //angle normalization
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

  //update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();

  cout << "Radar updated" << endl;

  cout << "x_ = " << x_ << endl;
  cout << "P_ = " << P_ << endl;
}
