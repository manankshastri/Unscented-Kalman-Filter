#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  P_ << 1, 0, 0, 0, 0, 
        0, 1, 0, 0, 0, 
        0, 0, 1, 0, 0, 
        0, 0, 0, 0.05, 0, 
        0, 0, 0, 0, 0.05;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.0;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

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
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  // state dimension
  n_x_ = 5;

  // augmented state dimension
  n_aug_ = n_x_ + 2;

  // sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // weights of sigma points
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/ (lambda_ + n_aug_);

  for (int i = 1; i < 2 * n_aug_ + 1; i++){
    weights_(i) = 0.5 / (lambda_ + n_aug_);
  }

  time_us_ = 0.0;

  // augmented mean vector
  x_aug = VectorXd(n_aug_);

  // augmented covariance matrix 
  P_aug = MatrixXd(n_aug_, n_aug_);

  // sigma point matrix
  Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);

  // RADAR ---------------------------
  // measurement dimension - Radar
  n_z_Radar_ = 3;

  // matrix for sigma points in measurement space - Radar
  Zsig_Radar_ = MatrixXd(n_z_Radar_, 2 * n_aug_ + 1);

  // mean preduicted measurement - Radar
  zpred_Radar_ = VectorXd(n_z_Radar_);

  // measurement covariance matrix S - Radar
  S_Radar_ = MatrixXd(n_z_Radar_, n_z_Radar_);

  // noise covariance matrix - Radar
  R_Radar_ = MatrixXd(3, 3);
  R_Radar_.fill(0.0);
  R_Radar_(0, 0) = std_radr_ * std_radr_;
  R_Radar_(1, 1) = std_radphi_ * std_radphi_;
  R_Radar_(2, 2) = std_radrd_ * std_radrd_;

  // matrix for cross-correlation Tc - Radar
  Tc_Radar_ = MatrixXd(n_x_, n_z_Radar_);

  // NIS
  NIS_Radar_ = 0.0;

  // LiDAR ----------------------------
  // measurement dimension - Lidar
  n_z_Lidar_ = 2;

  // matrix for sigma points in measurement space - Lidar
  Zsig_Lidar_ = MatrixXd(n_z_Lidar_, 2 * n_aug_ +  1);

  // mean predicted measurement - Lidar
  zpred_Lidar_ = VectorXd(n_z_Lidar_);

  // measurement covariance matrix S - Lidar
  S_Lidar_ = MatrixXd(n_z_Lidar_, n_z_Lidar_);
  
  // noise covariance matrix - Lidar
  R_Lidar_ = MatrixXd(2, 2);
  R_Lidar_.fill(0.0);
  R_Lidar_(0, 0) = std_laspx_ * std_laspx_;
  R_Lidar_(1, 1) = std_laspy_ * std_laspy_;

  // matrix for cross correlation - Lidar
  Tc_Lidar_ = MatrixXd(n_x_, n_z_Lidar_);

  // NIS
  NIS_Lidar_ = 0.0;

  is_initialized_ = false;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */

  if (!is_initialized_) {

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {

      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double rhoD = meas_package.raw_measurements_(2);

      double px = rho * cos(phi);
      double py = rho * sin(phi);

      double vx = rhoD * cos(phi);
      double vy = rhoD * sin(phi);

      double v = sqrt(vx*vx + vy*vy);

      x_ << px, py, v, 0, 0;

      // P_ << std_radr_ * std_radr_, 0, 0, 0, 0,
      //       0, std_radr_ * std_radr_, 0, 0, 0, 
      //       0, 0, std_radrd_ * std_radrd_, 0, 0,
      //       0, 0, 0, std_radphi_ * std_radphi_, 0,
      //       0, 0, 0, 0, std_radphi_ * std_radphi_;
    }

    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0, 0, 0;

      // P_ << std_laspx_ * std_laspx_, 0, 0, 0, 0,
      //       0, std_laspy_ * std_laspy_, 0, 0, 0, 
      //       0, 0, 1, 0, 0,
      //       0, 0, 0, 1, 0, 
      //       0, 0, 0, 0, 1;
    }
    is_initialized_ = true;

    std::cout<<"Stage 1 Complete!!"<<std::endl;
    return;
  }

  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  Prediction(dt);
  // std::cout<<"Prediction Complete!!"<<std::endl;

  if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
    // std::cout<<"Using Radar"<<std::endl;
    UpdateRadar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
    // std::cout<<"Using Lidar"<<std::endl;
    UpdateLidar(meas_package);
  }
}


void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  // sqaure root matrix
  MatrixXd P_augT = P_aug.llt().matrixL(); 

  Xsig_aug.col(0) = x_aug;

  for (int i = 0; i<n_aug_; i++) {
    Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * P_augT.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * P_augT.col(i);
  }

  // std::cout<<"Xsig_aug : "<< Xsig_aug<< std::endl;
  std::cout<<"Sigma Points Augmented!!"<<std::endl;

  // sigma points prediction

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    double px = Xsig_aug(0, i);
    double py = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawD = Xsig_aug(4, i);

    double nu_A = Xsig_aug(5, i);
    double nu_YawDD = Xsig_aug(6, i);

    double px_k, py_k, v_k, yaw_k, yawD_k;

    if (fabs(yawD) > 0.00001) {
      px_k = px + (v/yawD) * (sin(yaw + yawD * delta_t) - sin(yaw));
      py_k = py + (v/yawD) * (cos(yaw) - cos(yaw + yawD * delta_t));
    }
    else {
      px_k = px + (v * delta_t * cos(yaw));
      py_k = py + (v * delta_t * sin(yaw));
    }

    v_k = v;
    yaw_k = yaw + yawD * delta_t;
    yawD_k = yawD;

    // adding noise 
    px_k += (0.5 * delta_t * delta_t * cos(yaw) * nu_A);
    py_k += (0.5 * delta_t * delta_t * sin(yaw) * nu_A);

    v_k += (delta_t * nu_A);
    yaw_k += (0.5 * delta_t * delta_t * nu_YawDD);
    yawD_k += (delta_t * nu_YawDD);


    Xsig_pred_(0, i) = px_k;
    Xsig_pred_(1, i) = py_k;
    Xsig_pred_(2, i) = v_k;
    Xsig_pred_(3, i) = yaw_k;
    Xsig_pred_(4, i) = yawD_k;
  }

  std::cout<<"Sigma Points Prediction Complete!!"<<std::endl;

  // state mean prediction

  x_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    x_ += weights_(i) * Xsig_pred_.col(i);
  }

  // state covariance prediction

  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd xDiff = Xsig_pred_.col(i) - x_;

    while (xDiff(3) > M_PI) xDiff(3) -= 2. * M_PI;
    while (xDiff(3) < -M_PI) xDiff(3) += 2. * M_PI;

    P_ += weights_(i) * xDiff * xDiff.transpose();
  }

  std::cout<<"State and Covariance Matrix Prediction Complete!!"<<std::endl;
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

  // transforming sigma points into measurement space 
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    Zsig_Lidar_(0, i) = Xsig_pred_(0, i);
    Zsig_Lidar_(1, i) = Xsig_pred_(1, i);
  }

  // mean predicted measurement
  zpred_Lidar_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    zpred_Lidar_ += weights_(i) * Zsig_Lidar_.col(i);
  }

  // predicted covariance
  S_Lidar_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd zDiff = Zsig_Lidar_.col(i) - zpred_Lidar_;

    S_Lidar_ += weights_(i) * zDiff * zDiff.transpose();
  }

  S_Lidar_ += R_Lidar_;

  Tc_Lidar_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd xDiff = Xsig_pred_.col(i) - x_;

    VectorXd zDiff = Zsig_Lidar_.col(i) - zpred_Lidar_;

    Tc_Lidar_ += weights_(i) * xDiff * zDiff.transpose();
  }

  // Kalman Gain
  MatrixXd KGain_lidar_ = Tc_Lidar_ * S_Lidar_.inverse();

  // update state mean and covariance matrix
  VectorXd zDiff1 = meas_package.raw_measurements_ - zpred_Lidar_;

  x_ += KGain_lidar_ * zDiff1;
  P_ -= KGain_lidar_ * S_Lidar_ * KGain_lidar_.transpose();

  NIS_Lidar_ = zDiff1.transpose() * S_Lidar_.inverse() * zDiff1;

  std::cout<<"Lidar State and Covariance Updated!!"<<std::endl; 
  std::cout<<"NIS Lidar: "<<NIS_Lidar_<<std::endl;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  // transforming sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {

    double px = Xsig_pred_(0, i);
    double py = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);
    double yawD = Xsig_pred_(4, i);

    Zsig_Radar_(0, i) = sqrt(px * px + py * py);
    Zsig_Radar_(1, i) = atan2(py, px);
    Zsig_Radar_(2, i) = (px * cos(yaw) * v + py * sin(yaw) * v) / (sqrt(px * px + py * py));
  }

  // calculate mean predicted measurement
  zpred_Radar_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    zpred_Radar_ += weights_(i) * Zsig_Radar_.col(i);
  }

  S_Radar_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd zDiff = Zsig_Radar_.col(i) - zpred_Radar_;

    while (zDiff(1) > M_PI) zDiff(1) -= 2. * M_PI;
    while (zDiff(1) < -M_PI) zDiff(1) += 2. * M_PI;

    S_Radar_ += weights_(i) * zDiff * zDiff.transpose();
  }

  S_Radar_ += R_Radar_;

  Tc_Radar_.fill(0.0);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd xDiff = Xsig_pred_.col(i) - x_;

    while (xDiff(3) > M_PI) xDiff(3) -= 2. * M_PI;
    while (xDiff(3) < -M_PI) xDiff(3) += 2. * M_PI;

    VectorXd zDiff = Zsig_Radar_.col(i) - zpred_Radar_;

    while (zDiff(1) > M_PI) zDiff(1) -= 2. * M_PI;
    while (zDiff(1) < -M_PI) zDiff(1) += 2. * M_PI;

    Tc_Radar_ += weights_(i) * xDiff * zDiff.transpose();
  }

  // Kalman Gain 
  MatrixXd KGain_Radar_ = Tc_Radar_ * S_Radar_.inverse();

  // update state mean and covariance matrix
  VectorXd zDiff1 = meas_package.raw_measurements_ - zpred_Radar_;

  while (zDiff1(1) > M_PI) zDiff1(1) -= 2. * M_PI;
  while (zDiff1(1) < -M_PI) zDiff1(1) += 2. * M_PI;

  x_ += KGain_Radar_ * zDiff1;
  P_ -= KGain_Radar_ * S_Radar_ * KGain_Radar_.transpose();

  NIS_Radar_ = zDiff1.transpose() * S_Radar_.inverse() * zDiff1;

  std::cout<<"Radar State and Covariance Updated!!"<<std::endl; 
  std::cout<<"NIS: "<<NIS_Radar_<<std::endl;
}