#ifndef UKF_H
#define UKF_H

#include "Eigen/Dense"
#include "measurement_package.h"
#include <iostream>

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);


  // initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  // if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // predicted sigma points matrix
  Eigen::MatrixXd Xsig_pred_;

  // time when the state is true, in us
  long long time_us_;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  // Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  // Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  // Radar measurement noise standard deviation radius in m
  double std_radr_;

  // Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  // Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  // Weights of sigma points
  Eigen::VectorXd weights_;

  // State dimension
  int n_x_;

  // Augmented state dimension
  int n_aug_;

  // Sigma point spreading parameter
  double lambda_;

  // augmented mean vector 
  Eigen::VectorXd x_aug;

  // augmented covariance matrix
  Eigen::MatrixXd P_aug;

  // sigma point matrix
  Eigen::MatrixXd Xsig_aug;

  // Radar -------------------
  // measurement dimension - Radar
  int n_z_Radar_;

  // matrix for sigma points in measurement space - Radar
  Eigen::MatrixXd Zsig_Radar_;

  // mean predicted measurement - Radar
  Eigen::VectorXd zpred_Radar_;

  // measurement covariance matrix S - Radar
  Eigen::MatrixXd S_Radar_;

  // noise covariance matrix 
  Eigen::MatrixXd R_Radar_;

  // matrix for cross-correlation Tc - Radar
  Eigen::MatrixXd Tc_Radar_;

  // NIS 
  double NIS_Radar_;

  // LiDAR -------------------
  // measurement dimension - Lidar
  int n_z_Lidar_;

  // matrix for sigma points in measurement space - Lidar
  Eigen::MatrixXd Zsig_Lidar_;

  // mean predicted measurement - Lidar
  Eigen::VectorXd zpred_Lidar_;

  // measurement covariance matrix S - Lidar
  Eigen::MatrixXd S_Lidar_;

  // noise covariance matrix
  Eigen::MatrixXd R_Lidar_;

  // matrix for cross correlation - Lidar
  Eigen::MatrixXd Tc_Lidar_;

  // NIS
  double NIS_Lidar_;
};

#endif  // UKF_H