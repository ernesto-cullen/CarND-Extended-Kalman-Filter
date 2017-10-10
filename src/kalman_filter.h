#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "measurement_package.h"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transition matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // Identity matrix, computed at the beginning
  Eigen::MatrixXd I;

  bool is_initialized_;

  // previous timestamp
  long previous_timestamp_;

  // acceleration noise components (sigma squared)
  double noise_ax = 9.0;
  double noise_ay = 9.0;

  // measurement covariance matrix - laser
  Eigen::MatrixXd R_laser_;
  // measurement covariance matrix - radar
  Eigen::MatrixXd R_radar_;
  // extraction matrix - laser
  Eigen::MatrixXd H_laser_;

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
   */
  void Predict();

  /**
   * Updates the state
   * @param z measurement at k+1
   * @param Hx measurement matrix * state (x)
   * @param R measurement covariant matrix
   * @param H measurement matrix
   */
  void Update(Eigen::VectorXd z, Eigen::VectorXd Hx, Eigen::MatrixXd R, Eigen::MatrixXd H);

  /**
   * process one measurement
   * @param measurement_pack data from measurement
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * update process noise covariant matrix after a step has passed
   * @param dt time passed, in seconds
   */
  void updateQ(double dt);
};

#endif /* KALMAN_FILTER_H_ */
