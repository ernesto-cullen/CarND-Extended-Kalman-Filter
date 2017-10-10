#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  F_ = MatrixXd(4,4);
  P_ = MatrixXd(4,4);
  x_ = VectorXd(4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
      0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

  Q_ = MatrixXd::Zero(4,4);

  P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

  I = MatrixXd::Identity(4,4);
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft;
  P_ = P_ + Q_;
}

void KalmanFilter::Update(Eigen::VectorXd z, Eigen::VectorXd Hx, Eigen::MatrixXd R, Eigen::MatrixXd H) {
  VectorXd y = z-Hx;
  MatrixXd Ht = H.transpose();
  MatrixXd S = H * P_ * Ht;
  S = S+R;
  MatrixXd K = P_ * Ht * S.inverse();
  //if measurement is in polar coordinates, normalize angle
  if (y.size() == 3) {
    y(1) = atan2(sin(y(1)), cos(y(1)));
  }
  x_ = x_ + K * y;
  P_ = (I-K*H)*P_;
}

void KalmanFilter::updateQ(double dt) {
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
      0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
      dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
      0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
}


void KalmanFilter::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    MatrixXd H;
    MatrixXd R;
    previous_timestamp_ = measurement_pack.timestamp_;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      x_ = Tools::polar2cartesian(measurement_pack.raw_measurements_);
      H = Tools::jacobian(x_);
      R = R_radar_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.6, 0;
      H = H_laser_;
      R = R_laser_;
    }

    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1.e6;
  previous_timestamp_ = measurement_pack.timestamp_;

  //update Q and F with elapsed time
  updateQ(dt);

  F_(0,2) = dt;
  F_(1,3) = dt;

  Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  VectorXd z = measurement_pack.raw_measurements_;
  VectorXd Hx;
  MatrixXd R, H;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    H = Tools::jacobian(Tools::polar2cartesian(z));
    Hx = Tools::cartesian2polar(x_);
    R = R_radar_;
  } else {
    H = H_laser_;
    Hx = H * x_;
    R = R_laser_;
  }
  Update(z, Hx, R, H);
}
