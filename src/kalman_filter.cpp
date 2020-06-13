#include "kalman_filter.h"

#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  static const long x_size = x_.size();
  static const MatrixXd I = MatrixXd::Identity(x_size, x_size);

  // Predicted measurement error
  const VectorXd z_pred = H_ * x_;
  const VectorXd y = z - z_pred;

  // Kalman Gain
  const MatrixXd Ht = H_.transpose();
  const MatrixXd S = H_ * P_ * Ht + R_;
  const MatrixXd Si = S.inverse();
  const MatrixXd PHt = P_ * Ht;
  const MatrixXd K = PHt * Si;

  // Update
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  static const long x_size = x_.size();
  static const MatrixXd I = MatrixXd::Identity(x_size, x_size);

  // Predicted measurement error
  const VectorXd z_pred = Tools::ProjectStateToRadar(x_);
  VectorXd y = z - z_pred;
  y(1) = Tools::normalize_angle(y(1));

  // Kalman Gain
  const MatrixXd Ht = H_.transpose();
  const MatrixXd S = H_ * P_ * Ht + R_;
  const MatrixXd Si = S.inverse();
  const MatrixXd PHt = P_ * Ht;
  const MatrixXd K = PHt * Si;

  // Update
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}
