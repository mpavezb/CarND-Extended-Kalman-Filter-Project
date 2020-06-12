#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

Eigen::VectorXd ProjectStateToRadar(const VectorXd &z) {
  const float px = z[0];
  const float py = z[1];
  const float vx = z[2];
  const float vy = z[3];

  float rho = sqrtf(px * px + py * py);
  float phi = atan2(py, px);
  float rho_dot = 0;
  if (rho > 0.0001) {
    rho_dot = (px * vx + py * vy) / rho;
  }

  Eigen::VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;
  return z_pred;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  const long x_size = x_.size();
  const MatrixXd I = MatrixXd::Identity(x_size, x_size);

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
  const long x_size = x_.size();
  const MatrixXd I = MatrixXd::Identity(x_size, x_size);

  // Predicted measurement error
  const VectorXd z_pred = ProjectStateToRadar(x_);
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

VectorXd KalmanFilter::GetState() const { return x_; }
