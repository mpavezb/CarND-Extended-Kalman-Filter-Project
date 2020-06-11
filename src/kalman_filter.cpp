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
  is_initialized_ = true;
}

void KalmanFilter::Predict() {
  if (not is_initialized_) return;
  /**
   * TODO: predict the state
   */
}

void KalmanFilter::Update(const VectorXd &z) {
  if (not is_initialized_) return;
  /**
   * TODO: update the state by using Kalman Filter equations
   */
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  if (not is_initialized_) return;
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}

VectorXd KalmanFilter::GetState() const { return x_; }
