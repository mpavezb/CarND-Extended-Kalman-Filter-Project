#include "FusionEKF.h"

#include <iostream>

// TODO: somehow it is important to include this again!
#include "Eigen/Dense"
#include "tools.h"

FusionEKF::FusionEKF() {
  // ---------------------------------------------
  // initializing matrices
  // ---------------------------------------------
  // Lidar measurement function matrix
  H_laser_ = Eigen::MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;

  // Lidar measurement covariance matrix
  R_laser_ = Eigen::MatrixXd(2, 2);
  R_laser_ << 0.0225, 0, 0, 0.0225;

  // Radar measurement covariance matrix
  R_radar_ = Eigen::MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

  // Radar measurement function jacobian
  // Has to be computed at each step.
  H_radar_ = Eigen::MatrixXd(3, 4);

  // State transition matrix needs to be updated on every step
  // to model the elapsed time on the motion equations.
  ekf_.F_ = Eigen::MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,  //
      0, 1, 0, 1,         //
      0, 0, 1, 0,         //
      0, 0, 0, 1;

  // State Covariance matrix is computed by Kalman Filter.
  // Initial values with high certainty on position and low in velocity.
  ekf_.P_ = Eigen::MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,  //
      0, 1, 0, 0,         //
      0, 0, 1000, 0,      //
      0, 0, 0, 1000;

  // State transition noise is computed at every step.
  ekf_.Q_ = Eigen::MatrixXd::Identity(4, 4);
}

FusionEKF::~FusionEKF() {}

Eigen::VectorXd FusionEKF::GetEstimate() const { return ekf_.GetState(); }

void FusionEKF::Initialize(const MeasurementPackage &measurement_pack) {
  switch (measurement_pack.sensor_type_) {
    case MeasurementPackage::SensorType::RADAR:
      InitializeWithRadar(measurement_pack.raw_measurements_[0],
                          measurement_pack.raw_measurements_[1],
                          measurement_pack.raw_measurements_[2]);
      break;
    case MeasurementPackage::SensorType::LASER:
      InitializeWithLidar(measurement_pack.raw_measurements_[0],
                          measurement_pack.raw_measurements_[1]);
      break;
  }
  previous_timestamp_ = measurement_pack.timestamp_;
}

void FusionEKF::InitializeWithRadar(const float rho, const float phi,
                                    const float rho_dot) {
  // Polar to cartesian conversion
  const float px = rho * cosf(phi);
  const float py = rho * sinf(phi);
  const float vx = rho_dot * cosf(phi);
  const float vy = rho_dot * sinf(phi);

  // state vector
  ekf_.x_ = Eigen::VectorXd(4);
  ekf_.x_ << px, py, vx, vy;

  is_initialized_ = true;
}

void FusionEKF::InitializeWithLidar(const float px, const float py) {
  // set the state with the initial location and zero velocity
  ekf_.x_ = Eigen::VectorXd(4);
  ekf_.x_ << px, py, 0, 0;

  is_initialized_ = true;
}

void FusionEKF::Prediction(long long timestamp) {
  // Updates F and Q according to the elapsed time.
  // Time is measured in seconds.

  // elapsed time
  float dt = (timestamp - previous_timestamp_) / 1000000.0;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Update F matrix
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // Update Q matrix
  ekf_.Q_ = Eigen::MatrixXd(4, 4);
  ekf_.Q_ << dt_4 / 4 * noise_ax_, 0, dt_3 / 2 * noise_ax_, 0, 0,
      dt_4 / 4 * noise_ay_, 0, dt_3 / 2 * noise_ay_, dt_3 / 2 * noise_ax_, 0,
      dt_2 * noise_ax_, 0, 0, dt_3 / 2 * noise_ay_, 0, dt_2 * noise_ay_;

  // predict
  ekf_.Predict();
  previous_timestamp_ = timestamp;
}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  if (!is_initialized_) {
    Initialize(measurement_pack);
    return;
  }

  /**
   * Prediction
   */
  Prediction(measurement_pack.timestamp_);

  /**
   * Update
   */
  const Eigen::VectorXd z = measurement_pack.raw_measurements_;
  switch (measurement_pack.sensor_type_) {
    case MeasurementPackage::SensorType::RADAR:
      H_radar_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.H_ = H_radar_;
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(z);

      break;
    case MeasurementPackage::SensorType::LASER:
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      ekf_.Update(z);
      break;
  }

  // print the output
  std::cout << "x_ = " << ekf_.x_ << std::endl;
  std::cout << "P_ = " << ekf_.P_ << std::endl;
}
