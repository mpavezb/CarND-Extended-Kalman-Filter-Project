#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <string>
#include <vector>

#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
 public:
  FusionEKF();
  virtual ~FusionEKF();

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * Get current Kalman Filter estimate
   */
  Eigen::VectorXd GetEstimate() const;

 private:
  void Initialize(const MeasurementPackage &measurement_pack);
  void InitializeWithRadar(const float rho, const float phi,
                           const float rho_dot);
  void InitializeWithLidar(const float px, const float py);
  void Prediction(long long timestamp);

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_{};

  // check whether the tracking toolbox was initialized or not (first
  // measurement)
  bool is_initialized_{false};

  // previous timestamp
  long long previous_timestamp_{0ll};

  // tool object used to compute Jacobian and RMSE
  Tools tools;
  Eigen::MatrixXd R_laser_;
  Eigen::MatrixXd R_radar_;
  Eigen::MatrixXd H_laser_;
  Eigen::MatrixXd H_radar_;

  // acceleration noise components
  float noise_ax_{9.0f};
  float noise_ay_{9.0f};
};

#endif  // FusionEKF_H_
