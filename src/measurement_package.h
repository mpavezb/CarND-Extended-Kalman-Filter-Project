#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
 public:
  enum class SensorType : uint8_t { LASER, RADAR };

  SensorType sensor_type_;
  long long timestamp_;

  Eigen::VectorXd raw_measurements_;
};

#endif  // MEASUREMENT_PACKAGE_H_
