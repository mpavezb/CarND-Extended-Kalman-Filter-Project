#include "tools.h"

#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  const auto n_estimations = estimations.size();
  rmse << 0, 0, 0, 0;

  // Check Inputs
  if (n_estimations == 0) {
    std::cout << "Invalid estimation vector (size == 0)." << std::endl;
    return rmse;
  }
  if (n_estimations != ground_truth.size()) {
    std::cout << "Estimation vector size (" << n_estimations
              << ") != ground truth size (" << ground_truth.size() << ")."
              << std::endl;
    return rmse;
  }

  for (int i = 0; i < n_estimations; ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }
  rmse = rmse / n_estimations;
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  MatrixXd m{};
  return m;
}
