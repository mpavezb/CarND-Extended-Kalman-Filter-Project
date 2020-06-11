#include "../src/tools.h"
#include "gtest/gtest.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

constexpr float DELTA_E = 0.001;

TEST(tools_test, CalculateRMSE_GivenEstimationAndGroundTruth_ExpectValidRMSE) {
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  // the input list of estimations
  VectorXd e(4);
  e << 1, 1, 0.2, 0.1;
  estimations.push_back(e);
  e << 2, 2, 0.3, 0.2;
  estimations.push_back(e);
  e << 3, 3, 0.4, 0.3;
  estimations.push_back(e);

  // the corresponding list of ground truth values
  VectorXd g(4);
  g << 1.1, 1.1, 0.3, 0.2;
  ground_truth.push_back(g);
  g << 2.1, 2.1, 0.4, 0.3;
  ground_truth.push_back(g);
  g << 3.1, 3.1, 0.5, 0.4;
  ground_truth.push_back(g);

  Tools tools{};
  const auto rmse = tools.CalculateRMSE(estimations, ground_truth);
  EXPECT_NEAR(rmse[0], 0.1, DELTA_E);
  EXPECT_NEAR(rmse[1], 0.1, DELTA_E);
  EXPECT_NEAR(rmse[2], 0.1, DELTA_E);
  EXPECT_NEAR(rmse[3], 0.1, DELTA_E);
}
