#include <string>
#include "legged_state_estimator/legged_state_estimator.hpp"


int main() {
  std::string urdf_path = "a1_description/urdf/a1_friction.urdf";
  double time_step = 0.0025;

  auto settings = legged_state_estimator::LeggedStateEstimatorSettings::UnitreeA1(urdf_path, time_step);
  auto estimator = legged_state_estimator::LeggedStateEstimator(settings);
  legged_state_estimator::LeggedStateEstimator estimator2(estimator);
  estimator2 = estimator;

  legged_state_estimator::LeggedStateEstimator estimator3;
  estimator3 = estimator;

  return 0;
}
