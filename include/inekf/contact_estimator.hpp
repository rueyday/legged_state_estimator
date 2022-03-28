#ifndef INEKF_CONTACT_ESTIMATOR_HPP_
#define INEKF_CONTACT_ESTIMATOR_HPP_

#include <vector>
#include <utility>
#include <cmath>
#include <stdexcept>
#include <iostream>

#include "Eigen/Core"
#include "Eigen/StdVector"

#include "inekf/macros.hpp"
#include "inekf/robot_model.hpp"
#include "inekf/schmitt_trigger.hpp"


namespace inekf {

struct ContactEstimatorSettings {
  std::vector<double> beta0;
  std::vector<double> beta1;
  std::vector<double> force_sensor_bias;
  double contact_force_cov_alpha;
  SchmittTriggerSettings schmitt_trigger_settings;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


class ContactEstimator {
public:
  ContactEstimator(const RobotModel& robot_model, 
                   const ContactEstimatorSettings& settings);

  ContactEstimator();

  INEKF_USE_DEFAULT_DESTTUCTOR(ContactEstimator);
  INEKF_USE_DEFAULT_COPY_CONSTRUCTOR(ContactEstimator);
  INEKF_USE_DEFAULT_COPY_ASSIGN_OPERATOR(ContactEstimator);
  INEKF_USE_DEFAULT_MOVE_CONSTRUCTOR(ContactEstimator);
  INEKF_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(ContactEstimator);

  void reset();

  void update(const RobotModel& robot_model, const Eigen::VectorXd& tauJ, 
              const std::vector<double>& force_sensor_raw);

  void setParameters(const ContactEstimatorSettings& settings);

  std::vector<std::pair<int, bool>> getContactState(const double prob_threshold=0.5) const;

  const std::vector<Eigen::Vector3d>& getContactForceEstimate() const;

  const std::vector<double>& getContactForceNormalEstimate() const;

  const std::vector<double>& getContactProbability() const;

  double getContactForceCovariance(const double prob_threshold=0.5) const;

  const std::vector<double>& getForceSensorBias() const;

  const std::vector<Eigen::Vector3d>& getContactSurfaceNormal() const;

  void setForceSensorBias(const std::vector<double>& force_sensor_bias);

  void setContactSurfaceNormal(const std::vector<Eigen::Vector3d>& contact_surface_normal);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  ContactEstimatorSettings settings_;
  std::vector<Eigen::Vector3d> contact_force_estimate_, contact_surface_normal_;
  std::vector<double> contact_force_normal_estimate_, 
                      contact_force_normal_estimate_prev_, 
                      contact_probability_, contact_covariance_;
  std::vector<SchmittTrigger> schmitt_trigger_;
  int num_contacts_;
};

} // namespace inekf

#endif // INEKF_CONTACT_ESTIMATOR_HPP_ 