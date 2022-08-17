#ifndef LEGGED_STATE_ESTIMATOR_LOW_PASS_FILTER_HPP_
#define LEGGED_STATE_ESTIMATOR_LOW_PASS_FILTER_HPP_

#include <cmath>
#include <stdexcept>
#include <iostream>

#include "Eigen/Core"


namespace legged_state_estimator {

///
/// @class LowPassFilter
/// @brief A simple low pass filter.
///
template <typename Scalar, int dim=Eigen::Dynamic>
class LowPassFilter {
public:
  using Vector = Eigen::Matrix<Scalar, dim, 1>;

  ///
  /// @brief Constructs a low pass filter.
  /// @param[in] sampling_time Sampling time.
  /// @param[in] cutoff_freq The cut-off frequency.
  /// @param[in] dynamic_size Size parameter used when the template paramter dim
  /// is set to Eigen::Dynamic.
  ///
  LowPassFilter(const Scalar sampling_time, const Scalar cutoff_freq,
                const int dynamic_size=0)
    : estimate_(),
      alpha_(0.0) {
    if (sampling_time <= 0) {
      throw std::out_of_range(
          "Invalid argment: sampling_time must be positive!");
    }
    if (cutoff_freq <= 0) {
      throw std::out_of_range(
          "Invalid argment: cutoff_freq must be positive!");
    }
    if (dim == Eigen::Dynamic && dynamic_size <= 0) {
      throw std::out_of_range(
          "Invalid argment: dynamic_size must be positive!");
    }
    const Scalar tau = 1.0 / (2.0*M_PI*cutoff_freq);
    alpha_ = tau / (tau + sampling_time);
    if (dim == Eigen::Dynamic) {
      estimate_.resize(dynamic_size);
    }
    estimate_.setZero();
  }

  ///
  /// @brief Default constructor. 
  ///
  LowPassFilter()
    : estimate_(),
      alpha_(0.0) {
  }

  ///
  /// @brief Default destructor. 
  ///
  ~LowPassFilter() = default;

  ///
  /// @brief Reset the filter and reset the estimate to zero. 
  ///
  void reset() {
    estimate_.setZero();
  }

  ///
  /// @brief Reset the filter with input estimate. 
  /// @param[in] estimate An initial estimate. 
  ///
  void reset(const Vector& estimate) {
    estimate_ = estimate;
  }

  ///
  /// @brief Updates the estimate.
  /// @param[in] obs Observation. 
  ///
  void update(const Vector& obs) {
    estimate_.array() *= alpha_;
    estimate_.noalias() += (1.0-alpha_) * obs;
  }

  ///
  /// @brief Gets the estimate.
  /// @return const reference to the estimate.
  ///
  const Vector& getEstimate() const {
    return estimate_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Vector estimate_;
  Scalar alpha_;
};

} // namespace legged_state_estimator 

#endif // LEGGED_STATE_ESTIMATOR_LOW_PASS_FILTER_HPP_ 