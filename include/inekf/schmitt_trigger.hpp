#ifndef INEKF_SCHMITT_TRIGGER_HPP_
#define INEKF_SCHMITT_TRIGGER_HPP_

#include <cstdint>

#include "inekf/macros.hpp"


namespace inekf {

struct SchmittTriggerSettings {
  double lower_threshold;
  double higher_threshold;
  uint64_t lower_time_delay;
  uint64_t higher_time_delay;
};


class SchmittTrigger {
public:
  SchmittTrigger(const SchmittTriggerSettings& settings);

  SchmittTrigger();

  INEKF_USE_DEFAULT_DESTTUCTOR(SchmittTrigger);
  INEKF_USE_DEFAULT_COPY_CONSTRUCTOR(SchmittTrigger);
  INEKF_USE_DEFAULT_COPY_ASSIGN_OPERATOR(SchmittTrigger);
  INEKF_USE_DEFAULT_MOVE_CONSTRUCTOR(SchmittTrigger);
  INEKF_USE_DEFAULT_MOVE_ASSIGN_OPERATOR(SchmittTrigger);

  void reset();

  void update(const uint64_t current_time, const double value);

  bool getState() const;

  void setParameters(const SchmittTriggerSettings& settings);

private:
  bool state_, first_call_;
  double lower_threshold_, higher_threshold_;
  uint64_t timer_, previous_time_, lower_time_delay_, higher_time_delay_;

};

} // namespace  inekf

#endif // INEKF_SCHMITT_TRIGGER_HPP_ 