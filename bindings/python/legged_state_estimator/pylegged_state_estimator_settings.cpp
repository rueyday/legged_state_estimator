#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "legged_state_estimator/legged_state_estimator_settings.hpp"


namespace legged_state_estimator {
namespace python {

namespace py = pybind11;

PYBIND11_MODULE(pylegged_state_estimator_settings, m) {
  py::class_<LeggedStateEstimatorSettings>(m, "LeggedStateEstimatorSettings")
    .def(py::init<>())
    .def_static("UnitreeA1", &LeggedStateEstimatorSettings::UnitreeA1,
                 py::arg("urdf_path"), py::arg("sampling_time"))
    .def_readwrite("urdf_path", &LeggedStateEstimatorSettings::urdf_path)
    .def_readwrite("imu_frame", &LeggedStateEstimatorSettings::imu_frame)
    .def_readwrite("contact_frames", &LeggedStateEstimatorSettings::contact_frames)
    .def_readwrite("contact_estimator_settings", &LeggedStateEstimatorSettings::contact_estimator_settings)
    .def_readwrite("inekf_noise_params", &LeggedStateEstimatorSettings::inekf_noise_params)
    .def_readwrite("dynamic_contact_estimation", &LeggedStateEstimatorSettings::dynamic_contact_estimation)
    .def_readwrite("contact_position_noise", &LeggedStateEstimatorSettings::contact_position_noise)
    .def_readwrite("contact_rotation_noise", &LeggedStateEstimatorSettings::contact_rotation_noise)
    .def_readwrite("sampling_time", &LeggedStateEstimatorSettings::sampling_time)
    .def_readwrite("lpf_gyro_accel_cutoff_frequency", &LeggedStateEstimatorSettings::lpf_gyro_accel_cutoff_frequency)
    .def_readwrite("lpf_lin_accel_cutoff_frequency", &LeggedStateEstimatorSettings::lpf_lin_accel_cutoff_frequency)
    .def_readwrite("lpf_dqJ_cutoff_frequency", &LeggedStateEstimatorSettings::lpf_dqJ_cutoff_frequency)
    .def_readwrite("lpf_ddqJ_cutoff_frequency", &LeggedStateEstimatorSettings::lpf_ddqJ_cutoff_frequency)
    .def_readwrite("lpf_tauJ_cutoff_frequency", &LeggedStateEstimatorSettings::lpf_dqJ_cutoff_frequency);
}

} // namespace python
} // namespace legged_state_estimator