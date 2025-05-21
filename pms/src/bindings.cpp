// #include <nanobind/nanobind.h>

// #include <nanobind/eigen/dense.h>
// #include <nanobind/nanobind.h>
// #include <nanobind/stl/string.h>
// #include <nanobind/stl/vector.h>
// #include <nanobind/operators.h>

// #include "ismpc_cpp/ismpc.h"
// #include "ismpc_cpp/tools/math/rotation_matrix.h"
// #include "ismpc_cpp/types/footstep.h"
// #include "ismpc_cpp/types/lip_state.h"
// #include "ismpc_cpp/types/support_phase.h"

// namespace nb = nanobind;
// using EigenMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
// NB_MAKE_OPAQUE(ismpc::RotationMatrix); // Needed for RotationMatrix bindings

// namespace pms {
// namespace python {

// NB_MODULE(ismpc, m) {
//     m.doc() = "Python bindings for the Planar-Monocular-Slam C++ library";

//     nb::enum_<TailType>(m, "TailType")
//         .value("PERIODIC", TailType::PERIODIC)
//         .value("TRUNCATED", TailType::TRUNCATED)
//         .value("ANTICIPATIVE", TailType::ANTICIPATIVE)
//         .export_values();

//     nb::class_<State>(m, "State")
//         // Constructor takes Params
//         .def(nb::init<const Params &>())
//         .def(nb::init<const State &>())
//         .def("__copy__", [](const State &self) {
//             return new State(self);
//         }, nb::rv_policy::take_ownership)
//         .def_rw("lip", &State::lip)
//         .def_rw("left_foot", &State::left_foot)
//         .def_rw("right_foot", &State::right_foot)
//         .def_rw("torso", &State::torso)
//         .def_rw("base", &State::base)
//         .def_rw("desired_lip", &State::desired_lip)
//         .def_rw("desired_left_foot", &State::desired_left_foot)
//         .def_rw("desired_right_foot", &State::desired_right_foot)
//         .def_rw("desired_torso", &State::desired_torso)
//         .def_rw("desired_base", &State::desired_base)
//         .def_ro("left_foot_x", &State::left_foot_x)
//         .def_ro("left_foot_y", &State::left_foot_y)
//         .def_ro("right_foot_x", &State::right_foot_x)
//         .def_ro("right_foot_y", &State::right_foot_y)
//         .def_rw("total_mpc_qp_duration", &State::total_mpc_qp_duration) // Made rw for potential reset
//         .def_rw("total_mpc_preprocessing_duration", &State::total_mpc_preprocessing_duration) // Made rw
//         .def_rw("total_mpc_postprocessing_duration", &State::total_mpc_postprocessing_duration) // Made rw
//         .def_rw("lip_history", &State::lip_history) // Made rw for potential clearing/access
//         .def_rw("left_foot_history", &State::left_foot_history) // Made rw
//         .def_rw("right_foot_history", &State::right_foot_history) // Made rw
//         .def("__str__", &State::toString);

//     nb::class_<KalmanFilter>(m, "KalmanFilter")
//         // Constructor takes Params
//         .def(nb::init<const Params &>())
//         .def("update", &KalmanFilter::update);
// };

// } // namespace python
// } // namespace pms
