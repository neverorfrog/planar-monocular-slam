#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "pms/dataset.h"
#include "pms/math/definitions.h"
#include "pms/math/pose2.h"
#include "pms/math/pose3.h"
#include "pms/math/rotation_matrix.h"

namespace nb = nanobind;
using EigenMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
NB_MAKE_OPAQUE(pms::RotationMatrix);  // Needed for RotationMatrix bindings

namespace pms {
namespace python {

NB_MODULE(pms, m) {
    m.doc() = "Python bindings for the Planar-Monocular-Slam C++ library";

    nb::class_<Pose2>(m, "Pose2")
        .def(nb::init<>())
        .def(nb::init<const Vector2 &>())
        .def(nb::init<const Angle &, const Vector2 &>())
        .def(nb::init<const Angle &, const Scalar, const Scalar &>())
        .def(nb::init<const Pose2 &>())
        .def(
            "__copy__", [](const Pose2 &self) { return new Pose2(self); }, nb::rv_policy::take_ownership)
        .def("rotation", [](const Pose2 &pose) { return static_cast<double>(pose.rotation); })
        .def_ro("translation", &Pose2::translation)
        .def("__str__", &Pose2::toString);

    nb::class_<Pose3>(m, "Pose3")
        .def(nb::init<>())
        .def(nb::init<const RotationMatrix &, const Vector3 &>())
        .def(
            "__copy__", [](const Pose3 &self) { return new Pose3(self); }, nb::rv_policy::take_ownership)
        .def("getVector", &Pose3::getVector)
        .def("__add__", [](const Pose3 &p1, const Pose3 &p2) { return p1 + p2; })
        .def("__str__", &Pose3::toString)
        .def_rw("rotation", &Pose3::rotation)
        .def_rw("translation", &Pose3::translation);

    nb::class_<Camera>(m, "Camera")
        .def(nb::init<>())
        .def(nb::init<const Camera &>())
        .def(
            "__copy__", [](const Camera &self) { return new Camera(self); }, nb::rv_policy::take_ownership)
        .def_rw("camera_matrix", &Camera::camera_matrix)
        .def_rw("pose", &Camera::pose)
        .def_rw("z_near", &Camera::z_near)
        .def_rw("z_far", &Camera::z_far)
        .def_rw("width", &Camera::width)
        .def_rw("height", &Camera::height)
        .def("__str__", &Camera::toString);

    nb::class_<Landmark>(m, "Landmark")
        .def(nb::init<>())
        .def(nb::init<const Vector3 &, const int>())
        .def(nb::init<const Landmark &>())
        .def(
            "__copy__", [](const Landmark &self) { return new Landmark(self); },
            nb::rv_policy::take_ownership)
        .def_ro("position", &Landmark::position)
        .def_ro("id", &Landmark::id)
        .def("__str__", &Landmark::toString);

    nb::class_<Measurement>(m, "Measurement")
        .def(nb::init<>())
        .def(nb::init<const int, const int, const int, const Vector2 &>())
        .def(nb::init<const Measurement &>())
        .def(
            "__copy__", [](const Measurement &self) { return new Measurement(self); },
            nb::rv_policy::take_ownership)
        .def_ro("seq_number", &Measurement::seq_number)
        .def_ro("current_id", &Measurement::current_id)
        .def_ro("actual_id", &Measurement::actual_id)
        .def_ro("image_point", &Measurement::image_point)
        .def("__str__", &Measurement::toString);

    nb::class_<TrajPoint>(m, "TrajPoint")
        .def(nb::init<>())
        .def(nb::init<const int, const Pose2 &, const Pose2 &>())
        .def(nb::init<const TrajPoint &>())
        .def(
            "__copy__", [](const TrajPoint &self) { return new TrajPoint(self); },
            nb::rv_policy::take_ownership)
        .def_ro("id", &TrajPoint::id)
        .def_ro("odometry", &TrajPoint::odometry)
        .def_ro("ground_truth", &TrajPoint::ground_truth)
        .def_ro("measurements", &TrajPoint::measurements)
        .def("__str__", &TrajPoint::toString);

    nb::class_<Dataset>(m, "Dataset")
        .def(nb::init<const std::string &>())
        .def(
            "__copy__", [](const Dataset &self) { return new Dataset(self); }, nb::rv_policy::take_ownership)
        .def_ro("camera", &Dataset::camera)
        .def_ro("world", &Dataset::world)
        .def_ro("trajectory", &Dataset::trajectory)
        .def("__str__", &Dataset::toString);
};

}  // namespace python
}  // namespace pms
