#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/operators.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "pms/camera/camera.h"
#include "pms/camera/triangulation.h"
#include "pms/dataset/dataset.h"
#include "pms/math/definitions.h"
#include "pms/math/pose2.h"
#include "pms/math/pose3.h"
#include "pms/math/rotation_matrix.h"
#include "pms/optimization/bundle_adjuster.h"
#include "pms/optimization/state.h"

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
        .def("__mul__", [](const Pose2 &pose, const Vector2 &vec) { return pose * vec; })
        .def("__mul__", [](const Pose2 &pose, const Pose2 &other) { return pose * other; })
        .def("__add__", [](const Pose2 &p1, const Pose2 &p2) { return p1 + p2; })
        .def("__sub__", [](const Pose2 &p1, const Pose2 &p2) { return p1 - p2; })
        .def("inverse", &Pose2::inverse)
        .def("__str__", &Pose2::toString);

    nb::class_<Pose3>(m, "Pose3")
        .def(nb::init<>())
        .def(nb::init<const RotationMatrix &, const Vector3 &>())
        .def(
            "__copy__", [](const Pose3 &self) { return new Pose3(self); }, nb::rv_policy::take_ownership)
        .def("getVector", &Pose3::getVector)
        .def("getPose2", &Pose3::getPose2)
        .def("inverse", &Pose3::inverse)
        .def("__add__", [](const Pose3 &p1, const Pose3 &p2) { return p1 + p2; })
        .def("__mul__", [](const Pose3 &pose, const Vector3 &vec) { return pose * vec; })
        .def("__mul__", [](const Pose3 &pose, const Pose3 &other) { return pose * other; })
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
        .def(nb::init<const int>())
        .def(nb::init<const Vector3 &, const int, const bool>())
        .def(nb::init<const Landmark &>())
        .def(
            "__copy__", [](const Landmark &self) { return new Landmark(self); },
            nb::rv_policy::take_ownership)
        .def_ro("position", &Landmark::position)
        .def_ro("id", &Landmark::id)
        .def_ro("valid", &Landmark::valid)
        .def("__str__", &Landmark::toString);

    nb::class_<Measurement>(m, "Measurement")
        .def(nb::init<>())
        .def(nb::init<const int, const int, const Vector2 &>())
        .def(nb::init<const Measurement &>())
        .def(
            "__copy__", [](const Measurement &self) { return new Measurement(self); },
            nb::rv_policy::take_ownership)
        .def_ro("pose_id", &Measurement::pose_id)
        .def_ro("landmark_id", &Measurement::landmark_id)
        .def_ro("image_point", &Measurement::image_point)
        .def("__str__", &Measurement::toString);

    nb::class_<TrajPoint>(m, "TrajPoint")
        .def(nb::init<>())
        .def(nb::init<const int, const Pose3 &, const Pose3 &>())
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
        .def("getOdometryPoses3", &Dataset::getOdometryPoses3)
        .def("getGroundTruthPoses3", &Dataset::getGroundTruthPoses3)
        .def("getLandmarkPositions", &Dataset::getLandmarkPositions)
        .def("__str__", &Dataset::toString);

    nb::class_<State>(m, "State")
        .def(nb::init<>())
        .def_ro("robot_poses", &State::robot_poses)
        .def_ro("landmarks", &State::landmarks)
        .def("getLandmarkPositions", &State::getLandmarkPositions,
             "Get the positions of all landmarks in the current state.");

    nb::class_<BundleAdjustmentConfig>(m, "BundleAdjustmentConfig")
        .def(nb::init<>())
        .def_rw("max_iterations", &BundleAdjustmentConfig::max_iterations)
        .def_rw("tolerance", &BundleAdjustmentConfig::tolerance)
        .def_rw("pose_pose", &BundleAdjustmentConfig::pose_pose)
        .def_rw("patience", &BundleAdjustmentConfig::patience);

    nb::class_<BundleAdjuster::OptimizationStats>(m, "OptimizationStats")
        .def_ro("num_iterations", &BundleAdjuster::OptimizationStats::num_iterations)
        .def_ro("pose_chi", &BundleAdjuster::OptimizationStats::pose_chi)
        .def_ro("landmark_chi", &BundleAdjuster::OptimizationStats::landmark_chi)
        .def_ro("converged", &BundleAdjuster::OptimizationStats::converged)
        .def_ro("num_inliers", &BundleAdjuster::OptimizationStats::num_inliers)
        .def_ro("position_error", &BundleAdjuster::OptimizationStats::position_error)
        .def_ro("orientation_error", &BundleAdjuster::OptimizationStats::orientation_error)
        .def_ro("map_error", &BundleAdjuster::OptimizationStats::map_error)
        .def("__str__", &BundleAdjuster::OptimizationStats::toString);

    nb::class_<BundleAdjuster>(m, "BundleAdjuster")
        .def(nb::init<const std::vector<Landmark> &, const Dataset &, const BundleAdjustmentConfig &>())
        .def("performIteration", &BundleAdjuster::performIteration,
             "Perform a single iteration of bundle adjustment.")
        .def("getState", &BundleAdjuster::getState, "Get the current optimization state.");

    nb::class_<TriangulationConfig>(m, "TriangulationConfig")
        .def(nb::init<>())
        .def_rw("homogeneous_threshold", &TriangulationConfig::homogeneous_threshold)
        .def_rw("enable_cheirality_check", &TriangulationConfig::enable_cheirality_check)
        .def_rw("min_observations", &TriangulationConfig::min_observations);

    nb::class_<Triangulator>(m, "Triangulator")
        .def(nb::init<const Camera &>())
        .def(nb::init<const Camera &, const TriangulationConfig &>())
        .def("triangulateLandmark", &Triangulator::triangulateLandmark,
             "Triangulate a single landmark from multiple observations.")
        .def("triangulateAll", &Triangulator::triangulateAll,
             "Triangulate all landmarks from grouped measurements.")
        .def("triangulateFromDataset", &Triangulator::triangulateFromDataset,
             "Triangulate all landmarks in a dataset.")
        .def("triangulateRansac", &Triangulator::triangulateRansac,
             "Triangulate landmarks using robust consecutive frame approach with outlier filtering.")
        .def("getCamera", &Triangulator::getCamera, nb::rv_policy::reference_internal)
        .def("getConfig", &Triangulator::getConfig, nb::rv_policy::reference_internal)
        .def("setConfig", &Triangulator::setConfig);

    m.def("triangulate", &triangulate, nb::arg("dataset"),
          "Triangulate landmarks from the given dataset and return a vector of landmarks.");

    m.def("triangulateRansac", &triangulateRansac, nb::arg("dataset"),
          "Triangulate landmarks using robust consecutive frame approach with outlier filtering and averaging.");
};

}  // namespace python
}  // namespace pms
