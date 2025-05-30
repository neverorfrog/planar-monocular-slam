#pragma once

#include <Eigen/Dense>
#include "Eigen/Core"

namespace pms {
inline Eigen::IOFormat PythonFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");
inline Eigen::IOFormat CleanFmt(4, 0, " ", "\n", "", "");
}
