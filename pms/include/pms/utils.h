#pragma once

#include <Eigen/Dense>

namespace pms {
inline Eigen::IOFormat PythonFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");
}
