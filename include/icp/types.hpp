#pragma once

#include <Eigen/Core>

namespace icp
{

template<typename Scalar>
using VectorX = typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

}
