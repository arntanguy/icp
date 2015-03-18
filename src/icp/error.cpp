#include "error.hpp"

namespace icp {

template<typename Scalar, typename PointSource, typename PointTarget>
void Error<Scalar, PointSource, PointTarget>::setInputTarget(const PctPtr& in) {
  target_ = in;

  // Resize the data structures
  errorVector_.resize(3 * target_->size(), Eigen::NoChange);
  weights_.resize(target_->size(), 3);
  weights_ = MatrixX::Ones(target_->size(), 3);
  J_.setZero(3 * target_->size(), 6);
}

template<typename Scalar, typename PointSource, typename PointTarget>
void Error<Scalar, PointSource, PointTarget>::setInputSource(const PcsPtr& in) {
  source_ = in;
}

template class Error<float, pcl::PointXYZ, pcl::PointXYZ>;
template class Error<float, pcl::PointXYZ, pcl::PointNormal>;

}  // namespace icp
