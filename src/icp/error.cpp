#include "error.hpp"

namespace icp {

template<typename Scalar, typename PointReference, typename PointCurrent>
void Error<Scalar, PointReference, PointCurrent>::setInputCurrent(const PctPtr& in) {
  current_ = in;

  // Resize the data structures
  errorVector_.resize(3 * current_->size(), Eigen::NoChange);
  weights_.resize(current_->size(), 3);
  weights_ = MatrixX::Ones(current_->size(), 3);
  J_.setZero(3 * current_->size(), 6);
}

template<typename Scalar, typename PointReference, typename PointCurrent>
void Error<Scalar, PointReference, PointCurrent>::setInputReference(const PcsPtr& in) {
  reference_ = in;
}

template class Error<float, pcl::PointXYZ, pcl::PointXYZ>;
template class Error<float, pcl::PointXYZ, pcl::PointNormal>;
template class Error<float, pcl::PointNormal, pcl::PointNormal>;

}  // namespace icp
