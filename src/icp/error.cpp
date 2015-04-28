#include "error.hpp"
#include "instanciate.hpp"

namespace icp {

template<typename Scalar, unsigned int DegreesOfFreedom, typename PointReference, typename PointCurrent>
void Error<Scalar, DegreesOfFreedom, PointReference, PointCurrent>::setInputCurrent(const PctPtr& in) {
  current_ = in;

  // Resize the data structures
  errorVector_.resize(3 * current_->size(), Eigen::NoChange);
  weights_.resize(current_->size(), 3);
  weights_ = MatrixX::Ones(current_->size(), 3);
  J_.setZero(3 * current_->size(), DegreesOfFreedom);
}

template<typename Scalar, unsigned int DegreesOfFreedom, typename PointReference, typename PointCurrent>
void Error<Scalar, DegreesOfFreedom, PointReference, PointCurrent>::setInputReference(const PcsPtr& in) {
  reference_ = in;
}

INSTANCIATE_ERROR;

}  // namespace icp
