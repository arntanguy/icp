#include "error.hpp"

namespace icp {

template<typename Scalar>
void Error<Scalar>::setInputTarget(const Pc::Ptr& in) {
  target_ = in;

  // Resize the data structures
  errorVector_.resize(3 * target_->size(), Eigen::NoChange);
  weights_.resize(target_->size(), 3);
  weights_ = MatrixX::Ones(target_->size(), 3);
  J_.setZero(3 * target_->size(), 6);
}

template<typename Scalar>
void Error<Scalar>::setInputSource(const Pc::Ptr& in) {
  source_ = in;
}

template class Error<float>;

}  // namespace icp
