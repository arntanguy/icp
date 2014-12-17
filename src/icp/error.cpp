#include "error.hpp"

namespace icp {

template<typename Scalar>
void Error<Scalar>::setInputTarget(const Pc::Ptr& in) {
  pc_m_ = in;

  // Resize the data structures
  errorVector_.resize(3 * pc_m_->size(), Eigen::NoChange);
  weights_.resize(pc_m_->size(), 3);
  weights_ = MatrixX::Ones(pc_m_->size(), 3);
  J_.setZero(3 * pc_m_->size(), 6);
}

template<typename Scalar>
void Error<Scalar>::setInputSource(const Pc::Ptr& in) {
  pc_d_ = in;
}

template class Error<float>;

}  // namespace icp
