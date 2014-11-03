#include "error.hpp"

namespace icp {

template<typename Scalar>
void Error<Scalar>::setModelPointCloud(const Pc::Ptr& model) {
  pc_m_ = model;

  // Resize the data structures
  errorVector_.resize(3 * pc_m_->size(), Eigen::NoChange);
  weights_.resize(pc_m_->size(), 3);
  weights_ = MatrixX::Ones(pc_m_->size(), 3);
  J_.setZero(3 * pc_m_->size(), 6);
}

template<typename Scalar>
void Error<Scalar>::setDataPointCloud(const Pc::Ptr& data) {
  pc_d_ = data;
}

template class Error<float>;

}  // namespace icp
