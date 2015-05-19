#include "error.hpp"
#include "instanciate.hpp"
#include "linear_algebra.hpp"

namespace icp {

template<typename Scalar, unsigned int DegreesOfFreedom, typename PointReference, typename PointCurrent>
void Error<Scalar, DegreesOfFreedom, PointReference, PointCurrent>::setInputCurrent(const PcsPtr &in) {
  current_ = in;

  // Resize the data structures
  errorVector_.resize(3 * current_->size(), Eigen::NoChange);
  weights_.resize(current_->size(), 3);
  weights_ = MatrixX::Ones(current_->size(), 3);
  J_.setZero(3 * current_->size(), DegreesOfFreedom);
}

template<typename Scalar, unsigned int DegreesOfFreedom, typename PointReference, typename PointCurrent>
Eigen::Matrix<Scalar, 4, 4> Error<Scalar, DegreesOfFreedom, PointReference, PointCurrent>::update() {
  auto Jt = J_.transpose();
  Eigen::Matrix<Scalar, DegreesOfFreedom, 1> x = -constraints_->getTwist((Jt*J_).ldlt().solve(Jt * errorVector_));
  // return update step transformation matrix
  return  la::expLie(x);
}

template<typename Scalar, unsigned int DegreesOfFreedom, typename PointReference, typename PointCurrent>
void Error<Scalar, DegreesOfFreedom, PointReference, PointCurrent>::setInputReference(const PcrPtr &in) {
  reference_ = in;
}

INSTANCIATE_ERROR;

}  // namespace icp
