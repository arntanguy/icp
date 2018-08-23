#include <icp/error.hpp>
#include <icp/instanciate.hpp>
#include <icp/linear_algebra.hpp>
#include <icp/mestimator.hpp>

namespace icp {

template<typename Scalar, unsigned int DegreesOfFreedom, typename PointReference, typename PointCurrent>
void Error<Scalar, DegreesOfFreedom, PointReference, PointCurrent>::setInputCurrent(const PcsPtr &in) {
  current_ = in;

  // Resize the data structures
  errorVector_.resize(3 * current_->size(), Eigen::NoChange);
  weightsVector_ = VectorX::Ones(current_->size() * 3);
  J_.setZero(3 * current_->size(), DegreesOfFreedom);
}

template<typename Scalar, unsigned int DegreesOfFreedom, typename PointReference, typename PointCurrent>
Eigen::Matrix<Scalar, 4, 4> Error<Scalar, DegreesOfFreedom, PointReference, PointCurrent>::update() {
  auto Jt = J_.transpose();
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> W = weightsVector_.asDiagonal().inverse();
  Eigen::Matrix<Scalar, DegreesOfFreedom, 1> x = -constraints_->getTwist((Jt*W*J_).ldlt().solve(Jt * W * errorVector_));
  // return update step transformation matrix
  return  la::expLie(x);
}

template<typename Scalar, unsigned int DegreesOfFreedom, typename PointReference, typename PointCurrent>
void Error<Scalar, DegreesOfFreedom, PointReference, PointCurrent>::setInputReference(const PcrPtr &in) {
  reference_ = in;
}

template<typename Scalar, unsigned int DegreesOfFreedom, typename PointReference, typename PointCurrent>
void Error<Scalar, DegreesOfFreedom, PointReference, PointCurrent>::computeWeights()
{
  Scalar mad = median_absolute_deviation(errorVector_);
  Scalar scale = mad / 0.6745;
  hubert_weight(errorVector_, weightsVector_, scale);
}

INSTANCIATE_ERROR;

}  // namespace icp
