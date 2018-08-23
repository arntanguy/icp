#include <icp/error_point_to_point_so3.hpp>
#include <icp/instanciate.hpp>
#include <icp/logging.hpp>
#include <icp/linear_algebra.hpp>


namespace icp
{

template<typename Scalar, typename PointReference, typename PointSource>
void ErrorPointToPointSO3<Scalar, PointReference, PointSource>::computeJacobian() {
  const unsigned int n = reference_->size();
  JacobianMatrix J;
  J.setZero(3 * n, 3);
  pcl::PointXYZ p;
  for (unsigned int i = 0; i < n; ++i)
  {
    const PointReference &p_t =  (*reference_)[i];
    p.x = p_t.x;
    p.y = p_t.y;
    p.z = p_t.z;
    J.row(i * 3)     <<      0,   -p.z,   p.y;
    J.row(i * 3 + 1) <<    p.z,      0,  -p.x;
    J.row(i * 3 + 2) <<   -p.y,    p.x,     0;
  }
  J_ = J;
}

template<typename Scalar, typename PointReference, typename PointSource>
void ErrorPointToPointSO3<Scalar, PointReference, PointSource>::computeError() {
  // XXX: Does not make use of eigen's map, possible optimization for floats

  PcPtr pc_e = pcltools::substractPointcloud<PointSource, PointReference>(current_, reference_);
  //Eigen::MatrixXf matrixMap = current_->getMatrixXfMap(3, 4, 0) - reference_->getMatrixXfMap(3, 4, 0);

  pcl::PointXYZ p;
  for (unsigned int i = 0; i < pc_e->size(); ++i)
  {
    const PointSource &p_t = (*pc_e)[i];
    p.x = p_t.x;
    p.y = p_t.y;
    p.z = p_t.z;
    errorVector_[i * 3] = p.x;
    errorVector_[i * 3 + 1] =  p.y;
    errorVector_[i * 3 + 2] =  p.z;
  }
  if (!errorVector_.allFinite()) {
    LOG(WARNING) << "Error Vector has NaN values\n!" << errorVector_;
  }
}

template<typename Scalar, typename PointReference, typename PointCurrent>
Eigen::Matrix<Scalar, 4, 4> ErrorPointToPointSO3<Scalar, PointReference, PointCurrent>::update() {
  auto Jt = J_.transpose();
  Eigen::Matrix<Scalar, 3, 1> x = -constraints_->getTwist((Jt*J_).ldlt().solve(Jt * errorVector_));
  // return update step transformation matrix
  Eigen::Matrix<Scalar, 4, 4> T = Eigen::Matrix<Scalar, 4, 4>::Identity();
  Eigen::Matrix<Scalar, 3, 3> rot = la::expSO3(x);
  T(0, 0) = rot(0, 0);
  T(0, 1) = rot(0, 1);
  T(0, 2) = rot(0, 2);
  T(1, 0) = rot(1, 0);
  T(1, 1) = rot(1, 1);
  T(1, 2) = rot(1, 2);
  T(2, 0) = rot(2, 0);
  T(2, 1) = rot(2, 1);
  T(2, 2) = rot(2, 2);
  LOG(INFO) << "Tso3: " << T;
  return  T;
}

INSTANCIATE_ERROR_POINT_TO_POINT_SO3;

} /* icp */

