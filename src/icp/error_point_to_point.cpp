#include "error_point_to_point.hpp"
#include "instanciate.hpp"
#include "linear_algebra.hpp"
#include "logging.hpp"


namespace icp
{

template<typename Scalar, typename Point>
void ErrorPointToPoint<Scalar, Point>::computeJacobian() {
  const int n = reference_->size();
  JacobianMatrix J;
  J.setZero(3 * n, 6);
  pcl::PointXYZ p;
  for (unsigned int i = 0; i < n; ++i)
  {
    const Point &p_t =  (*reference_)[i];
    p.x = p_t.x;
    p.y = p_t.y;
    p.z = p_t.z;
    J.row(i * 3)     << -1,     0,    0,    0,   -p.z,   p.y;
    J.row(i * 3 + 1) <<  0,    -1,    0,  p.z,      0,  -p.x;
    J.row(i * 3 + 2) <<  0,     0,   -1, -p.y,    p.x,     0;
  }
  constraints_.processJacobian(J, J_);
}

template<typename Scalar, typename Point>
void ErrorPointToPoint<Scalar, Point>::computeError() {
  // XXX: Does not make use of eigen's map, possible optimization for floats

  typename pcl::PointCloud<Point>::Ptr pc_e = pcltools::substractPointcloud<Point, Point>(current_, reference_);
  //Eigen::MatrixXf matrixMap = current_->getMatrixXfMap(3, 4, 0) - reference_->getMatrixXfMap(3, 4, 0);

  pcl::PointXYZ p;
  for (unsigned int i = 0; i < pc_e->size(); ++i)
  {
    const Point &p_t = (*pc_e)[i];
    p.x = p_t.x;
    p.y = p_t.y;
    p.z = p_t.z;
    errorVector_[i * 3] =  weights_(i, 0) * p.x;
    errorVector_[i * 3 + 1] =  weights_(i, 1) * p.y;
    errorVector_[i * 3 + 2] =  weights_(i, 2) * p.z;
  }
  if (!errorVector_.allFinite()) {
    LOG(WARNING) << "Error Vector has NaN values\n!" << errorVector_;
  }
}

template<typename Scalar, typename Point>
Eigen::Matrix<Scalar, 4, 4> ErrorPointToPoint<Scalar, Point>::update() {
  auto Jt = J_.transpose();
  Eigen::Matrix<Scalar, 6, 1> x = constraints_.getTwist(-(Jt * J_).ldlt().solve(Jt * errorVector_));
  // return update step transformation matrix
  return  la::expLie(x);
}

INSTANCIATE_ERROR_POINT_TO_POINT;

} /* icp */

