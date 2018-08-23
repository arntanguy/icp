#include <icp/error_point_to_point.hpp>
#include <icp/instanciate.hpp>
#include <icp/logging.hpp>


namespace icp
{

template<typename Scalar, typename PointReference, typename PointSource>
void ErrorPointToPoint<Scalar, PointReference, PointSource>::computeJacobian() {
  const unsigned int n = reference_->size();
  JacobianMatrix J;
  J.setZero(3 * n, 6);
  pcl::PointXYZ p;
  for (unsigned int i = 0; i < n; ++i)
  {
    const PointReference &p_t =  (*reference_)[i];
    p.x = p_t.x;
    p.y = p_t.y;
    p.z = p_t.z;
    J.row(i * 3)     <<  -1,     0,    0,    0,   -p.z,   p.y;
    J.row(i * 3 + 1) <<  0,    -1,    0,  p.z,      0,  -p.x;
    J.row(i * 3 + 2) <<  0,     0,   -1, -p.y,    p.x,     0;
  }
  constraints_->processJacobian(J, J_);
}

template<typename Scalar, typename PointReference, typename PointSource>
void ErrorPointToPoint<Scalar, PointReference, PointSource>::computeError() {
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

INSTANCIATE_ERROR_POINT_TO_POINT;

} /* icp */

