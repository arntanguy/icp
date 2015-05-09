#include "error_point_to_point_sim3.hpp"
#include "instanciate.hpp"
#include "logging.hpp"


namespace icp
{
  
template<typename Scalar, typename Point>
void ErrorPointToPointSim3<Scalar, Point>::computeJacobian() {
      const int n = reference_->size();
      JacobianMatrix J;
      J.setZero(3 * n, 7);
      for (unsigned int i = 0; i < n; ++i)
      {
        const Point &p = (*reference_)[i];
        J.row(i * 3)     << -1,     0,    0,    0,   -p.z,   p.y,  -p.x;
        J.row(i * 3 + 1) <<  0,    -1,    0,  p.z,      0,  -p.x,  -p.y;
        J.row(i * 3 + 2) <<  0,     0,   -1, -p.y,    p.x,     0,  -p.z;
      }
      constraints_.processJacobian(J, J_);
}

template<typename Scalar, typename Point>
void ErrorPointToPointSim3<Scalar, Point>::computeError() {
  // XXX: Does not make use of eigen's map, possible optimization for floats
  typename pcl::PointCloud<Point>::Ptr pc_e = pcltools::substractPointcloud<Point, Point>(current_, reference_);
  //Eigen::MatrixXf matrixMap = current_->getMatrixXfMap(3, 4, 0) - reference_->getMatrixXfMap(3, 4, 0);

  for (unsigned int i = 0; i < pc_e->size(); ++i)
  {
    const Point &p = (*pc_e)[i];
    errorVector_[i * 3] =  weights_(i, 0) * p.x;
    errorVector_[i * 3 + 1] =  weights_(i, 1) * p.y;
    errorVector_[i * 3 + 2] =  weights_(i, 2) * p.z;
  }
  if(!errorVector_.allFinite()) {
    LOG(WARNING) << "Error Vector has NaN values\n!" << errorVector_;
  }
}

INSTANCIATE_ERROR_POINT_TO_POINT_SIM3;

} /* icp */ 

