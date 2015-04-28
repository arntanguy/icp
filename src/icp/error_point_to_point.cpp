#include "error_point_to_point.hpp"
#include "instanciate.hpp"


namespace icp
{

template<typename Dtype, typename Point>
void ErrorPointToPoint<Dtype, Point>::computeJacobian() {
  const int n = reference_->size();
  J_.setZero(3 * n, 6);
  pcl::PointXYZ p;
  for (unsigned int i = 0; i < n; ++i)
  {
    const Point &p_t =  (*reference_)[i];
    p.x = p_t.x;
    p.y = p_t.y;
    p.z = p_t.z;
    J_.row(i * 3)     << -1,     0,    0,    0,   -p.z,   p.y;
    J_.row(i * 3 + 1) <<  0,    -1,    0,  p.z,      0,  -p.x;
    J_.row(i * 3 + 2) <<  0,     0,   -1, -p.y,    p.x,     0;
  }
}

template<typename Dtype, typename Point>
void ErrorPointToPoint<Dtype, Point>::computeError() {
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
    //LOG(WARNING) << "Displaying p_e";
    //for (int i = 0; i < pc_e->size(); i++) {
    //  LOG(WARNING) << (*pc_e)[i];
    //}
    //LOG(WARNING) << "Displaying reference_";
    //for (int i = 0; i < reference_->size(); i++) {
    //  LOG(WARNING) << (*reference_)[i];
    //}
    //LOG(WARNING) << "Displaying current_";
    //for (int i = 0; i < current_->size(); i++) {
    //  LOG(WARNING) << (*current_)[i];
    //}
  }
}


/**
 * @brief Specialization for float type (TODO)
 * This version of the error computation makes use of the fast matrix map
 * between the internal representation and Eigen's. The matrix map assumes
 * floats for speed improvement, and thus is not applicable for generic types
 */
//template<typename Dtype>
//void ErrorPointToPoint<Dtype>::computeError() {
//  LOG(WARNING) << "Warning: assuming float, there might be a loss of precision!";
//  LOG(WARNING) <<
//               "Warning: this has not been tested on anything else than floats. It probably won't work"
//               "for arbitrary types!";
//  ErrorPointToPoint<float>::computeError();
//}
//

INSTANCIATE_ERROR_POINT_TO_POINT;

} /* icp */

