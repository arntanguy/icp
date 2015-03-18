#include "error_point_to_plane.hpp"


namespace icp
{

template<typename Dtype, typename PointSource, typename PointTarget>
void ErrorPointToPlane<Dtype, PointSource, PointTarget>::computeJacobian() {
  const int n = target_->size();
  J_.setZero(n, 6);
  for (unsigned int i = 0; i < n; ++i)
  {
    const PointTarget &p = (*target_)[i];
    J_.row(i) << p.normal_x, p.normal_y, p.normal_z,
           p.y *p.normal_z - p.z *p.normal_y,
           p.z *p.normal_x - p.x *p.normal_z,
           p.y *p.normal_y - p.y *p.normal_x;
  }
}

template<typename Dtype, typename PointSource, typename PointTarget>
void ErrorPointToPlane<Dtype, PointSource, PointTarget>::computeError() {
  // XXX: Does not make use of eigen's map, possible optimization for floats

  PcsPtr pc_e = pcltools::substractPointcloud<PointSource, PointTarget>(source_, target_);
  //Eigen::MatrixXf matrixMap = target_->getMatrixXfMap(3, 4, 0) - source_->getMatrixXfMap(3, 4, 0);

  for (unsigned int i = 0; i < pc_e->size(); ++i)
  {
    const pcl::PointXYZ &p = (*pc_e)[i];
    const pcl::PointNormal &n = (*target_)[i];
    errorVector_[i] =  weights_(i, 0) * n.normal_x * p.x
                       + weights_(i, 1) * n.normal_y * p.y
                       + weights_(i, 2) * n.normal_z * p.z;
  }
  //if (!errorVector_.allFinite()) {
   // LOG(WARNING) << "Error Vector has NaN values\n!" << errorVector_;
    LOG(WARNING) << "Displaying p_e";
    for (int i = 0; i < pc_e->size(); i++) {
      LOG(WARNING) << (*pc_e)[i];
      LOG(WARNING) << (*target_)[i];
    }
   // LOG(WARNING) << "Displaying source_";
   // for (int i = 0; i < source_->size(); i++) {
   //   LOG(WARNING) << (*source_)[i];
   // }
   // LOG(WARNING) << "Displaying target_";
   // for (int i = 0; i < target_->size(); i++) {
   //   LOG(WARNING) << (*target_)[i];
   // }
  //}
}

template<typename Scalar, typename PointSource, typename PointTarget>
void ErrorPointToPlane<Scalar, PointSource, PointTarget>::setInputTarget(const PctPtr &in) {
  target_ = in;

  // Resize the data structures
  errorVector_.resize(target_->size(), Eigen::NoChange);
  weights_.resize(target_->size(), 3);
  weights_ = MatrixX::Ones(target_->size(), 3);
  J_.setZero(target_->size(), 6);
}

template<typename Scalar, typename PointSource, typename PointTarget>
void ErrorPointToPlane<Scalar, PointSource, PointTarget>::setInputSource(const PcsPtr &in) {
  source_ = in;
}

/**
 * @brief Specialization for float type (TODO)
 * This version of the error computation makes use of the fast matrix map
 * between the internal representation and Eigen's. The matrix map assumes
 * floats for speed improvement, and thus is not applicable for generic types
 */
//template<typename Dtype>
//void ErrorPointToPlane<Dtype>::computeError() {
//  LOG(WARNING) << "Warning: assuming float, there might be a loss of precision!";
//  LOG(WARNING) <<
//               "Warning: this has not been tested on anything else than floats. It probably won't work"
//               "for arbitrary types!";
//  ErrorPointToPlane<float>::computeError();
//}
//


// Explicit instantiation
template class ErrorPointToPlane<float, pcl::PointXYZ, pcl::PointNormal>;
typedef ErrorPointToPlane<float, pcl::PointXYZ, pcl::PointNormal> ErrorPointToPlanef;

} /* icp */

