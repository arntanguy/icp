#include "error_point_to_point.hpp"


namespace icp
{
  
template<typename Dtype>
void ErrorPointToPoint<Dtype>::computeJacobian() {
      const int n = pc_d_->size();
      J_.setZero(3 * n, 6);
      for (unsigned int i = 0; i < n; ++i)
      {
        const pcl::PointXYZ &p = (*pc_d_)[i];
        J_.row(i * 3)     << -1,     0,    0,    0,   -p.z,   p.y;
        J_.row(i * 3 + 1) <<  0,    -1,    0,  p.z,      0,  -p.x;
        J_.row(i * 3 + 2) <<  0,     0,   -1, -p.y,    p.x,     0;
      }
}

template<typename Dtype>
void ErrorPointToPoint<Dtype>::computeError() {
  // XXX: Does not make use of eigen's map, possible optimization for floats

  Pc::Ptr pc_e = pcltools::substractPointcloud<pcl::PointXYZ>(pc_d_, pc_m_);
  //Eigen::MatrixXf matrixMap = pc_m_->getMatrixXfMap(3, 4, 0) - pc_d_->getMatrixXfMap(3, 4, 0);

  for (unsigned int i = 0; i < pc_e->size(); ++i)
  {
    const pcl::PointXYZ &p = (*pc_e)[i];
    errorVector_[i * 3] =  weights_(i, 0) * p.x;
    errorVector_[i * 3 + 1] =  weights_(i, 1) * p.y;
    errorVector_[i * 3 + 2] =  weights_(i, 2) * p.z;
  }
  if(!errorVector_.allFinite()) {
    LOG(WARNING) << "Error Vector has NaN values\n!" << errorVector_;
    LOG(WARNING) << "Displaying p_e";
    for(int i=0; i < pc_e->size(); i++) {
      LOG(WARNING) << (*pc_e)[i];
    }
    LOG(WARNING) << "Displaying pc_d_";
    for(int i=0; i < pc_d_->size(); i++) {
      LOG(WARNING) << (*pc_d_)[i];
    }
    LOG(WARNING) << "Displaying pc_m_";
    for(int i=0; i < pc_m_->size(); i++) {
      LOG(WARNING) << (*pc_m_)[i];
    }
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


// Explicit instantiation
template class ErrorPointToPoint<float>;

} /* icp */ 

