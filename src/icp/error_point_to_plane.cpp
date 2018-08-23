//  This file is part of the Icp_ Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
#include <icp/error_point_to_plane.hpp>
#include <icp/instanciate.hpp>
#include <icp/logging.hpp>


namespace icp
{

template<typename Dtype, typename PointReference, typename PointCurrent>
void ErrorPointToPlane<Dtype, PointReference, PointCurrent>::computeJacobian() {
  const unsigned int n = current_->size();
  J_.setZero(n, 6);
  for (unsigned int i = 0; i < n; ++i)
  {
    const PointCurrent &p = (*current_)[i];
    J_.row(i) << p.normal_x, p.normal_y, p.normal_z,
           p.y *p.normal_z - p.z *p.normal_y,
           p.z *p.normal_x - p.x *p.normal_z,
           p.y *p.normal_y - p.y *p.normal_x;
  }
}

template<typename Dtype, typename PointReference, typename PointCurrent>
void ErrorPointToPlane<Dtype, PointReference, PointCurrent>::computeError() {
  // XXX: Does not make use of eigen's map, possible optimization for floats

  PcrPtr pc_e = pcltools::substractPointcloud<PointReference, PointCurrent>(reference_, current_);
  //Eigen::MatrixXf matrixMap = current_->getMatrixXfMap(3, 4, 0) - reference_->getMatrixXfMap(3, 4, 0);

  for (unsigned int i = 0; i < pc_e->size(); ++i)
  {
    const auto &p = (*pc_e)[i];
    const PointCurrent &n = (*current_)[i];
    errorVector_[i] =  n.normal_x * p.x
                       + n.normal_y * p.y
                       + n.normal_z * p.z;
  }
  if (!errorVector_.allFinite()) {
    LOG(WARNING) << "Error Vector has NaN values\n!" << errorVector_;
    LOG(WARNING) << "Displaying p_e";
    for (unsigned int i = 0; i < pc_e->size(); i++) {
      LOG(WARNING) << (*pc_e)[i];
    }
    LOG(WARNING) << "Displaying reference_";
    for (unsigned int i = 0; i < reference_->size(); i++) {
      LOG(WARNING) << (*reference_)[i];
    }
    LOG(WARNING) << "Displaying current_";
    for (unsigned int i = 0; i < current_->size(); i++) {
      LOG(WARNING) << (*current_)[i];
    }
  }
}

template<typename Scalar, typename PointReference, typename PointCurrent>
void ErrorPointToPlane<Scalar, PointReference, PointCurrent>::setInputCurrent(const PcsPtr &in) {
  current_ = in;

  // Resize the data structures
  errorVector_.resize(current_->size(), Eigen::NoChange);
  weightsVector_ = VectorX::Ones(current_->size());
  J_.setZero(current_->size(), 6);
}

template<typename Scalar, typename PointReference, typename PointCurrent>
void ErrorPointToPlane<Scalar, PointReference, PointCurrent>::setInputReference(const PcrPtr &in) {
  reference_ = in;
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

INSTANCIATE_ERROR_POINT_TO_PLANE;

} /* icp */

