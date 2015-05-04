//  This file is part of the Icp_ Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#include "error_point_to_plane_sim3.hpp"
#include "instanciate.hpp"


namespace icp
{

template<typename Scalar, typename Point>
void ErrorPointToPlaneSim3<Scalar, Point>::computeJacobian() {
  const int n = current_->size();
  J_.setZero(n, 7);
  for (unsigned int i = 0; i < n; ++i)
  {
    const Point &p = (*current_)[i];
    J_.row(i) << p.normal_x, p.normal_y, p.normal_z,
           p.y *p.normal_z - p.z *p.normal_y,
           p.z *p.normal_x - p.x *p.normal_z,
           p.y *p.normal_y - p.y *p.normal_x,
           p.x * p.normal_x + p.y * p.normal_y + p.z * p.normal_z;
  }
}

template<typename Scalar, typename Point>
void ErrorPointToPlaneSim3<Scalar, Point>::computeError() {
  // XXX: Does not make use of eigen's map, possible optimization for floats

  PcPtr pc_e = pcltools::substractPointcloud<Point, Point>(reference_, current_);
  //Eigen::MatrixXf matrixMap = current_->getMatrixXfMap(3, 4, 0) - reference_->getMatrixXfMap(3, 4, 0);

  for (unsigned int i = 0; i < pc_e->size(); ++i)
  {
    const auto &p = (*pc_e)[i];
    const pcl::PointNormal &n = (*current_)[i];
    errorVector_[i] =  weights_(i, 0) * n.normal_x * p.x
                       + weights_(i, 1) * n.normal_y * p.y
                       + weights_(i, 2) * n.normal_z * p.z;
  }
  if (!errorVector_.allFinite()) {
    LOG(WARNING) << "Error Vector has NaN values\n!" << errorVector_;
  }
}

template<typename Scalar, typename Point>
void ErrorPointToPlaneSim3<Scalar, Point>::setInputCurrent(const PcPtr& in) {
  current_ = in;

  // Resize the data structures
  errorVector_.resize(current_->size(), Eigen::NoChange);
  weights_.resize(current_->size(), 3);
  weights_ = MatrixX::Ones(current_->size(), 3);
  J_.setZero(current_->size(), 7);
}

template<typename Scalar, typename Point>
void ErrorPointToPlaneSim3<Scalar, Point>::setInputReference(const PcPtr &in) {
  reference_ = in;
}

/**
 * @brief Specialization for float type (TODO)
 * This version of the error computation makes use of the fast matrix map
 * between the internal representation and Eigen's. The matrix map assumes
 * floats for speed improvement, and thus is not applicable for generic types
 */
//template<typename Scalar>
//void ErrorPointToPlaneSim3<Scalar>::computeError() {
//  LOG(WARNING) << "Warning: assuming float, there might be a loss of precision!";
//  LOG(WARNING) <<
//               "Warning: this has not been tested on anything else than floats. It probably won't work"
//               "for arbitrary types!";
//  ErrorPointToPlaneSim3<float>::computeError();
//}
//

INSTANCIATE_ERROR_POINT_TO_PLANE_SIM3;

} /* icp */

