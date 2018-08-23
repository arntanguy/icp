//  This file is part of the Icp_ Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
#include <icp/error_point_to_plane_so3.hpp>
#include <icp/instanciate.hpp>
#include <icp/linear_algebra.hpp>
#include <icp/logging.hpp>


namespace icp
{

template<typename Dtype, typename PointReference, typename PointCurrent>
void ErrorPointToPlaneSO3<Dtype, PointReference, PointCurrent>::computeJacobian() {
  const unsigned int n = current_->size();
  J_.setZero(n, 3);
  for (unsigned int i = 0; i < n; ++i)
  {
    const PointCurrent &p = (*current_)[i];
    J_.row(i) << p.y*p.normal_z - p.z*p.normal_y, p.z*p.normal_x - p.x*p.normal_z, p.x*p.normal_y - p.y*p.normal_x;
  }
}

template<typename Dtype, typename PointReference, typename PointCurrent>
void ErrorPointToPlaneSO3<Dtype, PointReference, PointCurrent>::computeError() {
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
  }
}

template<typename Scalar, typename PointReference, typename PointCurrent>
void ErrorPointToPlaneSO3<Scalar, PointReference, PointCurrent>::setInputCurrent(const PcsPtr &in) {
  current_ = in;

  // Resize the data structures
  errorVector_.resize(current_->size(), Eigen::NoChange);
  weightsVector_ = VectorX::Ones(current_->size());
  J_.setZero(current_->size(), 6);
}

template<typename Scalar, typename PointReference, typename PointCurrent>
void ErrorPointToPlaneSO3<Scalar, PointReference, PointCurrent>::setInputReference(const PcrPtr &in) {
  reference_ = in;
}

template<typename Scalar, typename PointReference, typename PointCurrent>
Eigen::Matrix<Scalar, 4, 4> ErrorPointToPlaneSO3<Scalar, PointReference, PointCurrent>::update() {
  auto Jt = J_.transpose();
  Eigen::Matrix<Scalar, 3, 1> x = -(Jt*J_).ldlt().solve(Jt * errorVector_);
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
  return  T;
}

/**
 * @brief Specialization for float type (TODO)
 * This version of the error computation makes use of the fast matrix map
 * between the internal representation and Eigen's. The matrix map assumes
 * floats for speed improvement, and thus is not applicable for generic types
 */
//template<typename Dtype>
//void ErrorPointToPlaneSO3<Dtype>::computeError() {
//  LOG(WARNING) << "Warning: assuming float, there might be a loss of precision!";
//  LOG(WARNING) <<
//               "Warning: this has not been tested on anything else than floats. It probably won't work"
//               "for arbitrary types!";
//  ErrorPointToPlaneSO3<float>::computeError();
//}
//

INSTANCIATE_ERROR_POINT_TO_PLANE_SO3;

} /* icp */

