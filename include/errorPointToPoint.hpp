//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#ifndef   ERROR_POINT_TO_POINT_HPP
#define   ERROR_POINT_TO_POINT_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include "error.hpp"
#include "pcltools.hpp"

/**
 * @brief Compute the point to point error for ICP
 */
template<typename Dtype>
class ErrorPointToPoint : public Error<Dtype> {
  public:
    typedef pcl::PointCloud<pcl::PointXYZ> Pc;
    typedef Eigen::Matrix<Dtype, Eigen::Dynamic, 1> ErrorVector;
    typedef Eigen::Matrix<Dtype, Eigen::Dynamic, 6> JacobianMatrix;
    using Error<Dtype>::errorVector_;
    using Error<Dtype>::J_;
    using Error<Dtype>::pc_m_;
    using Error<Dtype>::pc_d_;

    //! Compute the error
    /*! e = P_reference - P_current_transform
       Stack the error in vectors of form
       eg = [ex_0; ey_0; ez_0; ex_1; ey_1; ez_1; ...; ex_n; ey_n; ez_n];
       */
    virtual void computeError();

    //! Jacobian of e(x), eg de/dx  (4th row is  allways 0)
    /*! [ 1, 0, 0,  0,  Z, -Y]
        [ 0, 1, 0, -Z,  0,  X]
        [ 0, 0, 1,  Y, -X,  0]
        Note
        We update the pose on the left hand side :
        hat_T <-- exp(x)*hat_T
        This means that the pose jacobian is computed  at x=hat_x,
        Eg;
        dexp(x)*hat_T*P/dP = dexp(x)*Pe/dP = [eye(3) ; skew(Pe)];

        If the update was computed on the right hand side :
        hat_T <-- hat_T*exp(x)
        The pose jacobian has to be estimated at x = 0
        eg dhat_T*exp(x)*P/dx = hat_T*[eye(3) skew(P)] */
    virtual void computeJacobian() {
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

    virtual JacobianMatrix getJacobian() const {
      return J_;
    }
    virtual ErrorVector getErrorVector() const {
      return errorVector_;
    }
};


/**
 * @brief Specialization for float type
 * This version of the error computation makes use of the fast matrix map
 * between the internal representation and Eigen's. The matrix map assumes
 * floats for speed improvement, and thus is not applicable for generic types
 */
template<typename Dtype>
void ErrorPointToPoint<Dtype>::computeError() {
  // XXX: Does not make use of eigen's map, possible optimization for floats

  Pc::Ptr pc_e = pcltools::substractPointcloud<pcl::PointXYZ>(pc_d_, pc_m_);
  //Eigen::MatrixXf matrixMap = pc_m_->getMatrixXfMap(3, 4, 0) - pc_d_->getMatrixXfMap(3, 4, 0);

  for (unsigned int i = 0; i < pc_e->size(); ++i)
  {
    const pcl::PointXYZ &p = (*pc_e)[i];
    errorVector_[i * 3] = p.x;
    errorVector_[i * 3 + 1] = p.y;
    errorVector_[i * 3 + 2] = p.z;
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

//template<typename Dtype>
//void ErrorPointToPoint<Dtype>::computeError() {
//  LOG(WARNING) << "Warning: assuming float, there might be a loss of precision!";
//  LOG(WARNING) <<
//               "Warning: this has not been tested on anything else than floats. It probably won't work"
//               "for arbitrary types!";
//  ErrorPointToPoint<float>::computeError();
//}

#endif
