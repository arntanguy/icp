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

namespace icp {

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
    using Error<Dtype>::weights_;

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
    virtual void computeJacobian();

    virtual JacobianMatrix getJacobian() const {
      return J_;
    }
    virtual ErrorVector getErrorVector() const {
      return errorVector_;
    }
};


}  // namespace icp

#endif
