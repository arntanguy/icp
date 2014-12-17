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
 *
 * \f[ e = P^* - P \f]
 *
 * Where \f$ P^* \f$ is the reference point cloud and \f$ P \f$ is the
 * transformed point cloud (the one we want to register).
 */
template<typename Dtype>
class ErrorPointToPoint : public Error<Dtype> {
  public:
    typedef pcl::PointCloud<pcl::PointXYZ> Pc;
    typedef Eigen::Matrix<Dtype, Eigen::Dynamic, 1> ErrorVector;
    typedef Eigen::Matrix<Dtype, Eigen::Dynamic, 6> JacobianMatrix;
    using Error<Dtype>::errorVector_;
    using Error<Dtype>::J_;
    using Error<Dtype>::target_;
    using Error<Dtype>::source_;
    using Error<Dtype>::weights_;

    //! Compute the error
    /*! \f[ e = P^* - P \f]
     *
     *  Stack the error in vectors of form
     *
     * \f[ eg = [ex_0; ey_0; ez_0; ex_1; ey_1; ez_1; ...; ex_n; ey_n; ez_n]; \f]
       */
    virtual void computeError();

    //! Jacobian of \f$ e(x) \f$, eg \f[ J = \frac{de}{dx} \f]
    /*!
        For a 3D point of coordinates \f$ (X, Y, Z) \f$, the jacobian is
        \f[ \left( \begin{array}{cccccc}
         1  & 0  & 0  &  0  &  Z  & -Y \\
         0  & 1  & 0  & -Z  &  0  &  X \\
         0  & 0  & 1  &  Y  & -X  &  0 \\
          \end{array} \right)
        \f]

        Note:

        We update the pose on the left hand side :
        \f[ \widehat{T} \leftarrow e^x*\widehat{T} \f]
        This means that the pose jacobian is computed  at \f$ x=\widehat{x} \f$,
        Eg;
        \f[ \frac{\partial (e^x*\widehat{T}*P)}{\partial P} =
        \frac{\partial e^x*Pe}{\partial P} = [eye(3) ; skew(Pe)];
        \f]

        If the update was computed on the right hand side :
        \f[ \widehat{T} \leftarrow \widehat{T}*e^x \f]
        The pose jacobian has to be estimated at \f$ x = 0 \f$
        eg \f[ \frac{\partial (\widehat{T}*e^x*P)}{\partial x} = \widehat{T}*[eye(3) skew(P)] \f]
        */
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
