//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#ifndef   ERROR_POINT_TO_POINT_SIM3_HPP
#define   ERROR_POINT_TO_POINT_SIM3_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include "error.hpp"
#include "pcltools.hpp"

namespace icp {

/**
 * @brief Compute the point to point error for ICP
 *
 * \f[ e = P - T\hat{T}P^* \f]
 *
 * Where \f$ P^* \f$ is the reference point cloud and \f$ P \f$ is the
 * transformed point cloud (the one we want to register).
 */
template<typename Dtype, typename Point>
class ErrorPointToPointSim3 : public Error<Dtype, 7, Point, Point> {
  public:
    typedef pcl::PointCloud<pcl::PointXYZ> Pc;
    typedef Eigen::Matrix<Dtype, Eigen::Dynamic, 1> ErrorVector;
    typedef Eigen::Matrix<Dtype, Eigen::Dynamic, 7> JacobianMatrix;
    using Error<Dtype, 7, Point, Point>::errorVector_;
    using Error<Dtype, 7, Point, Point>::J_;
    using Error<Dtype, 7, Point, Point>::current_;
    using Error<Dtype, 7, Point, Point>::reference_;
    using Error<Dtype, 7, Point, Point>::weights_;

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
         1  & 0  & 0  &  0  &  Z  & -Y & X \\
         0  & 1  & 0  & -Z  &  0  &  X & Y \\
         0  & 0  & 1  &  Y  & -X  &  0 & Z \\
          \end{array} \right)
        \f]
    */
    virtual void computeJacobian();

    virtual JacobianMatrix getJacobian() const {
      return J_;
    }
    virtual ErrorVector getErrorVector() const {
      return errorVector_;
    }
};

typedef ErrorPointToPointSim3<float, pcl::PointXYZ> ErrorPointToPointXYZSim3;

}  // namespace icp

#endif
