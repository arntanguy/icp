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

#define DEFINE_ERROR_POINT_TO_POINT_SIM3_TYPES(Scalar, Suffix) \
  typedef ErrorPointToPointSim3<Scalar, pcl::PointXYZ, pcl::PointXYZ> ErrorPointToPointXYZSim3##Suffix; \
  typedef ErrorPointToPointSim3<Scalar, pcl::PointXYZRGB, pcl::PointXYZRGB> ErrorPointToPointXYZRGBSim3##Suffix; \
  typedef ErrorPointToPointSim3<Scalar, pcl::PointXYZ, pcl::PointNormal> ErrorPointToPointNormalSim3##Suffix;

namespace icp {

/**
 * @brief Compute the point to point error for ICP
 *
 * \f[ e = P - T\hat{T}P^* \f]
 *
 * Where \f$ P^* \f$ is the reference point cloud and \f$ P \f$ is the
 * transformed point cloud (the one we want to register).
 */
template<typename Scalar, typename PointReference, typename PointSource>
class ErrorPointToPointSim3 : public Error<Scalar, 7, PointReference, PointSource> {
  public:
    typedef pcl::PointCloud<PointSource> Pc;
    typedef pcl::PointCloud<PointReference> Pr;
    typedef typename Pc::Ptr PcPtr;
    typedef typename Pr::Ptr PrPtr;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ErrorVector;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> JacobianMatrix;
    using Error<Scalar, 7, PointReference, PointSource>::errorVector_;
    using Error<Scalar, 7, PointReference, PointSource>::J_;
    using Error<Scalar, 7, PointReference, PointSource>::current_;
    using Error<Scalar, 7, PointReference, PointSource>::reference_;
    using Error<Scalar, 7, PointReference, PointSource>::weightsVector_;
    using Error<Scalar, 7, PointReference, PointSource>::constraints_;

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

DEFINE_ERROR_POINT_TO_POINT_SIM3_TYPES(float, );
DEFINE_ERROR_POINT_TO_POINT_SIM3_TYPES(float, f);

}  // namespace icp

#endif
