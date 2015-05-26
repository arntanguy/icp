//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#ifndef   ICP_ERROR_POINT_TO_PLANE_SO3_HPP
#define   ICP_ERROR_POINT_TO_PLANE_SO3_HPP

#include <Eigen/Core>
#include <Eigen/Dense>
#include "error.hpp"
#include "pcltools.hpp"

#define DEFINE_ERROR_POINT_TO_PLANE_SO3_TYPES(Scalar, Suffix) \
  typedef ErrorPointToPlaneSO3<Scalar, pcl::PointNormal, pcl::PointNormal> ErrorPointToPlaneSO3Normal##Suffix;


namespace icp {

/**
 * @brief Compute the point to point error for ICP
 *
 * \f[ e = P^* - P \f]
 *
 * Where \f$ P^* \f$ is the reference point cloud and \f$ P \f$ is the
 * transformed point cloud (the one we want to register).
 */
template<typename Scalar, typename PointReference, typename PointCurrent>
class ErrorPointToPlaneSO3 : public Error<Scalar, 6, PointReference, PointCurrent> {
  public:
    typedef typename pcl::PointCloud<PointCurrent> Pcs;
    typedef typename pcl::PointCloud<PointReference> Pcr;
    typedef typename Pcs::Ptr PcsPtr;
    typedef typename Pcr::Ptr PcrPtr;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ErrorVector;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 6> JacobianMatrix;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
    using Error<Scalar, 6, PointReference, PointCurrent>::errorVector_;
    using Error<Scalar, 6, PointReference, PointCurrent>::J_;
    using Error<Scalar, 6, PointReference, PointCurrent>::current_;
    using Error<Scalar, 6, PointReference, PointCurrent>::reference_;
    using Error<Scalar, 6, PointReference, PointCurrent>::weights_;

    //! Compute the error
    /*! \f[ e(x) = n(P-T(x)\hat{T}P^* \f]
     *
     *  Stack the error in vectors of form
     *
     * \f[ eg = [e_0; e_1; ...; e_n] \in {R}^{n\times1}; \f]
       */
    virtual void computeError();

    //! Jacobian of \f$ e(x) \f$, eg \f[ J = \frac{de}{dx} \f]
    /*!
        For a 3D point of coordinates \f$ (X, Y, Z) \f$, the jacobian is
        \f[ n^T \left( \begin{array}{cccccc}
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
    
    virtual void setInputReference(const PcrPtr& in);
    virtual void setInputCurrent(const PcsPtr& in);

    virtual Eigen::Matrix<Scalar, 4, 4> update();
};

DEFINE_ERROR_POINT_TO_PLANE_SO3_TYPES(float, )
DEFINE_ERROR_POINT_TO_PLANE_SO3_TYPES(float, f)

}  // namespace icp

#endif
