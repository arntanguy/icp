//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#ifndef   MESTIMATOR_HUBERT_HPP
#define   MESTIMATOR_HUBERT_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "eigentools.hpp"
#include "mestimator.hpp"

#define DEFINE_MESTIMATOR_TYPES(Scalar, Suffix) \
  typedef MEstimatorHubert<Scalar, pcl::PointXYZ, pcl::PointXYZ> MEstimatorHubertXYZ##Suffix; \
  typedef MEstimatorHubert<Scalar, pcl::PointXYZRGB, pcl::PointXYZRGB> MEstimatorHubertXYZRGB##Suffix; \
  typedef MEstimatorHubert<Scalar, pcl::PointNormal, pcl::PointNormal> MEstimatorHubertNormal##Suffix;

namespace icp {

template<typename Scalar, typename PointReference, typename PointSource>
class MEstimatorHubert : public MEstimator<Scalar, PointReference, PointSource> {
  public:
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;
    typedef typename pcl::PointCloud<PointSource> Pc;
    typedef typename Pc::Ptr PcPtr;
    typedef typename pcl::PointCloud<PointReference> Pr;
    typedef typename Pr::Ptr PrPtr;
    typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
    typedef typename Eigen::Matrix<Scalar, 4, 1> Vector3;

    VectorX weightsHuber(Scalar scale, VectorX rectified);

  protected:
    using MEstimator<Scalar, PointReference, PointSource>::weights_;
    using MEstimator<Scalar, PointReference, PointSource>::cloudModel_;
    using MEstimator<Scalar, PointReference, PointSource>::cloudReference_;
    using MEstimator<Scalar, PointReference, PointSource>::mad_;

  public:
    MEstimatorHubert() : MEstimator<Scalar, PointReference, PointSource>() {}
    virtual ~MEstimatorHubert() {}

    /**
     * @brief Computes the weights of the MEstimator from data
     *
     * This function should be called on the data point cloud before trying to
     * apply the MEstimator to initialize it according to the statistical
     * properties of the data.
     *
     * @param pc
     * Input point cloud
     */
    virtual void computeWeights();

};

DEFINE_MESTIMATOR_TYPES(float, );
DEFINE_MESTIMATOR_TYPES(float, f);

}  // namespace icp

#endif
