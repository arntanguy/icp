//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#ifndef   MESTIMATOR_HPP
#define   MESTIMATOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace icp {

/**
 * @brief Robust estimator based on Median Absolute Deviation 
 */
template<typename Scalar>
class MEstimator {
 public:
    typedef pcl::PointCloud<pcl::PointXYZ> Pc;
    typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

  protected:
    MatrixX weights_;

  public:
    MEstimator() {}
    virtual ~MEstimator() {}

    /**
     * @brief Computes the weights of the MEstimator from data
     * This function should be called on the data point cloud before trying to
     * apply the MEstimator to initialize it according to the statistical
     * properties of the data.
     * If not called first, apply
     *
     * @param pc
     * Input point cloud
     */
    virtual void computeWeights(const Pc::Ptr pc) = 0;

    virtual MatrixX getWeights() const {
      return weights_;
    }

};

}  // namespace icp

#endif
