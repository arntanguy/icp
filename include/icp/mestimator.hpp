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
#include "maximum_absolute_deviation.hpp"

namespace icp {

/**
 * @brief Robust estimator based on Median Absolute Deviation
 */
template<typename Scalar, typename Point>
class MEstimator {
  public:
    typedef typename pcl::PointCloud<Point> Pc;
    typedef typename pcl::PointCloud<Point>::Ptr PcPtr;
    typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
    typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;

  protected:
    MatrixX weights_;
    PcPtr cloudModel_;
    PcPtr cloudReference_;
    MaximumAbsoluteDeviation<Scalar, Point> mad_;


  public:
    MEstimator() {}
    virtual ~MEstimator() {}

    void setModelCloud(const PcPtr &pc) {
      cloudModel_ = pc;
      mad_.setModelCloud(cloudModel_);
    }

    void setReferenceCloud(const PcPtr &ref, Eigen::Matrix<Scalar, 4, 4> T) {
      cloudReference_ = ref;
      mad_.setReferenceCloud(cloudReference_, T);
    }

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
    virtual void computeWeights() = 0;

    virtual MatrixX getWeights() const {
      return weights_;
    }

    void createWeightColoredCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr dstCloud) {
      //pcl::copyPointCloud(cloudReference_, dstCloud);
      dstCloud->clear();
      dstCloud->resize(cloudReference_->size());
      for (unsigned int i = 0; i < cloudReference_->size(); ++i) {
        const Point& p = (*cloudReference_)[i];
        pcl::PointXYZRGB pr;
        pr.x = p.x;
        pr.y = p.y;
        pr.z = p.z;
        pr.r = 255;
        pr.g = 0;
        pr.b = 255;
        dstCloud->push_back(pr);
      }
    }

};

}  // namespace icp

#endif
