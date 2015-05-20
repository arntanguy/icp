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
template<typename Scalar, typename PointReference, typename PointSource>
class MEstimator {
  public:
    typedef typename pcl::PointCloud<PointSource> Pc;
    typedef typename pcl::PointCloud<PointReference> Pr;
    typedef typename Pc::Ptr PcPtr;
    typedef typename Pr::Ptr PrPtr;
    typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;
    typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> VectorX;

  protected:
    MatrixX weights_;
    PcPtr cloudModel_;
    PrPtr cloudReference_;
    MaximumAbsoluteDeviation<Scalar, PointReference, PointSource> mad_;


  public:
    MEstimator() : cloudModel_(new Pc()), cloudReference_(new Pr()) {}
    virtual ~MEstimator() {}

    /**
     * @brief Sets the model cloud, compute it's MAD
     *
     * @param pc
     */
    void setModelCloud(const PcPtr &pc) {
      cloudModel_ = pc;
      mad_.setModelCloud(cloudModel_);
    }

    /**
     * @brief Sets the reference cloud, compute the mestimator's MAD
     *
     * @param ref
     */
    void setReferenceCloud(const PrPtr &pcr) {
      cloudReference_ = pcr;
      mad_.setReferenceCloud(cloudReference_);
    }

    /**
     * @brief Sets the reference cloud, compute the mestimator's MAD
     * with respect to a median offseted by T, and using the model's
     * maximum residual error as a scaling factor.
     * This can be used to statistically restrain the pointcloud to
     * the interresting points, provided that a good initial estimate is known.
     *
     * @param ref
     */
    void setReferenceCloud(const PrPtr &pcr, Eigen::Matrix<Scalar, 4, 4> T) {
      cloudReference_ = pcr;
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

    /**
     * @brief Creates a pointcloud colored with the weights of the mestimator.
     * The color components are as follow:
     * - red: 0-255, weight along x axis
     * - green: 0-255, weight along y axis
     * - blue: 0-255, weight along z axis
     *
     * @param dstCloud
     */
    void createWeightColoredCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr dstCloud) {
      //pcl::copyPointCloud(cloudReference_, dstCloud);
      if (cloudReference_->size() > 0) {
        dstCloud->clear();
        dstCloud->resize(cloudReference_->size());
        for (unsigned int i = 0; i < cloudReference_->size(); ++i) {
          const PointReference &p = (*cloudReference_)[i];
          pcl::PointXYZRGB pr;
          pr.x = p.x;
          pr.y = p.y;
          pr.z = p.z;
          // pack r/g/b into rgb
          uint8_t r = (uint8_t)(255 * weights_(i, 0));
          uint8_t g = (uint8_t)(255 * weights_(i, 1));
          uint8_t b = (uint8_t)(255 * weights_(i, 2));
          uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
          pr.rgb = *reinterpret_cast<float *>(&rgb);

          dstCloud->push_back(pr);
        }
      }
    }

    /**
     * @brief Creates a pointcloud colored with the weights of the mestimator.
     * The color components are as follow:
     * - red = green = blue: 0-255, combined weight for x, y, z axis
     *
     * @param dstCloud
     */
    void createWeightColoredCloudIntensity(pcl::PointCloud<pcl::PointXYZRGB>::Ptr dstCloud) {
      //pcl::copyPointCloud(cloudReference_, dstCloud);
      dstCloud->clear();
      dstCloud->resize(cloudReference_->size());
      for (unsigned int i = 0; i < cloudReference_->size(); ++i) {
        const PointReference &p = (*cloudReference_)[i];
        pcl::PointXYZRGB pr;
        pr.x = p.x;
        pr.y = p.y;
        pr.z = p.z;
        // pack r/g/b into rgb
        uint8_t weight = (uint8_t)(255 * weights_(i, 0) * weights_(i, 1) * weights_(i, 2));
        uint8_t r = (uint8_t)(weight);
        uint8_t g = (uint8_t)(weight);
        uint8_t b = (uint8_t)(weight);
        uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        pr.rgb = *reinterpret_cast<float *>(&rgb);

        dstCloud->push_back(pr);
      }
    }

};

}  // namespace icp

#endif
