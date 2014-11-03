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

#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "eigentools.hpp"
#include "mestimator.hpp"

namespace icp {

template<typename Scalar>
class MaximumAbsoluteDeviation
{
  protected:
    Scalar scale_;
    Scalar mad_;
    Scalar noise_threshold_;

  public:
    typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, 1> Vector1;
    MaximumAbsoluteDeviation(float noise_threshold = 0.01)
      : noise_threshold_(noise_threshold)
    {}

    Vector1 operator() (const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> &v) {
      // Median centered residual error
      Vector1 r = (v - eigentools::median(v) *
                   Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Ones(v.rows(), 1)).cwiseAbs();


      // robust standard deviation (MAD)
      //scale = 1.4826 * (1+5/(n-p)) * median(r);
      mad_ = eigentools::median(r);
      scale_ =  1.4826 * mad_; 

      // If MAD is less that noise threshold
      if (scale_ < noise_threshold_) {
        // scale = 1.4826 * (1+5/(n-p)) * 1;
        scale_ = 1.4826;
      }

      return r;
    }

    Scalar getMad() const {
      return mad_;
    }
};

template<typename Scalar>
class MEstimatorHubert : public MEstimator<Scalar> {
  public:
    typedef pcl::PointCloud<pcl::PointXYZ> Pc;
    typedef typename Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

  protected:
    MatrixX weights_;
    // Robust standard deviation
    Scalar scale_x_, scale_y_, scale_z_;

  public:
    MEstimatorHubert() : MEstimator<Scalar>() {}
    virtual ~MEstimatorHubert() {}

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
    virtual void computeWeights(const Pc::Ptr pc);
    //{
    //  MaximumAbsoluteDeviation<float> mad;
    //  Eigen::MatrixXf m = pc->getMatrixXfMap().transpose(); 
    //  LOG(INFO) << "Rows: " << m.rows() << ", cols: " << m.cols();
    //  mad(m.col(0));
    //  LOG(INFO) << "MAD x = " << mad.getMad();
    //  mad(m.col(1));
    //  LOG(INFO) << "MAD y = " << mad.getMad();
    //  mad(m.col(2));
    //  LOG(INFO) << "MAD z = " << mad.getMad();

    //  Eigen::MatrixXf weights_(m.rows(), m.cols());
    //  LOG(FATAL) << "TODO";
    //};

    MatrixX getWeights() const {
      return weights_;
    }

};

}  // namespace icp

#endif
