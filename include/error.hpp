//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#ifndef   ERROR_HPP
#define   ERROR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * @brief Abstract interface with all the functions that are needed to compute
 * everything related to the ICP's error
 */
template<typename Dtype>
class Error {
  public:
    typedef pcl::PointCloud<pcl::PointXYZ> Pc;
    typedef Eigen::Matrix<Dtype, Eigen::Dynamic, 1> ErrorVector;
    typedef Eigen::Matrix<Dtype, Eigen::Dynamic, 6> JacobianMatrix;

  protected:
    Pc::Ptr pc_m_;
    Pc::Ptr pc_d_;

    //! Vector containing the error for each point
    ErrorVector errorVector_;
    //! Corresponding Jacobian
    JacobianMatrix J_;

  public:
    virtual void computeError() = 0;
    virtual void computeJacobian() = 0;

    virtual JacobianMatrix getJacobian() const {
      return J_;
    }
    virtual ErrorVector getErrorVector() const {
      return errorVector_;
    }

    virtual void setModelPointCloud(const Pc::Ptr &model) {
      pc_m_ = model;
    }
    virtual void setDataPointCloud(const Pc::Ptr &data) {
      pc_d_ = data;
    }
};

#endif
