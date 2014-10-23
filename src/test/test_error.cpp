//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "eigen_tools.hpp"
#include "icp.hpp"
#include "errorPointToPoint.hpp"

namespace icp {

class TestErrorPointToPoint : public ::testing::Test
{
  protected:
    virtual void SetUp() {
      pc1_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
               new pcl::PointCloud<pcl::PointXYZ>());
      pc2_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
               new pcl::PointCloud<pcl::PointXYZ>());
      const int pointcloud_size = 100;
      err_expected_.resize(3 * pointcloud_size, 1);
      for (unsigned int i = 0; i < pointcloud_size; ++i)
      {
        pc1_->push_back(pcl::PointXYZ(i, 2.f * i, 3.f * i));
        pc2_->push_back(pcl::PointXYZ(-2.f * i, 3.f * i, i));
        err_expected_[i * 3] = -3.f * i;
        err_expected_[i * 3 + 1] =  i; err_expected_[i * 3 + 2] = -2.f * i;
      }
    }

    virtual void TearDown() {
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc1_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_;
    ErrorPointToPoint<float> err_;
    Eigen::Matrix<float, Eigen::Dynamic, 1> err_expected_;
};


TEST_F(TestErrorPointToPoint, IdentityErrorVector) {
  err_.setModelPointCloud(pc1_);
  err_.setDataPointCloud(pc1_);
  err_.computeError();

  Eigen::MatrixXf ev = err_.getErrorVector();
  Eigen::MatrixXf ev_expected = Eigen::MatrixXf::Zero(ev.rows(), 1);

  EXPECT_EQ(ev_expected, ev) <<
                             "Error vector should have been null as the two point clouds are identical!";
}

TEST_F(TestErrorPointToPoint, RealErrorVector) {
  err_.setModelPointCloud(pc1_);
  err_.setDataPointCloud(pc2_);
  err_.computeError();

  Eigen::MatrixXf ev = err_.getErrorVector();
  //LOG(INFO) << ev - err_expected_;

  EXPECT_EQ(err_expected_,
            ev) << "Error vector is wrong! (Or equality comparison isn't good enough)";

}


}  // namespace icp
