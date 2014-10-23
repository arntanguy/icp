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
#include "eigen_tools.hpp"
#include "icp.hpp"
#include "errorPointToPoint.hpp"

namespace icp {

/**
   Creates a test fixture
**/
class IcpTest : public ::testing::Test {
 protected:
  // Initialize the object for the test
  virtual void SetUp() {
    // Constructs a random point cloud
    pc_m_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < 100; i++) {
      pc_m_->push_back(pcl::PointXYZ(rand(), rand(), rand()));
      pc_m_->push_back(pcl::PointXYZ(rand(), rand(), rand()));
      pc_m_->push_back(pcl::PointXYZ(rand(), rand(), rand()));
      pc_m_->push_back(pcl::PointXYZ(rand(), rand(), rand()));
    }

    icp_.setModelPointCloud(pc_m_);
  }
  // Destructor for the test
  virtual void TearDown() {
  }

  // XXX: templates
  Icp<float, ErrorPointToPoint<float>, float> icp_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_m_;
};

/**
 * Tests whether ICP converges when the two point clouds are already aligned
 */
TEST_F(IcpTest, Identity) {
  // Identity transformation
  Eigen::Matrix4f transformation
      = createTransformationMatrix(0.0f,
                                   0.0f,
                                   0.0f,
                                   0.0f,
                                   0.0f,
                                   0.0f);
  Sophus::SE3Group<float> transformationSO3(transformation);
  Sophus::SE3Group<float>::Tangent transformationTwist
      = transformationSO3.log();

  //  Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_d
      (new pcl::PointCloud<pcl::PointXYZ>());
  // Generates a data point cloud to be matched against the model
  pcl::transformPointCloud(*pc_m_, *pc_d, transformation);

  icp_.setDataPointCloud(pc_d);
  icp_.run();
  IcpResults_<float> r = icp_.getResults();
  EXPECT_EQ(transformationTwist, r.registrationTwist)
      << "Expected:\n " << transformationTwist
      << "\nActual:\n " << r.registrationTwist;
}

}  // namespace icp
