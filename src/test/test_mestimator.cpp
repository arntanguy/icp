//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include "mestimator_hubert.hpp"

namespace icp {

/**
   Creates a test fixture
**/
class MEstimatorHubertTest : public ::testing::Test {
  protected:
    // Initialize the object for the test
    virtual void SetUp() {
      // Constructs a random point cloud
      pc_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
              new pcl::PointCloud<pcl::PointXYZ>());
      for (int i = 0; i < 100; i++) {
        pc_->push_back(pcl::PointXYZ(rand(), rand(), rand()));
      }
    }
    // Destructor for the test
    virtual void TearDown() {
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_;
};

TEST_F(MEstimatorHubertTest, MAD) {
  Eigen::VectorXf v(7); 
  v << 1.f, 1.f, 2.f, 2.f, 4.f, 6.f, 9.f;

  const float expected_mad = 1.f;

  MaximumAbsoluteDeviation<float> mad;
  mad(v);
  float actual_mad = mad.getMad();

  EXPECT_FLOAT_EQ(expected_mad, actual_mad);
}

TEST_F(MEstimatorHubertTest, TestComputeWeights) {
  MEstimatorHubert<float> m;
  m.computeWeights(pc_);
  Eigen::MatrixXf w = m.getWeights();
}

}  // namespace icp
