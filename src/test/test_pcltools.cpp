//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include "pcltools.hpp"

namespace test_icp {

/**
   Creates a test fixture
**/
class PclToolsTest : public ::testing::Test {
    typedef typename pcl::PointCloud<pcl::PointXYZ> Pc;
  protected:
    // Initialize the object for the test
    virtual void SetUp() {
      // Constructs a random point cloud
      pc1_ = Pc::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
      pc2_ = Pc::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
      pc_sub = Pc::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
      for (int i = 0; i < 100; i++) {
        pc1_->push_back(pcl::PointXYZ(i, 1 * i, 3 * i));
        pc2_->push_back(pcl::PointXYZ(3 * i, 2 * i, i));
        pc_sub->push_back(pcl::PointXYZ(2 * i, i, -2 * i));
      }


      p1_ = pcl::PointXYZ(-1.3, 1, 0.9);
      p2_ = pcl::PointXYZ(1.2, 1.1, 0.2);
      p_sub = pcl::PointXYZ(2.5, 0.1, -0.7);
    }
    // Destructor for the test
    virtual void TearDown() {
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc1_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc2_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_sub;
    pcl::PointXYZ p1_;
    pcl::PointXYZ p2_;
    pcl::PointXYZ p_sub;
};

TEST_F(PclToolsTest, EqualityPoints) {
  EXPECT_TRUE(pcltools::isApprox(p1_, p1_)) << "Points should be equal!";
}

TEST_F(PclToolsTest, SubstractPoints) {
  // p2-p1
  pcl::PointXYZ result = pcltools::substract(p1_, p2_);

  EXPECT_TRUE(pcltools::isApprox(p_sub, result)) << "Expected: " << p_sub << ", Actual: " << result;
}

TEST_F(PclToolsTest, SubstractPointClouds) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr result(
    pcltools::substractPointcloud<pcl::PointXYZ>(pc1_, pc2_));

  for (unsigned int i = 0; i < pc_sub->size(); i++) {
    EXPECT_TRUE(pcltools::isApprox((*pc_sub)[i], (*result)[i])) << "Expected: " <<
        (*pc_sub)[i] << ", Actual: " << (*result)[i];
  }
}

TEST_F(PclToolsTest, SubPointCloud) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr src = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  src->push_back(pcl::PointXYZ(0, 0, 0));
  src->push_back(pcl::PointXYZ(0, 0, 1));
  src->push_back(pcl::PointXYZ(0, 1, 0));
  src->push_back(pcl::PointXYZ(0, 1, 1));

  pcl::PointCloud<pcl::PointXYZ>::Ptr dst = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
  pcltools::subPointCloud<pcl::PointXYZ>(src, { 0, 2, 3 }, dst);

  EXPECT_EQ(dst->size(), 3) << "Size of subsampled pointcloud is wrong!";

  pcltools::subPointCloud<pcl::PointXYZ>(src, { 1, 3 }, dst);
  EXPECT_EQ(dst->size(), 2) << "Size of subsampled pointcloud is wrong!";
}

}  // namespace test_icp
