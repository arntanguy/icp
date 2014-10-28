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
#include "eigentools.hpp"
#include "icp.hpp"
#include "errorPointToPoint.hpp"

namespace icp {

/**
   Creates a test fixture
**/
class IcpTest : public ::testing::Test {
  public:
    typedef typename Sophus::SE3Group<float>::Tangent Twist;

  protected:
    // Initialize the object for the test
    virtual void SetUp() {
      // Constructs a random point cloud
      pc_m_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                new pcl::PointCloud<pcl::PointXYZ>());
      for (int i = 0; i < 100; i++) {
        pc_m_->push_back(pcl::PointXYZ(10*rand(), 10*rand(), 10*rand()));
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
    = eigentools::createTransformationMatrix(0.0f,
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
  EXPECT_TRUE(r.registrationTwist.isApprox(transformationTwist, 10e-2))
      << "Expected:\n " << transformationTwist
      << "\nActual:\n " << r.registrationTwist
      << "\nTransformation:\n " << transformation;
}


/**
 * Runs ICP several time on the same data with same parameters, and checks
 * whether it always gives the same result
 * This is meant to try and prevent errors due to floating point approximations
 */
TEST_F(IcpTest, Repeatability) {
  Eigen::Matrix4f transformation
    = eigentools::createTransformationMatrix(0.f,
        0.05f,
        0.f,
        static_cast<float>(M_PI) / 200.f,
        static_cast<float>(M_PI) / 200.f,
        0.f);
  //Eigen::Matrix4f transformation = Eigen::MatrixXf::Identity(4,4);
    //= eigentools::createTransformationMatrix(0.0f,
    //    0.0f,
    //    0.0f,
    //    0.0f,
    //    0.0f,
    //    0.0f);

  //  Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_d
  (new pcl::PointCloud<pcl::PointXYZ>());
  // Generates a data point cloud to be matched against the model
  pcl::transformPointCloud(*pc_m_, *pc_d, transformation);

  icp_.setDataPointCloud(pc_d);

  icp_.run();
  icp::IcpResultsf result = icp_.getResults();
  icp::IcpResultsf newresult;
  const int NBTESTS = 100;
  for (int i = 0; i < NBTESTS; ++i)
  {
    icp_.run();
    newresult = icp_.getResults();
    EXPECT_TRUE(newresult.registrationTwist.isApprox(result.registrationTwist,
                10e-3))
        << "ICP repeatability failed on try " << i << "/" << NBTESTS
        << "Result 1: \n" << result.registrationTwist
        << "Result 2: \n" << newresult.registrationTwist;
  }
}

}  // namespace icp
