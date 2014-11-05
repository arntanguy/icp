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
#include "error_point_to_point.hpp"
#include "linear_algebra.hpp"
#include "mestimator_hubert.hpp"

namespace test_icp {

using namespace icp;

/**
   Creates a test fixture
**/
class IcpTest : public ::testing::Test {
  public:
    typedef typename Eigen::Matrix<float, 6, 1> Twist;

  protected:
    // Initialize the object for the test
    virtual void SetUp() {
      // Constructs a random point cloud
      pc_m_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
                new pcl::PointCloud<pcl::PointXYZ>());
      for (int i = 0; i < 100; i++) {
        pc_m_->push_back(pcl::PointXYZ(10 * rand(), 10 * rand(), 10 * rand()));
      }

      icp_.setModelPointCloud(pc_m_);

      identityTwist_ = Eigen::Matrix<float, 6, 1>::Zero(6, 1);
      identityTransform_ = la::expSE3(identityTwist_);
    }
    // Destructor for the test
    virtual void TearDown() {
    }

    // XXX: templates
    Icp<float, ErrorPointToPoint<float>, MEstimatorHubert<float>> icp_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_m_;

    // Identity transformation
    Twist identityTwist_;
    Eigen::Matrix4f identityTransform_;
};


/**
 * Tests whether ICP converges when the two point clouds are already aligned
 */
TEST_F(IcpTest, Identity) {
  //  Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_d
  (new pcl::PointCloud<pcl::PointXYZ>());
  // Generates a data point cloud to be matched against the model
  pcl::transformPointCloud(*pc_m_, *pc_d, identityTransform_);

  icp_.setDataPointCloud(pc_d);
  icp_.run();
  IcpResults_<float> r = icp_.getResults();
  EXPECT_TRUE(r.transformation.isApprox(Eigen::MatrixXf::Identity(4,4), 10e-2))
      << "Expected:\n " << Eigen::MatrixXf::Identity(4,4)
      << "\nActual:\n " << r.transformation
      << "\nTransformation:\n " << identityTransform_;
  EXPECT_FLOAT_EQ(r.getFinalError(), 0.f) << "Final error for identity should be 0";
}

/**
 * @brief Tests ICP alignement with 2 points and only translation
 * They should match perfectly
 */
TEST_F(IcpTest, TwoPointsTranslate) {
  icp::IcpParametersf param;
  param.max_iter = 1000;
  param.min_variation = 10e-9;
  icp_.setParameters(param);

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_m = pcl::PointCloud<pcl::PointXYZ>::Ptr(
              new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ p1_r = pcl::PointXYZ(1, 0, 0);
    pcl::PointXYZ p2_r = pcl::PointXYZ(1, 1, 0);
    pc_m->push_back(p1_r);
    pc_m->push_back(p2_r);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_d = pcl::PointCloud<pcl::PointXYZ>::Ptr(
              new pcl::PointCloud<pcl::PointXYZ>());
    pc_d->push_back(pcl::PointXYZ(1, 0, 0.5));
    pc_d->push_back(pcl::PointXYZ(1, 1, 0.5));

    icp_.setModelPointCloud(pc_m);
    icp_.setDataPointCloud(pc_d);
    icp_.run();

    icp::IcpResultsf result = icp_.getResults();
    const float error = result.getFinalError();
    //Twist finalTwist = result.registrationTwist;
    EXPECT_NEAR(0.f, error, 10e-3) << "Unable to perfectly align two translated points!";

    pcl::PointXYZ p1 = result.registeredPointCloud->at(0);
    pcl::PointXYZ p2 = result.registeredPointCloud->at(1);
    EXPECT_TRUE(pcltools::isApprox(p1, p1_r, 10e-3)) << "Tranformed and reference point 1 do not match!";
    EXPECT_TRUE(pcltools::isApprox(p2, p2_r, 10e-3)) << "Tranformed and reference point 2 do not match!";
  }
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_m = pcl::PointCloud<pcl::PointXYZ>::Ptr(
              new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ p1_r = pcl::PointXYZ(1, 0, 0);
    pcl::PointXYZ p2_r = pcl::PointXYZ(1, 1, 0);
    pc_m->push_back(p1_r);
    pc_m->push_back(p2_r);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_d = pcl::PointCloud<pcl::PointXYZ>::Ptr(
              new pcl::PointCloud<pcl::PointXYZ>());
    pc_d->push_back(pcl::PointXYZ(1, 0.2, 0.5));
    pc_d->push_back(pcl::PointXYZ(1, 1.2, 0.5));

    icp_.setModelPointCloud(pc_m);
    icp_.setDataPointCloud(pc_d);
    icp_.run();

    icp::IcpResultsf result = icp_.getResults();
    const float error = result.getFinalError();
    //Twist finalTwist = result.registrationTwist;
    EXPECT_NEAR(0.f, error, 10e-4) << "Unable to perfectly align two translated points!";

    pcl::PointXYZ p1 = result.registeredPointCloud->at(0);
    pcl::PointXYZ p2 = result.registeredPointCloud->at(1);
    EXPECT_TRUE(pcltools::isApprox(p1, p1_r, 10e-2)) << "Tranformed and reference point 1 do not match!";
    EXPECT_TRUE(pcltools::isApprox(p2, p2_r, 10e-2)) << "Tranformed and reference point 2 do not match!";
  }
}

/**
 * @brief Tests ICP alignement with 2 points and only rotation
 * They should match perfectly
 */
TEST_F(IcpTest, TwoPointsRotate) {
  icp::IcpParametersf param;
  param.max_iter = 1000;
  param.min_variation = 10e-9;
  icp_.setParameters(param);

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_m = pcl::PointCloud<pcl::PointXYZ>::Ptr(
              new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ p1_r = pcl::PointXYZ(1, 0, 0);
    pcl::PointXYZ p2_r = pcl::PointXYZ(1, 1, 0);
    pc_m->push_back(p1_r);
    pc_m->push_back(p2_r);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_d = pcl::PointCloud<pcl::PointXYZ>::Ptr(
              new pcl::PointCloud<pcl::PointXYZ>());
    pc_d->push_back(pcl::PointXYZ(1, 0, 0));
    pc_d->push_back(pcl::PointXYZ(1, 1, 0.5));

    icp_.setModelPointCloud(pc_m);
    icp_.setDataPointCloud(pc_d);
    icp_.run();

    icp::IcpResultsf result = icp_.getResults();
    const float error = result.getFinalError();
    //Twist finalTwist = result.registrationTwist;
    EXPECT_NEAR(error, 0.f, 10e-2) << "Unable to perfectly align two rotated points!";

    pcl::PointXYZ p1 = result.registeredPointCloud->at(0);
    pcl::PointXYZ p2 = result.registeredPointCloud->at(1);
    EXPECT_TRUE(pcltools::isApprox(p1, p1_r, 10e-2)) << "Tranformed and reference point 1 do not match!";
    EXPECT_TRUE(pcltools::isApprox(p2, p2_r, 10e-2)) << "Tranformed and reference point 2 do not match!";
  }
}

/**
 * @brief Tests ICP alignement with 2 points translated and rotated
 * They should match perfectly
 */
TEST_F(IcpTest, TwoPointsTranslateAndRotate) {
  icp::IcpParametersf param;
  param.max_iter = 1000;
  param.min_variation = 10e-9;
  icp_.setParameters(param);

  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_m = pcl::PointCloud<pcl::PointXYZ>::Ptr(
              new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ p1_r = pcl::PointXYZ(1, 0, 0);
    pcl::PointXYZ p2_r = pcl::PointXYZ(1, 1, 0);
    pc_m->push_back(p1_r);
    pc_m->push_back(p2_r);

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_d = pcl::PointCloud<pcl::PointXYZ>::Ptr(
              new pcl::PointCloud<pcl::PointXYZ>());
    pc_d->push_back(pcl::PointXYZ(1.2, 0, 0.2));
    pc_d->push_back(pcl::PointXYZ(1.2, 1, 0.7));

    icp_.setModelPointCloud(pc_m);
    icp_.setDataPointCloud(pc_d);
    icp_.run();

    icp::IcpResultsf result = icp_.getResults();
    const float error = result.getFinalError();
    //Twist finalTwist = result.registrationTwist;
    EXPECT_NEAR(0.f, error, 10e-2) << "Unable to perfectly align two rotated points!";

    pcl::PointXYZ p1 = result.registeredPointCloud->at(0);
    pcl::PointXYZ p2 = result.registeredPointCloud->at(1);
    EXPECT_TRUE(pcltools::isApprox(p1, p1_r, 10e-2)) << "Tranformed and reference point 1 do not match!";
    EXPECT_TRUE(pcltools::isApprox(p2, p2_r, 10e-2)) << "Tranformed and reference point 2 do not match!";
  }
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
  //  Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_d
  (new pcl::PointCloud<pcl::PointXYZ>());
  // Generates a data point cloud to be matched against the model
  pcl::transformPointCloud(*pc_m_, *pc_d, transformation);

  icp_.setDataPointCloud(pc_d);

  icp_.run();
  icp::IcpResultsf result = icp_.getResults();
  icp::IcpResultsf newresult;
  const int NBTESTS = 20;
  for (int i = 0; i < NBTESTS; ++i)
  {
    icp_.run();
    newresult = icp_.getResults();
    EXPECT_TRUE(newresult.transformation.isApprox(result.transformation,
                10e-3))
        << "ICP repeatability failed on try " << i << "/" << NBTESTS
        << "Result 1: \n" << result.transformation
        << "Result 2: \n" << newresult.transformation;
  }
}

}  // namespace test_icp
