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
#include "constraints.hpp"

#define RAND_SCALE 10

namespace test_icp {

using namespace icp;

#define DECLARE_TYPES(TypeParam) \
  typedef typename TypeParam::PointCloudPtr PointCloudPtr; \
  typedef typename TypeParam::PointCloud PointCloud; \
  typedef typename TypeParam::IcpMethod IcpMethod; \
  typedef typename TypeParam::IcpError IcpError; \
  typedef typename TypeParam::PointType PointType

/**
 * Encapsulate the templated types into one to pass to the unique template of
 * gtest fixtures
 **/
template <typename IcpMethod_, typename IcpError_, unsigned int DoF_, typename PointType_>
struct TypeDefinitions
{
  typedef IcpMethod_ IcpMethod;
  typedef IcpError_ IcpError;
  typedef PointType_ PointType;
  typedef typename pcl::PointCloud<PointType> PointCloud;
  typedef typename pcl::PointCloud<PointType>::Ptr PointCloudPtr;
  static constexpr unsigned int DoF = DoF_;
};
template<typename IcpMethod_, typename IcpError_, unsigned int DoF_, typename PointType_> constexpr unsigned int
TypeDefinitions<IcpMethod_, IcpError_, DoF_, PointType_>::DoF;

/**
   Creates a test fixture
**/
template<typename T>
class IcpCommonTest : public ::testing::Test {
  public:
    typedef std::list<T> List;
    static T shared_;
    T value_;

    DECLARE_TYPES(T);

  protected:
    // Initialize the object for the test
    virtual void SetUp() {
      //// Constructs a random point cloud
      pc_m_ = PointCloudPtr(new PointCloud());
      pc_s_ = PointCloudPtr(new PointCloud());
      for (int i = 0; i < 100; i++) {
        pc_m_->push_back(PointType(RAND_SCALE * rand(), RAND_SCALE * rand(), RAND_SCALE * rand()));
      }
      icp_.setInputReference(pc_m_);
    }
    // Destructor for the test
    virtual void TearDown() {
    }

    IcpMethod icp_;
    PointCloudPtr pc_m_;
    PointCloudPtr pc_s_;
};

using testing::Types;
// The list of types we want to test.
// XXX: missing point to plane
typedef Types < TypeDefinitions<IcpPointToPointHubert, ErrorPointToPointXYZ, 6, pcl::PointXYZ>,
        TypeDefinitions<IcpPointToPointHubertXYZRGB, ErrorPointToPointXYZRGB, 6, pcl::PointXYZRGB>,
        TypeDefinitions<IcpPointToPointHubertSim3, ErrorPointToPointXYZSim3, 7, pcl::PointXYZ>,
        TypeDefinitions<IcpPointToPointHubertXYZRGBSim3, ErrorPointToPointXYZRGBSim3, 7, pcl::PointXYZRGB>
        > Implementations;
TYPED_TEST_CASE(IcpCommonTest, Implementations);

/**
 * Tests whether ICP converges when the two point clouds are already aligned
 */
TYPED_TEST(IcpCommonTest, Identity) {
  DECLARE_TYPES(TypeParam);

  PointCloudPtr pc_d(new PointCloud());
  // Generates a data point cloud to be matched against the model
  pcl::transformPointCloud(*(this->pc_m_), *pc_d, Eigen::Matrix4f::Identity());

  this->icp_.setInputCurrent(pc_d);
  this->icp_.run();
  IcpResults r = this->icp_.getResults();
  EXPECT_TRUE(r.transformation.isApprox(Eigen::Matrix4f::Identity(), 10e-2))
      << "Expected:\n " << Eigen::MatrixXf::Identity(4, 4)
      << "\nActual:\n " << r.transformation
      << "\nTransformation:\n " << Eigen::Matrix4f::Identity();
  EXPECT_FLOAT_EQ(r.getFinalError(),
                  0.f) << "Final error for identity should be 0";
}

/**
 * @brief Tests ICP alignement with 2 points and only translation
 * They should match perfectly
 */
TYPED_TEST(IcpCommonTest, TwoPointsTranslate) {
  DECLARE_TYPES(TypeParam);

  IcpParameters param;
  param.max_iter = 1000;
  param.min_variation = 10e-9;
  this->icp_.setParameters(param);

  {
    PointCloudPtr pc_m = PointCloudPtr(
                           new PointCloud());
    PointType p1_r = PointType(1, 0, 0);
    PointType p2_r = PointType(1, 1, 0);
    pc_m->push_back(p1_r);
    pc_m->push_back(p2_r);

    PointCloudPtr pc_d = PointCloudPtr(
                           new PointCloud());
    pc_d->push_back(PointType(1, 0, 0.5));
    pc_d->push_back(PointType(1, 1, 0.5));

    this->icp_.setInputReference(pc_m);
    this->icp_.setInputCurrent(pc_d);
    this->icp_.run();

    icp::IcpResults result = this->icp_.getResults();
    const float error = result.getFinalError();
    //Twist finalTwist = result.registrationTwist;
    EXPECT_NEAR(0.f, error, 10e-3) <<
                                   "Unable to perfectly align two translated points!";

    PointCloud registeredPointCloud;
    pcl::transformPointCloud(*pc_d, registeredPointCloud, result.transformation);
    PointType p1 = registeredPointCloud.at(0);
    PointType p2 = registeredPointCloud.at(1);
    EXPECT_TRUE(pcltools::isApprox(p1, p1_r,
                                   10e-3)) << "Tranformed and reference point 1 do not match!";
    EXPECT_TRUE(pcltools::isApprox(p2, p2_r,
                                   10e-3)) << "Tranformed and reference point 2 do not match!";
  }

  {
    PointCloudPtr pc_m = PointCloudPtr(new PointCloud());
    PointType p1_r = PointType(1, 0, 0);
    PointType p2_r = PointType(1, 1, 0);
    pc_m->push_back(p1_r);
    pc_m->push_back(p2_r);

    PointCloudPtr pc_d = PointCloudPtr(new PointCloud());
    pc_d->push_back(PointType(1, 0.2, 0.5));
    pc_d->push_back(PointType(1, 1.2, 0.5));

    this->icp_.setInputReference(pc_m);
    this->icp_.setInputCurrent(pc_d);
    this->icp_.run();

    IcpResults result = this->icp_.getResults();
    const float error = result.getFinalError();
    //Twist finalTwist = result.registrationTwist;
    EXPECT_NEAR(0.f, error, 10e-4) <<
                                   "Unable to perfectly align two translated points!";

    PointCloud registeredPointCloud;
    pcl::transformPointCloud(*pc_d, registeredPointCloud, result.transformation);
    PointType p1 = registeredPointCloud.at(0);
    PointType p2 = registeredPointCloud.at(1);
    EXPECT_TRUE(pcltools::isApprox(p1, p1_r,
                                   10e-2)) << "Tranformed and reference point 1 do not match!";
    EXPECT_TRUE(pcltools::isApprox(p2, p2_r,
                                   10e-2)) << "Tranformed and reference point 2 do not match!";
  }
}


/**
 * @brief Tests ICP alignement with 2 points and only rotation
 * They should match perfectly
 */
TYPED_TEST(IcpCommonTest, TwoPointsRotate) {
  DECLARE_TYPES(TypeParam);

  IcpParameters param;
  param.max_iter = 1000;
  param.min_variation = 10e-9;
  this->icp_.setParameters(param);

  PointCloudPtr pc_m(new PointCloud());
  PointCloudPtr pc_s(new PointCloud());
  PointType p1_r = PointType(1, 0, 0);
  PointType p2_r = PointType(1, 1, 0);
  pc_m->push_back(p1_r);
  pc_m->push_back(p2_r);

  pc_s->push_back(PointType(1, 0, 0));
  pc_s->push_back(PointType(1, 1, 0.5));

  this->icp_.setInputReference(pc_m);
  this->icp_.setInputCurrent(pc_s);
  this->icp_.run();

  IcpResults result = this->icp_.getResults();
  const float error = result.getFinalError();
  EXPECT_NEAR(error, 0.f, 10e-2) << "Unable to perfectly align two rotated points!";

  PointCloud registeredPointCloud;
  pcl::transformPointCloud(*pc_s, registeredPointCloud, result.transformation);
  PointType p1 = registeredPointCloud.at(0);
  PointType p2 = registeredPointCloud.at(1);
  EXPECT_TRUE(pcltools::isApprox(p1, p1_r, 10e-2))
      << "Tranformed and reference point 1 do not match!\n"
      << "Expected: " << p1_r
      << "\nActual: " << p1;
  EXPECT_TRUE(pcltools::isApprox(p2, p2_r, 10e-2))
      << "Tranformed and reference point 2 do not match!\n"
      << "Expected: " << p2_r
      << "\nActual: " << p2;
}

/**
 * @brief Tests ICP alignement with 2 points translated and rotated
 * They should match perfectly
 */
TYPED_TEST(IcpCommonTest, TwoPointsTranslateAndRotate) {
  DECLARE_TYPES(TypeParam);

  IcpParameters param;
  param.max_iter = 1000;
  param.min_variation = 10e-9;
  this->icp_.setParameters(param);

  {
    PointCloudPtr pc_m = PointCloudPtr(new PointCloud());
    PointType p1_r = PointType(1, 0, 0);
    PointType p2_r = PointType(1, 1, 0);
    pc_m->push_back(p1_r);
    pc_m->push_back(p2_r);

    PointCloudPtr pc_d = PointCloudPtr(new PointCloud());
    pc_d->push_back(PointType(1.2, 0, 0.2));
    pc_d->push_back(PointType(1.2, 1, 0.7));

    this->icp_.setInputReference(pc_m);
    this->icp_.setInputCurrent(pc_d);
    this->icp_.run();

    icp::IcpResults result = this->icp_.getResults();
    const float error = result.getFinalError();
    //Twist finalTwist = result.registrationTwist;
    EXPECT_NEAR(0.f, error, 10e-2) <<
                                   "Unable to perfectly align two rotated points!";

    PointCloud registeredPointCloud;
    transformPointCloud(*pc_d, registeredPointCloud, result.transformation);
    PointType p1 = registeredPointCloud.at(0);
    PointType p2 = registeredPointCloud.at(1);
    EXPECT_TRUE(pcltools::isApprox(p1, p1_r,
                                   10e-2)) << "Tranformed and reference point 1 do not match!";
    EXPECT_TRUE(pcltools::isApprox(p2, p2_r,
                                   10e-2)) << "Tranformed and reference point 2 do not match!";
  }
}

/**
 * Runs ICP several time on the same data with same parameters, and checks
 * whether it always gives the same result
 * This is meant to try and prevent errors due to floating point approximations
 */
TYPED_TEST(IcpCommonTest, Repeatability) {
  DECLARE_TYPES(TypeParam);

  Eigen::Matrix4f transformation
    = eigentools::createTransformationMatrix(0.f,
        0.05f,
        0.f,
        static_cast<float>(M_PI) / 200.f,
        static_cast<float>(M_PI) / 200.f,
        0.f);
  //  Executing the transformation
  PointCloudPtr pc_d (new PointCloud());
  // Generates a data point cloud to be matched against the model
  pcl::transformPointCloud(*this->pc_m_, *pc_d, transformation);

  this->icp_.setInputCurrent(pc_d);

  this->icp_.run();
  icp::IcpResults result = this->icp_.getResults();
  icp::IcpResults newresult;
  const int NBTESTS = 20;
  for (int i = 0; i < NBTESTS; ++i)
  {
    this->icp_.run();
    newresult = this->icp_.getResults();
    EXPECT_TRUE(newresult.transformation.isApprox(result.transformation,
                10e-3))
        << "ICP repeatability failed on try " << i << "/" << NBTESTS
        << "Result 1: \n" << result.transformation
        << "Result 2: \n" << newresult.transformation;
  }
}


//TYPED_TEST(IcpCommonTest, TranlationConstraintEnforcement) {
//  DECLARE_TYPES(TypeParam);
//
//  const float translationFactor = 1;
//  Eigen::Matrix4f translateX = Eigen::Matrix4f::Identity();
//  translateX(0, 3) = translationFactor;
//  translateX(1, 3) = translationFactor;
//  translateX(2, 3) = translationFactor;
//
//  FixTranslationConstraint tc;
//  Constraints<float, TypeParam::DoF> c;
//
//
//  {
//    tc.setFixedAxes(true, false, false);
//    c.setTranslationConstraint(tc);
//    IcpError err;
//    err.setConstraints(c);
//
//    this->icp_.setError(err);
//    PointCloudPtr pc_translated(new PointCloud());
//    pcl::transformPointCloud(*this->pc_m_, *pc_translated, translateX);
//    this->icp_.setInputCurrent(pc_translated);
//    this->icp_.run();
//    IcpResults r = this->icp_.getResults();
//    EXPECT_EQ(r.transformation(0, 3), 0) <<
//                                         "Error, translation on X wasn't 0 despite a fixed axis contraint on X. Transformation is:" << r.transformation;
//  }
//  {
//    tc.setFixedAxes(false, true, false);
//    c.setTranslationConstraint(tc);
//    IcpError err;
//    err.setConstraints(c);
//
//    this->icp_.setError(err);
//    PointCloudPtr pc_translated(new PointCloud());
//    pcl::transformPointCloud(*this->pc_m_, *pc_translated, translateX);
//    this->icp_.setInputCurrent(pc_translated);
//    this->icp_.run();
//    IcpResults r = this->icp_.getResults();
//    EXPECT_EQ(r.transformation(1, 3), 0) <<
//                                         "Error, translation on Y wasn't 1 despite a fixed axis contraint on X. Transformation is:" << r.transformation;
//  }
//  {
//    tc.setFixedAxes(false, false, true);
//    c.setTranslationConstraint(tc);
//    IcpError err;
//    err.setConstraints(c);
//
//    this->icp_.setError(err);
//    PointCloudPtr pc_translated(new PointCloud());
//    pcl::transformPointCloud(*this->pc_m_, *pc_translated, translateX);
//    this->icp_.setInputCurrent(pc_translated);
//    this->icp_.run();
//    IcpResults r = this->icp_.getResults();
//    EXPECT_EQ(r.transformation(2, 3), 0) <<
//                                         "Error, translation on Z wasn't 0 despite a fixed axis contraint on X. Transformation is:" << r.transformation;
//  }
//  {
//    tc.setFixedAxes(true, false, true);
//    c.setTranslationConstraint(tc);
//    IcpError err;
//    err.setConstraints(c);
//
//    this->icp_.setError(err);
//    PointCloudPtr pc_translated(new PointCloud());
//    pcl::transformPointCloud(*this->pc_m_, *pc_translated, translateX);
//    this->icp_.setInputCurrent(pc_translated);
//    this->icp_.run();
//    IcpResults r = this->icp_.getResults();
//    EXPECT_EQ(r.transformation(0, 3), 0) <<
//                                         "Error, translation on X wasn't 0 despite a fixed axis contraint on X. Transformation is:" << r.transformation;
//    EXPECT_EQ(r.transformation(2, 3), 0) <<
//                                         "Error, translation on Z wasn't 0 despite a fixed axis contraint on X. Transformation is:" << r.transformation;
//  }
//  {
//    tc.setFixedAxes(false, true, true);
//    c.setTranslationConstraint(tc);
//    IcpError err;
//    err.setConstraints(c);
//
//    this->icp_.setError(err);
//    PointCloudPtr pc_translated(new PointCloud());
//    pcl::transformPointCloud(*this->pc_m_, *pc_translated, translateX);
//    this->icp_.setInputCurrent(pc_translated);
//    this->icp_.run();
//    IcpResults r = this->icp_.getResults();
//    EXPECT_EQ(r.transformation(1, 3), 0) <<
//                                         "Error, translation on Y wasn't 0 despite a fixed axis contraint on X. Transformation is:" << r.transformation;
//    EXPECT_EQ(r.transformation(2, 3), 0) <<
//                                         "Error, translation on Z wasn't 0 despite a fixed axis contraint on X. Transformation is:" << r.transformation;
//  }
//  {
//    tc.setFixedAxes(true, true, true);
//    c.setTranslationConstraint(tc);
//    IcpError err;
//    err.setConstraints(c);
//
//    this->icp_.setError(err);
//    PointCloudPtr pc_translated(new PointCloud());
//    pcl::transformPointCloud(*this->pc_m_, *pc_translated, translateX);
//    this->icp_.setInputCurrent(pc_translated);
//    this->icp_.run();
//    IcpResults r = this->icp_.getResults();
//    EXPECT_EQ(r.transformation(0, 3), 0) <<
//                                         "Error, translation on Y wasn't 0 despite a fixed axis contraint on X. Transformation is:" << r.transformation;
//    EXPECT_EQ(r.transformation(1, 3), 0) <<
//                                         "Error, translation on Y wasn't 0 despite a fixed axis contraint on X. Transformation is:" << r.transformation;
//    EXPECT_EQ(r.transformation(2, 3), 0) <<
//                                         "Error, translation on Y wasn't 0 despite a fixed axis contraint on X. Transformation is:" << r.transformation;
//  }
//
//}

}  //  namespace test_icp
