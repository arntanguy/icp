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
#include "linear_algebra.hpp"

namespace test_icp {

using namespace icp;

/**
 * Encapsulate the templated types into one to pass to the unique template of
 * gtest fixtures
 **/
template <typename IcpMethod_, typename PointType_>
struct TypeDefinitions
{
  typedef IcpMethod_ IcpMethod;
  typedef PointType_ PointType;
  typedef typename pcl::PointCloud<PointType> PointCloud;
  typedef typename pcl::PointCloud<PointType>::Ptr PointCloudPtr;
};

/**
   Creates a test fixture
**/
template<typename T>
class IcpCommonTest : public ::testing::Test {
  public:
    typedef std::list<T> List;
    static T shared_;
    T value_;

  protected:
    // Initialize the object for the test
    virtual void SetUp() {
      //// Constructs a random point cloud
      pc_m_ = typename T::PointCloudPtr(new typename T::PointCloud());
      pc_s_ = typename T::PointCloudPtr(new typename T::PointCloud());
      for (int i = 0; i < 100; i++) {
        pc_m_->push_back(typename T::PointType(10 * rand(), 10 * rand(), 10 * rand()));
      }
      icp_.setInputReference(pc_m_);
    }
    // Destructor for the test
    virtual void TearDown() {
    }

    typename T::IcpMethod icp_;
    typename T::PointCloudPtr pc_m_;
    typename T::PointCloudPtr pc_s_;
};

using testing::Types;
// The list of types we want to test.
// XXX: missing point to plane
typedef Types < TypeDefinitions<IcpPointToPointHubert, pcl::PointXYZ>,
                TypeDefinitions<IcpPointToPointHubertXYZRGB, pcl::PointXYZRGB>,
                TypeDefinitions<IcpPointToPointHubertSim3, pcl::PointXYZ>,
                TypeDefinitions<IcpPointToPointHubertXYZRGBSim3, pcl::PointXYZRGB>
                > Implementations;
TYPED_TEST_CASE(IcpCommonTest, Implementations);

/**
 * Tests whether ICP converges when the two point clouds are already aligned
 */
TYPED_TEST(IcpCommonTest, Identity) {
  //  Executing the transformation
  typename TypeParam::PointCloudPtr pc_d(new typename TypeParam::PointCloud());
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

}
