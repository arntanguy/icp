//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#include <gtest/gtest.h>
#include <glog/logging.h>
#include "linear_algebra.hpp"

namespace test_icp {

/**
   Creates a test fixture
**/
class LinearAlgebraTest : public ::testing::Test {
  protected:
    // Initialize the object for the test
    virtual void SetUp() {
    }
    // Destructor for the test
    virtual void TearDown() {
    }
};

//TEST_F(LinearAlgebraTest, TestSIM3) {
//  {
//    Eigen::Matrix<float, 7, 1> twist = Eigen::Matrix<float, 7, 1>::Zero();
//    Eigen::Matrix4f result = la::expLie(twist);
//    Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
//    EXPECT_TRUE(expected.isApprox(result)) << "Failed for twist: " << twist << "\nExpected: " << expected << "\n" << "Result: " << result;
//  }
//  {
//    Eigen::Matrix<float, 7, 1> twist;
//    twist << -0.001, 0.002, -0.0008, 12, 0.3, 0.9, 0;
//    Eigen::Matrix4f result = la::expLie(twist);
//    Eigen::Matrix4f expected;
//    expected << 
//    0.9992,    0.0411,   -0.0024,   -0.0010,
//   -0.0343,    0.8634,    0.5033,   -0.0001,
//    0.0228,   -0.5028,    0.8641,   -0.0000,
//         0,         0,         0,    1.0000;
//    EXPECT_TRUE(expected.isApprox(result)) << "Failed for twist: " << twist << "\nExpected: " << expected << "\n" << "Result: " << result;
//  }
//  {
//    Eigen::Matrix<float, 7, 1> twist;
//    twist << 0, 0, 0, 0, 0, 0, 3;
//    Eigen::Matrix4f result = la::expLie(twist);
//    Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
//    expected(0,0) = 20.08550;
//    expected(1,1) = 20.08550;
//    expected(2,2) = 20.08550;
//    EXPECT_TRUE(expected.isApprox(result));
//  }
//  {
//    Eigen::Matrix<float, 7, 1> twist;
//    twist << 0.4, 9, 2.3, 0.5, -0.7, 0.0, 3;
//    Eigen::Matrix4f result = la::expLie(twist);
//    Eigen::Matrix4f expected;
//    expected << 18.0232, -1.4731, -4.4533, -0.2893,
//                -1.4731, 19.0333, -3.1809, 2.5535,
//                4.4533, 3.1809, 16.9710, 1.6901,
//                0,         0,      0,    1.0000;
//    EXPECT_TRUE(expected.isApprox(result)) << "Expected: " << expected << "\n" << "Result: " << result;
//  }
//}

}
