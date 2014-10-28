//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#include <gtest/gtest.h>
#include "eigentools.hpp"

namespace icp {

/**
   Creates a test fixture
**/
class EigenToolsTest : public ::testing::Test {
  protected:
    // Initialize the object for the test
    virtual void SetUp() {
    }
    // Destructor for the test
    virtual void TearDown() {
    }
};

TEST_F(EigenToolsTest, CreateTransformationMatrix) {
  // Identity transformation
  Eigen::Matrix4f transformation;
  Eigen::Matrix4f expected;
  const int MAX_TRY = 100;
  for (int i = 0; i < MAX_TRY; i++) {

    transformation = eigentools::createTransformationMatrix(0.0f,
                     0.0f,
                     0.0f,
                     0.0f,
                     0.0f,
                     0.0f);
    expected << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;
    EXPECT_TRUE(transformation.allFinite()) <<
                                            "Try " << i << "/" << MAX_TRY <<
                                            " - Some values in the transformation matrix are NaN or infinite";
    EXPECT_TRUE(expected.isApprox(transformation)) << "Try " << i << "/" << MAX_TRY
        << "Expected:\n" << expected <<
        "\nActual\n" << transformation;
  }
}

}  // namespace icp
