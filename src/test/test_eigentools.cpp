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
#include "eigentools.hpp"

namespace eigentools {

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

TEST_F(EigenToolsTest, Sort) {
  LOG(WARNING) <<
               "This test only checks whether sorting works for VectorX, not matrices";
  Eigen::VectorXf unsorted(6);
  unsorted << 10, 1, 2, -4, 6, 8;
  Eigen::VectorXf reference_sorted(6);
  reference_sorted << -4, 1, 2, 6, 8, 10;

  Eigen::VectorXf sorted = unsorted;
  sort(sorted);
  EXPECT_TRUE(reference_sorted.isApprox(sorted)) << "Expected:\n" <<
      reference_sorted
      << "\nActual:\n" << sorted;
}

TEST_F(EigenToolsTest, Median) {
  LOG(WARNING) <<
               "This test only checks whether sorting works for VectorX, not matrices";
  { // Odd length single value
    Eigen::VectorXf unsorted(3);
    unsorted << 12, 3, 5;
    const float reference_median = 5.f;
    float actual_median = median(unsorted);
    EXPECT_FLOAT_EQ(reference_median, actual_median) << "Median of odd-length array (12, 3, 5) wrong";
  }
  {  // Odd length
    Eigen::VectorXf unsorted(15);
    unsorted << 3, 13, 7, 5, 21, 23, 39, 23, 40, 23, 14, 12, 56, 23, 29;
    const float reference_median = 23.f;
    float actual_median = median(unsorted);
    EXPECT_FLOAT_EQ(reference_median, actual_median) << "Median of odd-length array wrong";

  }

  { // Even length
    Eigen::VectorXf unsorted(14);
    unsorted << 3, 13, 7, 5, 21, 23, 23, 40, 23, 14, 12, 56, 23, 29;
    const float reference_median = 22.f;
    float actual_median = median(unsorted);
    EXPECT_FLOAT_EQ(reference_median, actual_median) << "Median of even-length array wrong";
  }
}


}  // namespace icp
