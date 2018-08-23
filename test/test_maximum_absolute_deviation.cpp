//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.



#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <list>
#include <icp/mestimator.hpp>
#include <icp/logging.hpp>

using namespace icp;

#define RAND_SCALE 10

class MaximumAbsoluteDeviationVectorTest : public ::testing::Test {
  protected:
    // Initialize the object for the test
    virtual void SetUp() {
    }
    // Destructor for the test
    virtual void TearDown() {
    }
};

TEST_F(MaximumAbsoluteDeviationVectorTest, Dummy)
{
}
