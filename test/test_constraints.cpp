//  This file is part of the Icp Library,
//
//  Copyright (C) 2014-2015 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <icp/constraints.hpp>
#include <icp/logging.hpp>

namespace test_icp {

using namespace icp;

class TestConstraints : public ::testing::Test
{
  typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> JacobianMatrix;
  protected:
    virtual void SetUp() {
      J_.resize(3, 6);
      J_ << 11, 12, 13, 14, 15, 16,
      21, 22, 23, 24, 25, 26,
      31, 32, 33, 34, 35, 36;
    }

    virtual void TearDown() {
    }

    Constraints6 c_;
    JacobianMatrix J_;
    JacobianMatrix Jc_;
};

TEST_F(TestConstraints, NoConstraint) {
  EXPECT_TRUE(!c_.hasConstraints()) << "Empty constraint is not empty!";
  EXPECT_EQ(c_.getTranslationConstraint().numFixedAxes(), 0) << "An empty constraint should not have any fixed axes!";
  //Eigen::Matrix<float, Eigen::Dynamic, 1> twist_dynamic;
  Eigen::Matrix<float, 6, 1> twist;
  twist << 1, 2, 3, 4, 5, 6;
  c_.getTwist(twist);
  ASSERT_TRUE(c_.getTwist(twist).isApprox(twist)) << "The twist shouldn't be modified when no constraints are present!";
}

TEST_F(TestConstraints, TestFixedX) {
  FixTranslationConstraint t_c(true, false, false);
  c_.setTranslationConstraint(t_c);
  Eigen::Matrix<float, 5, 1> test_twist;
  test_twist << 2, 3, 4, 5, 6;
  Eigen::Matrix<float, 6, 1> expected_twist;
  expected_twist << 0, 2, 3, 4, 5, 6;
  ASSERT_TRUE(c_.getTwist(test_twist).isApprox(expected_twist)) <<
      "When X constraint is active, the twist should start with 0 (no translation) and have the usual values elsewhere";
}

TEST_F(TestConstraints, TestFixedY) {
  FixTranslationConstraint t_c(false, true, false);
  c_.setTranslationConstraint(t_c);
  Eigen::Matrix<float, 5, 1> test_twist;
  test_twist << 1, 3, 4, 5, 6;
  Eigen::Matrix<float, 6, 1> expected_twist;
  expected_twist << 1, 0, 3, 4, 5, 6;
  ASSERT_TRUE(c_.getTwist(test_twist).isApprox(expected_twist)) <<
      "When Y constraint is active, the twist should have 0 as a second component (no translation) and have the usual values elsewhere";
}

TEST_F(TestConstraints, TestFixedZ) {
  FixTranslationConstraint t_c(false, false, true);
  c_.setTranslationConstraint(t_c);
  Eigen::Matrix<float, 5, 1> test_twist;
  test_twist << 1, 2, 4, 5, 6;
  Eigen::Matrix<float, 6, 1> expected_twist;
  expected_twist << 1, 2, 0, 4, 5, 6;
  ASSERT_TRUE(c_.getTwist(test_twist).isApprox(expected_twist)) <<
      "When Z constraint is active, the twist should have 0 as a third component (no translation) and have the usual values elsewhere";
}

TEST_F(TestConstraints, TestFixedXYZ) {
  FixTranslationConstraint t_c(true, true, true);
  c_.setTranslationConstraint(t_c);
  Eigen::Matrix<float, 3, 1> test_twist;
  test_twist << 4, 5, 6;
  Eigen::Matrix<float, 6, 1> expected_twist;
  expected_twist << 0, 0, 0, 4, 5, 6;
  ASSERT_TRUE(c_.getTwist(test_twist).isApprox(expected_twist)) <<
      "X, Y, Z are fixed, the translation part of the twist should be 0!";
}

TEST_F(TestConstraints, TestProcessJacobianX) {
  FixTranslationConstraint t_c(true, false, false);
  c_.setTranslationConstraint(t_c);
  c_.processJacobian(J_, Jc_);


  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jexpected;
  Jexpected.resize(3, 5);
  Jexpected << 12, 13, 14, 15, 16,
            22, 23, 24, 25, 26,
            32, 33, 34, 35, 36;
  ASSERT_TRUE(Jexpected.isApprox(Jc_));
}

TEST_F(TestConstraints, TestProcessJacobianY) {
  FixTranslationConstraint t_c(false, true, false);
  c_.setTranslationConstraint(t_c);
  c_.processJacobian(J_, Jc_);


  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jexpected;
  Jexpected.resize(3, 5);
  Jexpected << 11, 13, 14, 15, 16,
            21, 23, 24, 25, 26,
            31, 33, 34, 35, 36;
  ASSERT_TRUE(Jexpected.isApprox(Jc_));
}

TEST_F(TestConstraints, TestProcessJacobianZ) {
  FixTranslationConstraint t_c(false, false, true);
  c_.setTranslationConstraint(t_c);
  c_.processJacobian(J_, Jc_);


  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jexpected;
  Jexpected.resize(3, 5);
  Jexpected << 11, 12, 14, 15, 16,
            21, 22, 24, 25, 26,
            31, 32, 34, 35, 36;
  ASSERT_TRUE(Jexpected.isApprox(Jc_));
}

TEST_F(TestConstraints, TestProcessJacobianXY) {
  FixTranslationConstraint t_c(true, true, false);
  c_.setTranslationConstraint(t_c);
  c_.processJacobian(J_, Jc_);

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jexpected;
  Jexpected.resize(3, 4);
  Jexpected << 13, 14, 15, 16,
               23, 24, 25, 26,
               33, 34, 35, 36;
  ASSERT_TRUE(Jexpected.isApprox(Jc_)) << "expected: " << Jexpected << "\nactual: " << J_;
}

TEST_F(TestConstraints, TestProcessJacobianXZ) {
  FixTranslationConstraint t_c(true, false, true);
  c_.setTranslationConstraint(t_c);
  c_.processJacobian(J_, Jc_);

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jexpected;
  Jexpected.resize(3, 4);
  Jexpected << 12, 14, 15, 16,
               22, 24, 25, 26,
               32, 34, 35, 36;
  ASSERT_TRUE(Jexpected.isApprox(Jc_)) << "expected: " << Jexpected << "\nactual: " << J_;
}

TEST_F(TestConstraints, TestProcessJacobianYZ) {
  FixTranslationConstraint t_c(false, true, true);
  c_.setTranslationConstraint(t_c);
  c_.processJacobian(J_, Jc_);

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jexpected;
  Jexpected.resize(3, 4);
  Jexpected << 11, 14, 15, 16,
               21, 24, 25, 26,
               31, 34, 35, 36;
  ASSERT_TRUE(Jexpected.isApprox(Jc_)) << "expected: " << Jexpected << "\nactual: " << J_;
}

TEST_F(TestConstraints, TestProcessJacobianXYZ) {
  FixTranslationConstraint t_c(true, true, true);
  c_.setTranslationConstraint(t_c);
  c_.processJacobian(J_, Jc_);

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jexpected;
  Jexpected.resize(3, 3);
  Jexpected << 14, 15, 16,
            24, 25, 26,
            34, 35, 36;
  ASSERT_TRUE(Jexpected.isApprox(Jc_)) << "expected: " << Jexpected << "\nactual: " << J_;
}

}  // namespace test_icp
