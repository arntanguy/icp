//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <icp/mestimator_hubert.hpp>
#include <icp/logging.hpp>

namespace test_icp {

using namespace icp;

/**
   Creates a test fixture
**/
class MEstimatorHubertTest : public ::testing::Test {
  protected:
    // Initialize the object for the test
    virtual void SetUp() {
      // Constructs a random point cloud
      pc_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(
              new pcl::PointCloud<pcl::PointXYZ>());
      for (int i = 0; i < 100; i++) {
        pc_->push_back(pcl::PointXYZ(rand(), rand(), rand()));
      }
    }
    // Destructor for the test
    virtual void TearDown() {
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_;
};

TEST_F(MEstimatorHubertTest, MAD) {
  Eigen::VectorXf v(7);
  v << 1.f, 1.f, 2.f, 2.f, 4.f, 6.f, 9.f;

  const float expected_mad = 1.f;

  MaximumAbsoluteDeviation<float> mad;
  mad(v);
  float actual_mad = mad.getMad();

  EXPECT_FLOAT_EQ(expected_mad, actual_mad);
}

TEST_F(MEstimatorHubertTest, TestComputeWeightsEven) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
  pc->push_back(
    pcl::PointXYZ( 0.007993371029844,   0.089788842598508,  -0.101494364268014));
  pc->push_back(
    pcl::PointXYZ(-0.094848098357051,  -0.013193786792458,  -0.047106991268317));
  pc->push_back(
    pcl::PointXYZ( 0.041149062142337,  -0.014720145615127,   0.013702487413005));
  pc->push_back(
    pcl::PointXYZ( 0.067697780568403,   0.100777340530544,  -0.029186337575357));
  pc->push_back(
    pcl::PointXYZ( 0.085773254520536,  -0.212365546241575,   0.030181855526101));
  pc->push_back(
    pcl::PointXYZ(-0.069115912538299,  -0.050458640551401,   0.039993094295580));
  pc->push_back(
    pcl::PointXYZ( 0.044937762316685,  -0.127059444980866,  -0.092996155894013));
  pc->push_back(
    pcl::PointXYZ( 0.010063335031508,  -0.038258480270765,  -0.017683026592923));
  pc->push_back(
    pcl::PointXYZ( 0.082606999846992,   0.064867926204862,  -0.213209459916153));
  pc->push_back(
    pcl::PointXYZ( 0.053615707992592,   0.082572714924176,   0.114536171051847));


  Eigen::MatrixXf expected(10, 4);
  expected <<  1.000000000000000f, 1.000000000000000f, 1.000000000000000f, 1.f,
           0.442786366169536f, 1.000000000000000f, 1.000000000000000f, 1.f,
           1.000000000000000f, 1.000000000000000f, 1.000000000000000f, 1.f,
           1.000000000000000f, 1.000000000000000f, 1.000000000000000f, 1.f,
           1.000000000000000f, 0.793208192411082f, 1.000000000000000f, 1.f,
           0.544372757455962f, 1.000000000000000f, 1.000000000000000f, 1.f,
           1.000000000000000f, 1.000000000000000f, 1.000000000000000f, 1.f,
           1.000000000000000f, 1.000000000000000f, 1.000000000000000f, 1.f,
           1.000000000000000f, 1.000000000000000f, 0.553531539425519f, 1.f,
           1.000000000000000f, 1.000000000000000f, 0.761366060511011f, 1.f;


  MEstimatorHubertXYZ m;
  m.setInputCloud(pc);
  m.computeWeights();
  Eigen::MatrixXf w = m.getWeights();
  LOG(INFO) << "WEIGHTS:\n" << w;

  EXPECT_TRUE(w.isApprox(expected, 10e-3)) << "Expected: \n" << expected
                                    << "\nActual:\n" << w;
}

TEST_F(MEstimatorHubertTest, TestComputeWeightsOdd) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
  pc->push_back(pcl::PointXYZ( 0.0538f,  0.2769f,  0.1409f));
  pc->push_back(pcl::PointXYZ( 0.1834f, -0.1350f,  0.1417f));
  pc->push_back(pcl::PointXYZ(-0.2259f,  0.3035f,  0.0671f));
  pc->push_back(pcl::PointXYZ( 0.0862f,  0.0725f, -0.1207f));
  pc->push_back(pcl::PointXYZ( 0.0319f, -0.0063f,  0.0717f));
  pc->push_back(pcl::PointXYZ(-0.1308f,  0.0715f,  0.1630f));
  pc->push_back(pcl::PointXYZ(-0.0434f, -0.0205f,  0.0489f));
  pc->push_back(pcl::PointXYZ( 0.0343f, -0.0124f,  0.1035f));
  pc->push_back(pcl::PointXYZ( 0.3578f,  0.1490f,  0.0727f));

  Eigen::MatrixXf expected(9, 4);
  expected <<
           1.000000000000000f, 0.732853141070062f, 0.809954818326118f, 1.f,
           0.934307583010008f, 0.729326396501058f, 0.800382922241652f, 1.f,
           0.535580009489893f, 0.648996491287797f, 1.000000000000000f, 1.f,
           1.000000000000000f, 1.000000000000000f, 0.285627656952000f, 1.f,
           1.000000000000000f, 1.000000000000000f, 1.000000000000000f, 1.f,
           0.844261808829360f, 1.000000000000000f, 0.611623471266495f, 1.f,
           1.000000000000000f, 1.000000000000000f, 1.000000000000000f, 1.f,
           1.000000000000000f, 1.000000000000000f, 1.000000000000000f, 1.f,
           0.430591473856147f, 1.000000000000000f, 1.000000000000000f, 1.f;

  MEstimatorHubertXYZ m;
  m.setInputCloud(pc);
  m.computeWeights();
  Eigen::MatrixXf w = m.getWeights();

  EXPECT_TRUE(w.isApprox(expected, 10e-3)) << "Expected: \n" << expected
                                    << "\nActual:\n" << w;
}

}  // namespace test_icp
