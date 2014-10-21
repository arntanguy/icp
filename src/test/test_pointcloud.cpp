#include <gtest/gtest.h>
#include "pointcloud.hpp"

namespace icp
{

/**
   Creates a test fixture
**/
class PointcloudTest : public ::testing::Test
{
  protected:
  // Initialize the object for the test
  virtual void SetUp() {
  }
  // Destructor for the test
  virtual void TearDown() {
  }

  Pointcloud<float> p_;

};

TEST_F(PointcloudTest, IsEmptyInitially) {
  EXPECT_EQ(0, p_.nbPoints());
}

TEST_F(PointcloudTest, SetPoints) {
  Pointcloud<float>::Pcl pc;
  pc << 1,0,0,1,
      0,1,0,1,
      0,0,1,1,
      1,1,1,1;
  p_.setPoints(pc);
  EXPECT_EQ(4, p_.nbPoints());
}


}  // namespace icp
