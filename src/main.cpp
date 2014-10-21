#include <glog/logging.h>

#include "pointcloud.hpp"

int main(int argc, char *argv[])
{
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  LOG(INFO) << "Starting ICP program";
  icp::Pointcloud<float> pcl;

  return 0;
}

