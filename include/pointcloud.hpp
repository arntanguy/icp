#pragma once

#include <Eigen/Dense>
#include "sophus/se3.hpp"

#include <stdexcept>

namespace icp
{

template<typename T>
class Pointcloud
{
 public:
  typedef Eigen::Matrix<T, Eigen::Dynamic, 4> Pcl;
 protected:
  Pcl pointcloud_;
 public:
  Pointcloud() {
  }
  Pointcloud(const Pcl& pcl) {
    pointcloud_ = pcl;
    Sophus::SE3Group<T> g;
  }
  virtual ~Pointcloud() {
  }

  void setPoints(const Pcl& pts);

  int nbPoints() const {
    return pointcloud_.rows();
  }
};

template<typename T>
void Pointcloud<T>::setPoints(const Pointcloud<T>::Pcl& pts) {
  if(pts.cols() != 4) {
    throw std::runtime_error("Point cloud must be of size Nx4");
  }
}


}  // namespace icp
