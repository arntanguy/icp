#include "pointcloud.hpp"
#include <stdexcept>

namespace icp {

template<typename T>
void Pointcloud<T>::setPoints(const Pointcloud<T>::Pcl& pts) {
  if(pts.cols() != 4) {
    throw std::runtime_error("Point cloud must be of size Nx4");
  }
}

}  // namespace icp
