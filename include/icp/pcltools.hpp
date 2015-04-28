#ifndef PCLTOOLS_HPP
#define PCLTOOLS_HPP

#include <pcl/point_cloud.h>
#include <glog/logging.h>



namespace pcltools
{
template<typename PointSource, typename PointDestination>
typename pcl::PointCloud<PointSource>::Ptr substractPointcloud(const typename pcl::PointCloud<PointSource>::Ptr pc1,
    const typename pcl::PointCloud<PointDestination>::Ptr pc2);

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr substractPointcloud(const typename pcl::PointCloud<PointT>::Ptr pc1,
    const typename pcl::PointCloud<PointT>::Ptr pc2);

template<typename PointT>
bool isApprox(const PointT &p1, const PointT &p2, const float thr = 10e-5) {
  return (fabs(p1.x - p2.x) < thr) &&
         (fabs(p1.y - p2.y) < thr) &&
         (fabs(p1.z - p2.z) < thr);
}

template<typename Point1, typename Point2>
Point1 substract(const Point1 &p1, const Point2 &p2) {
  Point1 result;
  result.x = p2.x - p1.x;
  result.y = p2.y - p1.y;
  result.z = p2.z - p1.z;
  return result;
}

template<typename PointSource, typename PointDestination>
typename pcl::PointCloud<PointSource>::Ptr substractPointcloud(
    const typename pcl::PointCloud<PointSource>::Ptr pc1,
    const typename pcl::PointCloud<PointDestination>::Ptr pc2) {

  if (pc1->size() != pc2->size()) throw
    std::runtime_error("pcltools::substract - Error the point clouds must have the same size!");
  if (pc1->size() == 0 || pc2->size() == 0) throw
    std::runtime_error("pcltools::substract - Error the point clouds must not be empty!");

  typename pcl::PointCloud<PointSource>::Ptr result(new pcl::PointCloud<PointSource>());
  result->reserve(pc1->size());
  for (unsigned int i = 0; i < pc1->size(); ++i) {
    const PointSource &p1 = (*pc1)[i];
    const PointDestination &p2 = (*pc2)[i];
    const PointSource &r = substract(p1, p2);
    result->push_back(r);
  }
  return result;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr substractPointcloud(const typename pcl::PointCloud<PointT>::Ptr pc1,
    const typename pcl::PointCloud<PointT>::Ptr pc2) {
  return substractPointcloud<PointT, PointT>(pc1, pc2);
}
template<typename PointT>
void subPointCloud(const typename pcl::PointCloud<PointT>::Ptr &src,
                   const std::vector<int> &indices,
                   typename pcl::PointCloud<PointT>::Ptr &dst) {
  dst->clear();
  dst->reserve(indices.size());
  for(unsigned int i=0; i < indices.size(); i++) {
    dst->push_back((*src)[indices[i]]);
  }
}


} /* pcltools */

#endif
