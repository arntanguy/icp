#ifndef ICP_UTILS_HPP
#define ICP_UTILS_HPP

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>


namespace icp
{

/**
 * @brief Computes an initial estimation based on N matching points
 * N needs to be at least 3 non coplanar points, or 4 coplanar ones
 *
 * @param points_src
 * Source points
 * @param points_dst
 * Matchin points in the other cloud (destination)
 * @param T
 * Rigid SE3 transformation of an initial registration from source to destination
 */
template<typename Dtype>
void initialEstimate(const pcl::PointCloud<pcl::PointXYZ> &points_src,
    const pcl::PointCloud<pcl::PointXYZ> &points_dst, Eigen::Matrix<Dtype, 4, 4>& T) {
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
  transformation_estimation.estimateRigidTransformation(points_src, points_dst, T);
}

}

#endif
