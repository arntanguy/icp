#ifndef ICP_ICP_INSTANCIATE_HPP
#define ICP_INSTANCIATE_HPP 



#define INSTANCIATE_ICP_FUN(Scalar, Src, Dst) \
  template class icp::Icp_<Scalar, Eigen::Matrix<Scalar, 6, 1>, Src, Dst, ErrorPointToPoint<Scalar, Src>, MEstimatorHubert<Scalar, Src>>; \
  template class icp::Icp_<Scalar, Eigen::Matrix<Scalar, 6, 1>, Src, Dst, ErrorPointToPlane<Scalar, Src>, MEstimatorHubert<Scalar, Src>>; \
  template class icp::Icp_<Scalar, Eigen::Matrix<Scalar, 7, 1>, Src, Dst, ErrorPointToPointSim3<Scalar, Src>, MEstimatorHubert<Scalar, Src>>; \
  template class icp::Icp_<Scalar, Eigen::Matrix<Scalar, 7, 1>, Src, Dst, ErrorPointToPlaneSim3<Scalar, Src>, MEstimatorHubert<Scalar, Src>>;

#define INSTANCIATE_ERROR_FUN(Scalar, Src, Dst) \
  template class icp::Error<Scalar, 6, Src, Dst>; \
  template class icp::Error<Scalar, 7, Src, Dst>; 

#define INSTANCIATE_ERROR_POINT_TO_POINT_FUN(Scalar, Src, Dst) \
  template class icp::ErrorPointToPoint<Scalar, Src>;

#define INSTANCIATE_ERROR_POINT_TO_PLANE_FUN(Scalar, Src, Dst) \
  template class icp::ErrorPointToPlane<Scalar, Src, Dst>;

#define INSTANCIATE_ERROR_POINT_TO_POINT_SIM3_FUN(Scalar, Src, Dst) \
  template class icp::ErrorPointToPointSim3<Scalar, Src>;

#define INSTANCIATE_ERROR_POINT_TO_PLANE_SIM3_FUN(Scalar, Point) \
  template class icp::ErrorPointToPlaneSim3<Scalar, Point>;

#define INSTANCIATE_MESTIMATOR_HUBERT_FUN(Scalar, Point) \
  template class icp::MEstimatorHubert<Scalar, Point>;

#define INSTANCIATE_CONSTRAINTS_FUN(Scalar, DegreesOfFreedom)  \
  template class icp::Constraints_<Scalar, DegreesOfFreedom>;

#define INSTANCIATE_ERROR \
    INSTANCIATE_ERROR_FUN(float, pcl::PointXYZ, pcl::PointXYZ) \
    INSTANCIATE_ERROR_FUN(float, pcl::PointXYZRGB, pcl::PointXYZRGB) \
    INSTANCIATE_ERROR_FUN(float, pcl::PointNormal, pcl::PointNormal) 

#define INSTANCIATE_ERROR_POINT_TO_POINT \
    INSTANCIATE_ERROR_POINT_TO_POINT_FUN(float, pcl::PointXYZ, pcl::PointXYZ) \
    INSTANCIATE_ERROR_POINT_TO_POINT_FUN(float, pcl::PointXYZRGB, pcl::PointXYZRGB) \
    INSTANCIATE_ERROR_POINT_TO_POINT_FUN(float, pcl::PointNormal, pcl::PointNormal) \
    //INSTANCIATE_ERROR_POINT_TO_POINT_FUN(double, pcl::PointNormal, pcl::PointNormal)

#define INSTANCIATE_ERROR_POINT_TO_PLANE \
    INSTANCIATE_ERROR_POINT_TO_PLANE_FUN(float, pcl::PointNormal, pcl::PointNormal) \
    //INSTANCIATE_ERROR_POINT_TO_PLANE_FUN(double, pcl::PointNormal, pcl::PointNormal)

#define INSTANCIATE_ERROR_POINT_TO_POINT_SIM3 \
    INSTANCIATE_ERROR_POINT_TO_POINT_SIM3_FUN(float, pcl::PointXYZ, pcl::PointXYZ) \
    INSTANCIATE_ERROR_POINT_TO_POINT_SIM3_FUN(float, pcl::PointNormal, pcl::PointNormal) \
    INSTANCIATE_ERROR_POINT_TO_POINT_SIM3_FUN(float, pcl::PointXYZRGB, pcl::PointXYZRGB) \
    //INSTANCIATE_ERROR_POINT_TO_POINT_SIM3_FUN(double, pcl::PointXYZ, pcl::PointXYZ)
#define INSTANCIATE_ERROR_POINT_TO_PLANE_SIM3 \
    INSTANCIATE_ERROR_POINT_TO_PLANE_SIM3_FUN(float, pcl::PointNormal);

#define INSTANCIATE_MESTIMATOR_HUBERT \
    INSTANCIATE_MESTIMATOR_HUBERT_FUN(float, pcl::PointXYZ) \
    INSTANCIATE_MESTIMATOR_HUBERT_FUN(float, pcl::PointXYZRGB) \
    INSTANCIATE_MESTIMATOR_HUBERT_FUN(float, pcl::PointNormal) \
    //INSTANCIATE_MESTIMATOR_HUBERT_FUN(double, pcl::PointXYZ)

#define INSTANCIATE_CONSTRAINTS  \
  INSTANCIATE_CONSTRAINTS_FUN(float, 6) \
  INSTANCIATE_CONSTRAINTS_FUN(float, 7)

#define INSTANCIATE_ICP \
  template class icp::Icp_<float, Eigen::Matrix<float, 6, 1>, pcl::PointXYZ, pcl::PointXYZ, ErrorPointToPoint<float, pcl::PointXYZ>, MEstimatorHubert<float, pcl::PointXYZ>>; \
  template class icp::Icp_<float, Eigen::Matrix<float, 6, 1>, pcl::PointXYZRGB, pcl::PointXYZRGB, ErrorPointToPoint<float, pcl::PointXYZRGB>, MEstimatorHubert<float, pcl::PointXYZRGB>>; \
  template class icp::Icp_<float, Eigen::Matrix<float, 6, 1>, pcl::PointNormal, pcl::PointNormal, ErrorPointToPlane<float, pcl::PointNormal, pcl::PointNormal>, MEstimatorHubert<float, pcl::PointNormal>>; \
  template class icp::Icp_<float, Eigen::Matrix<float, 7, 1>, pcl::PointXYZ, pcl::PointXYZ, ErrorPointToPointSim3<float, pcl::PointXYZ>, MEstimatorHubert<float, pcl::PointXYZ>>; \
  template class icp::Icp_<float, Eigen::Matrix<float, 7, 1>, pcl::PointXYZRGB, pcl::PointXYZRGB, ErrorPointToPointSim3<float, pcl::PointXYZRGB>, MEstimatorHubert<float, pcl::PointXYZRGB>>; \
  template class icp::Icp_<float, Eigen::Matrix<float, 7, 1>, pcl::PointNormal, pcl::PointNormal, ErrorPointToPlaneSim3<float, pcl::PointNormal>, MEstimatorHubert<float, pcl::PointNormal>>;



#endif
