#ifndef ICP_INSTANCIATE_HPP
#define ICP_INSTANCIATE_HPP 


#define INSTANCIATE_ERROR_FUN(Scalar, Src, Dst) \
  template class icp::Error<Scalar, 6, Src, Dst>; \
  template class icp::Error<Scalar, 7, Src, Dst>; 

#define INSTANCIATE_ERROR_POINT_TO_POINT_FUN(Scalar, Src, Dst) \
  template class icp::ErrorPointToPoint<Scalar, Src, Dst>;

#define INSTANCIATE_ERROR_POINT_TO_PLANE_FUN(Scalar, Src, Dst) \
  template class icp::ErrorPointToPlane<Scalar, Src, Dst>;

#define INSTANCIATE_ERROR_POINT_TO_POINT_SIM3_FUN(Scalar, Src, Dst) \
  template class icp::ErrorPointToPointSim3<Scalar, Src, Dst>;

#define INSTANCIATE_ERROR_POINT_TO_PLANE_SIM3_FUN(Scalar, Src, Dst) \
  template class icp::ErrorPointToPlaneSim3<Scalar, Src, Dst>;

#define INSTANCIATE_MESTIMATOR_HUBERT_FUN(Scalar, PointReference, PointSource) \
  template class icp::MEstimatorHubert<Scalar, PointReference, PointSource>;

#define INSTANCIATE_MAD_VECTOR_FUN(Scalar, PointReference, PointSource) \
  template class icp::MaximumAbsoluteDeviation<Scalar, PointReference, PointSource>;

#define INSTANCIATE_MAD_FUN(Scalar) \
  template class icp::MaximumAbsoluteDeviationVector<Scalar>;

#define INSTANCIATE_CONSTRAINTS_FUN(Scalar, DegreesOfFreedom)  \
  template class icp::Constraints_<Scalar, DegreesOfFreedom>; \
  template class icp::JacobianConstraints<Scalar, DegreesOfFreedom>;

#define INSTANCIATE_ERROR \
    INSTANCIATE_ERROR_FUN(float, pcl::PointXYZ, pcl::PointXYZ) \
    INSTANCIATE_ERROR_FUN(float, pcl::PointXYZRGB, pcl::PointXYZRGB) \
    INSTANCIATE_ERROR_FUN(float, pcl::PointNormal, pcl::PointNormal) \
    INSTANCIATE_ERROR_FUN(float, pcl::PointXYZ, pcl::PointNormal) \
    INSTANCIATE_ERROR_FUN(float, pcl::PointXYZRGB, pcl::PointNormal) 

#define INSTANCIATE_ERROR_POINT_TO_POINT \
    INSTANCIATE_ERROR_POINT_TO_POINT_FUN(float, pcl::PointXYZ, pcl::PointXYZ) \
    INSTANCIATE_ERROR_POINT_TO_POINT_FUN(float, pcl::PointXYZRGB, pcl::PointXYZRGB) \
    INSTANCIATE_ERROR_POINT_TO_POINT_FUN(float, pcl::PointNormal, pcl::PointNormal) \
    //INSTANCIATE_ERROR_POINT_TO_POINT_FUN(double, pcl::PointNormal, pcl::PointNormal)

#define INSTANCIATE_ERROR_POINT_TO_PLANE \
    INSTANCIATE_ERROR_POINT_TO_PLANE_FUN(float, pcl::PointXYZ, pcl::PointNormal) \
    INSTANCIATE_ERROR_POINT_TO_PLANE_FUN(float, pcl::PointNormal, pcl::PointNormal)
    //INSTANCIATE_ERROR_POINT_TO_PLANE_FUN(double, pcl::PointNormal, pcl::PointNormal)

#define INSTANCIATE_ERROR_POINT_TO_POINT_SIM3 \
    INSTANCIATE_ERROR_POINT_TO_POINT_SIM3_FUN(float, pcl::PointXYZ, pcl::PointXYZ) \
    INSTANCIATE_ERROR_POINT_TO_POINT_SIM3_FUN(float, pcl::PointXYZ, pcl::PointNormal) \
    INSTANCIATE_ERROR_POINT_TO_POINT_SIM3_FUN(float, pcl::PointNormal, pcl::PointNormal) \
    INSTANCIATE_ERROR_POINT_TO_POINT_SIM3_FUN(float, pcl::PointXYZRGB, pcl::PointXYZRGB) \
    //INSTANCIATE_ERROR_POINT_TO_POINT_SIM3_FUN(double, pcl::PointXYZ, pcl::PointXYZ)
#define INSTANCIATE_ERROR_POINT_TO_PLANE_SIM3 \
    INSTANCIATE_ERROR_POINT_TO_PLANE_SIM3_FUN(float, pcl::PointNormal, pcl::PointNormal); \
    INSTANCIATE_ERROR_POINT_TO_PLANE_SIM3_FUN(float, pcl::PointXYZ, pcl::PointNormal);

#define INSTANCIATE_MAD \
  INSTANCIATE_MAD_FUN(float); \
  INSTANCIATE_MAD_VECTOR_FUN(float, pcl::PointXYZ, pcl::PointXYZ); \
  INSTANCIATE_MAD_VECTOR_FUN(float, pcl::PointXYZ, pcl::PointNormal); \
  INSTANCIATE_MAD_VECTOR_FUN(float, pcl::PointNormal, pcl::PointNormal); \
  INSTANCIATE_MAD_VECTOR_FUN(float, pcl::PointXYZ, pcl::PointXYZRGB); \
  INSTANCIATE_MAD_VECTOR_FUN(float, pcl::PointXYZRGB, pcl::PointXYZ); \
  INSTANCIATE_MAD_VECTOR_FUN(float, pcl::PointXYZRGB, pcl::PointNormal); \
  INSTANCIATE_MAD_VECTOR_FUN(float, pcl::PointXYZRGB, pcl::PointXYZRGB);

#define INSTANCIATE_MESTIMATOR_HUBERT \
    INSTANCIATE_MESTIMATOR_HUBERT_FUN(float, pcl::PointXYZ, pcl::PointXYZ) \
    INSTANCIATE_MESTIMATOR_HUBERT_FUN(float, pcl::PointNormal, pcl::PointNormal) \
    INSTANCIATE_MESTIMATOR_HUBERT_FUN(float, pcl::PointXYZ, pcl::PointXYZRGB) \
    INSTANCIATE_MESTIMATOR_HUBERT_FUN(float, pcl::PointXYZRGB, pcl::PointXYZRGB) \
    INSTANCIATE_MESTIMATOR_HUBERT_FUN(float, pcl::PointXYZ, pcl::PointNormal) \
    //INSTANCIATE_MESTIMATOR_HUBERT_FUN(double, pcl::PointXYZ)

#define INSTANCIATE_CONSTRAINTS  \
  INSTANCIATE_CONSTRAINTS_FUN(float, 6) \
  INSTANCIATE_CONSTRAINTS_FUN(float, 7)

#define INSTANCIATE_ICP \
  template class icp::Icp_<float, pcl::PointXYZ, pcl::PointXYZ, ErrorPointToPoint<float, pcl::PointXYZ, pcl::PointXYZ>, MEstimatorHubert<float, pcl::PointXYZ, pcl::PointXYZ>>; \
  template class icp::Icp_<float, pcl::PointXYZRGB, pcl::PointXYZRGB, ErrorPointToPoint<float, pcl::PointXYZRGB, pcl::PointXYZRGB>, MEstimatorHubert<float, pcl::PointXYZRGB, pcl::PointXYZRGB>>; \
  template class icp::Icp_<float, pcl::PointXYZ, pcl::PointNormal, ErrorPointToPlane<float, pcl::PointXYZ, pcl::PointNormal>, MEstimatorHubert<float, pcl::PointXYZ, pcl::PointNormal>>; \
  template class icp::Icp_<float, pcl::PointNormal, pcl::PointNormal, ErrorPointToPlane<float, pcl::PointNormal, pcl::PointNormal>, MEstimatorHubert<float, pcl::PointNormal, pcl::PointNormal>>; \
  template class icp::Icp_<float, pcl::PointXYZ, pcl::PointXYZ, ErrorPointToPointSim3<float, pcl::PointXYZ, pcl::PointXYZ>, MEstimatorHubert<float, pcl::PointXYZ, pcl::PointXYZ>>; \
  template class icp::Icp_<float, pcl::PointXYZRGB, pcl::PointXYZRGB, ErrorPointToPointSim3<float, pcl::PointXYZRGB, pcl::PointXYZRGB>, MEstimatorHubert<float, pcl::PointXYZRGB, pcl::PointXYZRGB>>; \
  template class icp::Icp_<float, pcl::PointNormal, pcl::PointNormal, ErrorPointToPlaneSim3<float, pcl::PointNormal, pcl::PointNormal>, MEstimatorHubert<float, pcl::PointNormal, pcl::PointNormal>>;\
  template class icp::Icp_<float, pcl::PointXYZ, pcl::PointNormal, ErrorPointToPlaneSim3<float, pcl::PointXYZ, pcl::PointNormal>, MEstimatorHubert<float, pcl::PointXYZ, pcl::PointNormal>>;



#endif
