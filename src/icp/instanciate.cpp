#include <icp.hpp>
#include <mestimator_hubert.hpp>
#include <error.hpp>
#include <error_point_to_plane.hpp>
#include <error_point_to_point.hpp>
#include <error_point_to_plane_sim3.hpp>
#include <error_point_to_point_sim3.hpp>

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

#define INSTANCIATE_MAD_VECTOR_FUN(Scalar) \
  template class icp::MaximumAbsoluteDeviationVector<Scalar>;

#define INSTANCIATE_MAD_FUN(Scalar, PointReference, PointSource) \
  template class icp::MaximumAbsoluteDeviation<Scalar, PointReference, PointSource>;

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
  INSTANCIATE_MAD_FUN(float, pcl::PointXYZ, pcl::PointXYZ); \
  INSTANCIATE_MAD_FUN(float, pcl::PointXYZ, pcl::PointNormal); \
  INSTANCIATE_MAD_FUN(float, pcl::PointNormal, pcl::PointNormal); \
  INSTANCIATE_MAD_FUN(float, pcl::PointXYZ, pcl::PointXYZRGB); \
  INSTANCIATE_MAD_FUN(float, pcl::PointXYZRGB, pcl::PointXYZ); \
  INSTANCIATE_MAD_FUN(float, pcl::PointXYZRGB, pcl::PointNormal); \
  INSTANCIATE_MAD_FUN(float, pcl::PointXYZRGB, pcl::PointXYZRGB);
  INSTANCIATE_MAD_VECTOR_FUN(float); \

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
  template class icp::Icp_<float, Eigen::Matrix<float, 6, 1>, pcl::PointXYZ, pcl::PointXYZ, icp::ErrorPointToPoint<float, pcl::PointXYZ, pcl::PointXYZ>, icp::MEstimatorHubert<float, pcl::PointXYZ, pcl::PointXYZ>>; \
  template class icp::Icp_<float, Eigen::Matrix<float, 6, 1>, pcl::PointXYZRGB, pcl::PointXYZRGB, icp::ErrorPointToPoint<float, pcl::PointXYZRGB, pcl::PointXYZRGB>, icp::MEstimatorHubert<float, pcl::PointXYZRGB, pcl::PointXYZRGB>>; \
  template class icp::Icp_<float, Eigen::Matrix<float, 6, 1>, pcl::PointXYZ, pcl::PointNormal, icp::ErrorPointToPlane<float, pcl::PointXYZ, pcl::PointNormal>, icp::MEstimatorHubert<float, pcl::PointXYZ, pcl::PointNormal>>; \
  template class icp::Icp_<float, Eigen::Matrix<float, 6, 1>, pcl::PointNormal, pcl::PointNormal, icp::ErrorPointToPlane<float, pcl::PointNormal, pcl::PointNormal>, icp::MEstimatorHubert<float, pcl::PointNormal, pcl::PointNormal>>; \
  template class icp::Icp_<float, Eigen::Matrix<float, 7, 1>, pcl::PointXYZ, pcl::PointXYZ, icp::ErrorPointToPointSim3<float, pcl::PointXYZ, pcl::PointXYZ>, icp::MEstimatorHubert<float, pcl::PointXYZ, pcl::PointXYZ>>; \
  template class icp::Icp_<float, Eigen::Matrix<float, 7, 1>, pcl::PointXYZRGB, pcl::PointXYZRGB, icp::ErrorPointToPointSim3<float, pcl::PointXYZRGB, pcl::PointXYZRGB>, icp::MEstimatorHubert<float, pcl::PointXYZRGB, pcl::PointXYZRGB>>; \
  template class icp::Icp_<float, Eigen::Matrix<float, 7, 1>, pcl::PointNormal, pcl::PointNormal, icp::ErrorPointToPlaneSim3<float, pcl::PointNormal, pcl::PointNormal>, icp::MEstimatorHubert<float, pcl::PointNormal, pcl::PointNormal>>;\
  template class icp::Icp_<float, Eigen::Matrix<float, 7, 1>, pcl::PointXYZ, pcl::PointNormal, icp::ErrorPointToPlaneSim3<float, pcl::PointXYZ, pcl::PointNormal>, icp::MEstimatorHubert<float, pcl::PointXYZ, pcl::PointNormal>>;


INSTANCIATE_MAD;
INSTANCIATE_CONSTRAINTS;
INSTANCIATE_MESTIMATOR_HUBERT;
INSTANCIATE_ERROR;
INSTANCIATE_ERROR_POINT_TO_POINT;
INSTANCIATE_ERROR_POINT_TO_POINT_SIM3;
INSTANCIATE_ERROR_POINT_TO_PLANE;
INSTANCIATE_ERROR_POINT_TO_PLANE_SIM3;
INSTANCIATE_ICP;

//using namespace icp;
//#define INSTANCTIATE_FOR(Scalar, PointReference, PointSource, DegreesOfFreedom) \
//      template class ErrorPointToPoint<Scalar, PointReference, PointSource>; \
//      template class MaximumAbsoluteDeviation<Scalar, PointReference, PointSource>; \
//      template class MaximumAbsoluteDeviationVector<Scalar>; \
//      template class MEstimatorHubert<Scalar, PointReference, PointSource>; \
//      template class Constraints_<Scalar, DegreesOfFreedom>; \
//      template class JacobianConstraints<Scalar, DegreesOfFreedom>; \
//      template class Icp_<float, Eigen::Matrix<Scalar, DegreesOfFreedom, 1>, PointReference, PointSource, ErrorPointToPoint<Scalar, PointReference, PointSource>, MEstimatorHubert<Scalar, PointReference, PointSource>> IcpPointToPointHubert;
//
