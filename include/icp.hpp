#ifndef ICP_H
#define ICP_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sophus/se3.hpp>
#include <fstream>

namespace icp {

template<typename T>
struct IcpParameters_
{
  // Rate of convergence
  T lambda;
  // Maximum number of allowed iterations
  int max_iter;
  // ICP stops when the error variation between two iteration is under
  // min_variation.
  T min_variation;
  // Twist representing the initial guess for the registration
  typename Sophus::SE3Group<T>::Tangent initial_guess;

  IcpParameters_() : lambda(0.1), max_iter(100), min_variation(10e-5) {}
};
template<typename T>
std::ostream& operator<<(std::ostream& s, const IcpParameters_<T>& p) {
  s << "Lambda: "  << p.lambda
    << "\nMax iterations: " << p.max_iter
    << "\nMin variation: " << p.min_variation
    << "\nInitial guess (twist):\n" << p.initial_guess;
  return s;
}

typedef IcpParameters_<float> IcpParametersf;
typedef IcpParameters_<double> IcpParametersd;

template<typename T, typename Error, typename MEstimator>
class Icp
{
 protected:
  typedef pcl::PointCloud<pcl::PointXYZ> Pc;
  typedef IcpParameters_<T> IcpParameters;

  // Reference (model) point cloud. This is the fixed point cloud to be registered against.
  Pc::Ptr pc_m_;
  // Data point cloud. This is the one needing registration
  Pc::Ptr pc_d_;
  // Current best registration
  Pc::Ptr pc_r_;

  // History of previous registration errors
  // First value is the initial error before ICP,
  // Last value is the final error after ICP.
  std::vector<T> registration_error_;

  // Instance of an error kernel used to compute the error vector, Jacobian...
  Error err_;
  // MEstimator instance, used to improve statistical robusteness against outliers.
  MEstimator mestimator_;

  // Parameters of the algorithm (rate of convergence, stopping condition...)
  IcpParameters param_;


 protected:
  void initialize(const Pc::Ptr& model, const Pc::Ptr& data, const IcpParameters& param) {
    pc_m_ = model;
    pc_d_ = data;
    pc_r_ = data;
    param_ = param;
  }
 public:
  Icp() {
  }
  Icp(const Pc::Ptr& model, const Pc::Ptr& data) : pc_m_(model), pc_d_(data) {
    initialize(model, data, IcpParameters());
  }
  Icp(const Pc::Ptr& model, const Pc::Ptr& data, const IcpParameters& param) {
    initialize(model, data, param);
  }

  void run() {
    LOG(ERROR) << "ICP Algorithm not implemented yet!";
  }

  void setParameters(const IcpParameters& param) {
    param_ = param;
  }

  IcpParameters getParameters() const {
    return param_;
  }
  void setModelPointModelCloud(const Pc::Ptr& pc) {
    pc_m_ = pc;
  }
  void setDataPointCloud(const Pc::Ptr& pc) {
    pc_d_ = pc;
  }
};

}  // namespace icp

#endif /* ICP_H */
