#include <cmath>
#include "icp.hpp"
#include "mestimator_hubert.hpp"
#include "error_point_to_point.hpp"
#include "error_point_to_plane.hpp"
#include "instanciate.hpp"
#include "logging.hpp"
#include "linear_algebra.hpp"


namespace icp {


template<typename Dtype, typename Twist, typename PointReference, typename PointCurrent, typename Error_, typename MEstimator>
void Icp_<Dtype, Twist, PointReference, PointCurrent, Error_, MEstimator>::initialize(const PcPtr &current,
    const PrPtr &reference,
    const IcpParameters &param) {
  setInputCurrent(current);
  setInputReference(reference);
  param_ = param;
}

template<typename Dtype, typename Twist, typename PointReference, typename PointCurrent, typename Error_, typename MEstimator>
void Icp_<Dtype, Twist, PointReference, PointCurrent, Error_, MEstimator>::findNearestNeighbors(const PcPtr &src,
    Dtype max_correspondance_distance,
    std::vector<int> &indices_ref,
    std::vector<int> &indices_current,
    std::vector<Dtype> &distances) {
  // We're only interrested in the nearest point
  const int K = 1;
  indices_ref.clear();
  indices_current.clear();
  indices_ref.reserve(src->size());
  indices_current.reserve(src->size());
  distances.clear();
  distances.reserve(src->size());
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<Dtype> pointNKNSquaredDistance(K);

  PointCurrent pt;
  for (unsigned int i = 0; i < src->size(); i++) {
    // Copy only coordinates from the point (for genericity)
    pt.x = src->at(i).x;
    pt.y = src->at(i).y;
    pt.z = src->at(i).z;

    // Look for the nearest neighbor
    if ( kdtree_.nearestKSearch(pt, K, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0 ) {
      Dtype distance = pointNKNSquaredDistance[0];
      if (distance <= max_correspondance_distance) {
        indices_ref.push_back(i);
        indices_current.push_back(pointIdxNKNSearch[0]);
        distances.push_back(distance);
      } else {
        //LOG(INFO) << "Ignoring, distance too big << " << distance;
      }
    } else {
      LOG(WARNING) << "Could not find a nearest neighbor for point " << i;
    }
  }
}

template<typename Dtype, typename Twist, typename PointReference, typename PointCurrent, typename Error_, typename MEstimator>
void Icp_<Dtype, Twist, PointReference, PointCurrent, Error_, MEstimator>::run() {
  if (P_current_->size() == 0 && P_ref_->size() == 0) {
    LOG(ERROR) << "Error: ICP can't run on empty pointclouds!";
    r_.has_converged = false;
    return;
  }

  // Cleanup
  r_.clear();
  iter_ = 0;
  boost::optional<Dtype> error_variation;

  // Stopping condition. ICP will stop when one of two things
  // happens
  // - The error variation drops below a small threshold min_variation
  // - The number of iteration reaches the maximum max_iter allowed
  while (step() && (!error_variation || (error_variation && *error_variation < 0 &&
                                         -*error_variation > param_.min_variation &&
                                         iter_ <= param_.max_iter)))  {
    error_variation = r_.getLastErrorVariation();

    if (error_variation) {
      LOG(INFO) << "Iteration " << iter_ << "/" << param_.max_iter <<
                std::setprecision(8) << ", E=" << r_.getLastError() <<
                ", error_variation=" << *error_variation;
    } else {
      LOG(INFO) << "Iteration " << iter_ << "/" << param_.max_iter <<
                std::setprecision(8) << ", E=" << r_.getLastError() <<
                ", error_variation=none";
    }
  }
  r_.has_converged = (iter_ <= param_.max_iter);
}

template<typename Dtype, typename Twist, typename PointReference, typename PointCurrent, typename Error_, typename MEstimator>
bool Icp_<Dtype, Twist, PointReference, PointCurrent, Error_, MEstimator>::step() {
  /**
   * Notations:
   * - P_ref_: reference point cloud \f[ P^* \f]
   * - P_current_: \f[ P \f], current point cloud (CAO model, cloud extracted from one sensor
   * view....)
   * - xk: pose twist to be optimized \f[ \xi \f]
   * - T_(xk): pose in SE3
   * - hat_T: previous pose
   **/

  ++iter_;

  std::vector<int> indices_ref;
  std::vector<int> indices_current;
  std::vector<Dtype> distances;
  PcPtr P_current_transformed(new Pc());
  PcPtr P_current_phi(new Pc());
  PcPtr P_ref_phi(new Pc());

  pcl::transformPointCloud(*P_current_, *P_current_transformed, T_);
  try {
    findNearestNeighbors(P_current_transformed, param_.max_correspondance_distance,
                         indices_ref, indices_current, distances);
  } catch (...) {
    LOG(WARNING) << "Could not find the nearest neighbors in the KD-Tree, impossible to run ICP without them!";
    return false;
  }

  // Generate new current point cloud with only the matches in it
  // XXX: Speed improvement possible by using the indices directly instead of
  // generating a new pointcloud. Maybe PCL has stuff to do it.
  pcltools::subPointCloud<PointCurrent>(P_current_transformed, indices_ref, P_current_phi);
  pcltools::subPointCloud<PointReference>(P_ref_, indices_current, P_ref_phi);

  // Update the reference point cloud to use the previously estimated one
  err_.setInputReference(P_ref_phi);
  err_.setInputCurrent(P_current_phi);
  // Computes the Jacobian
  err_.computeJacobian();

  // Initialize mestimator weights from point cloud
  mestimator_.setReferenceCloud(P_ref_phi, T_);
  mestimator_.computeWeights();
  err_.setWeights(mestimator_.getWeights());

  // Computes the error
  err_.computeError();

  // Transforms the reference point cloud according to new twist
  // Computes the Gauss-Newton update-step
  T_ = err_.update() * T_;

  Dtype E = err_.getErrorNorm();
  r_.registrationError.push_back(E);
  r_.transformation = T_;
  r_.relativeTransformation = param_.initial_guess.inverse() * T_;
  r_.scale = Sophus::Sim3f(T_).scale();
  if (std::isinf(E)) {
    LOG(WARNING) << "Error is infinite!";
  }
  return true;
}


INSTANCIATE_ICP;

}  // namespace icp
