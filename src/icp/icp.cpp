#include "icp.hpp"
#include "mestimator_hubert.hpp"
#include "error_point_to_point.hpp"
#include "error_point_to_plane.hpp"


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
  for (int i = 0; i < src->size(); i++) {
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
  /**
   * Notations:
   * - P_ref_: reference point cloud \f[ P^* \f]
   * - P_current_: \f[ P \f], current point cloud (CAO model, cloud extracted from one sensor
   * view....)
   * - xk: pose twist to be optimized \f[ \xi \f]
   * - T(xk): pose in SE3
   * - hat_T: previous pose
   **/



  /**
   * Initialization
   **/

  // Initialize the transformation twist to the initial guess
  Twist xk = param_.initial_guess;
  Eigen::Matrix<float, 4, 4> T;
  // Create transformation matrix from twist
  T = la::expLie(xk);

  // Contains the best current registration
  // XXX REFERENCE IS FIXED
  PcPtr P_current_transformed = PcPtr(new Pc());
  // Transforms the reference point cloud according to initial twist
  pcl::transformPointCloud(*P_current_, *P_current_transformed, T);


  /**
   * Nearest neighbor search in KD-Tree
   **/
  std::vector<int> indices_ref;
  std::vector<int> indices_current;
  std::vector<Dtype> distances;
  try {
    findNearestNeighbors(P_current_transformed, param_.max_correspondance_distance,
                         indices_ref, indices_current, distances);
    LOG(INFO) << "i_src: " << indices_ref.size() << ", i_target: " <<
              indices_current.size();
  } catch (...) {
    LOG(FATAL) <<
               "Could not find the nearest neighbors in the KD-Tree, impossible to run ICP without them!";
  }

  // Create a point cloud containing all points in current matching reference points
  PcPtr P_current_phi(new Pc());
  PrPtr P_ref_phi(new Pr());
  pcltools::subPointCloud<PointCurrent>(P_current_transformed, indices_ref, P_current_phi);
  pcltools::subPointCloud<PointReference>(P_ref_, indices_current, P_ref_phi);

  /**
   * Computing the initial error
   **/
  err_.setInputCurrent(P_current_phi);
  err_.setInputReference(P_ref_phi);
  // Initialize mestimator weights from point cloud
  //mestimator_.computeWeights(P_current_phi);
  // Weight every point according to the mestimator to avoid outliers
  //err_.setWeights(mestimator_.getWeights());
  err_.computeError();
  // Vector containing the error for each point
  // [ex_0, ey_0, ez_0, ... , ex_N, ey_N, ez_N]
  MatrixX e = err_.getErrorVector();
  // E is the global error
  Dtype E = e.norm();

  MatrixX J;
  MatrixX Jt;
  Twist x;

  // Cleanup
  r_.clear();

  unsigned int iter = 0;
  Dtype error_variation = 0;

  // Stopping condition. ICP will stop when one of two things
  // happens
  // - The error variation drops below a small threshold min_variation
  // - The number of iteration reaches the maximum max_iter allowed
  while ( iter == 0 || (error_variation >= 0 &&
                        error_variation > param_.min_variation
                        && iter < param_.max_iter) ) {
    DLOG(INFO) << "Iteration " << iter + 1 << "/" << param_.max_iter <<
               std::setprecision(8) << ", E=" << E <<
               ", error_variation=" << error_variation;
    ++iter;
    r_.registrationError.push_back(E);

    // Computes the Jacobian
    err_.computeJacobian();
    J = err_.getJacobian();
    //DLOG(INFO) << "Jacobian\n" << J;
    Jt = J.transpose();
    if (!e.allFinite()) LOG(FATAL) << "NaN value in e!";
    if (!J.allFinite()) LOG(FATAL) << "NaN value in J!";
    if (!Jt.allFinite()) LOG(FATAL) << "NaN value in Jt!";


    // Computes the Gauss-Newton update-step
    // XXX: Numerically unstable!
    //x = -param_.lambda * eigentools::pseudoInverse(J) * e;
    auto H = Jt * J;
    if (!H.allFinite()) LOG(FATAL) << "NaN value in H!";
    auto p_inv = H.ldlt();
    if (!e.allFinite()) LOG(FATAL) << "NaN value in e!";
    x = -param_.lambda * H.ldlt().solve(Jt * e);
    if (!x.allFinite()) LOG(FATAL) << "NaN value in x!";
    //LOG(INFO) << "\n" << x;
    //H = Eigen::Matrix<double,6,6>();
    //g = Eigen::Matrix<double,6,1>();
    //dx = H.ldlt ().solve (g);

    // Transforms the reference point cloud according to new twist
    Eigen::Matrix<Dtype, 4, 4> &T_prev = T;
    // hat_T = e^x * hat_T
    T = la::expLie(x) * T;

    pcl::transformPointCloud(*P_current_, *P_current_transformed, T);
    try {
      findNearestNeighbors(P_current_transformed, param_.max_correspondance_distance,
                           indices_ref, indices_current, distances);
      //LOG(INFO) << "i_src: " << indices_ref.size() << ", i_target: " <<
      //          indices_current.size();
    } catch (...) {
      LOG(WARNING) <<
                   "Could not find the nearest neighbors in the KD-Tree, impossible to run ICP without them!";
    }

    // Generate new current point cloud with only the matches in it
    // XXX: Speed improvement possible by using the indices directly instead of
    // generating a new pointcloud. Maybe PCL has stuff to do it.
    pcltools::subPointCloud<PointCurrent>(P_current_transformed, indices_ref, P_current_phi);
    //pcl::io::savePCDFileASCII ("/tmp/test_target.pcd", *P_current_phi);
    pcltools::subPointCloud<PointReference>(P_ref_, indices_current, P_ref_phi);

    // Update the reference point cloud to use the previously estimated one
    err_.setInputReference(P_ref_phi);
    err_.setInputCurrent(P_current_phi);


    // Updating the mestimator would only be needed if the source point cloud
    // wasn't rigid, which is the case
    //mestimator_.computeWeights(P_current_phi);
    //err_.setWeights(mestimator_.getWeights());

    // Computes the error for next iteration
    err_.computeError();
    e = err_.getErrorVector();
    //DLOG(INFO) << "Error\n" << e;
    Dtype E_new = e.norm();
    if (std::isinf(E_new)) {
      LOG(WARNING) << "Error is infinite!";
    }
    // Check the amount of error deviation to determine when to stop
    error_variation = E - E_new;
    E = E_new;

    if (error_variation < 0) {
      T = T_prev;
      pcl::transformPointCloud(*P_current_, *P_current_transformed, T);
    } else {
      r_.registrationError.push_back(E);
    }
  }

  r_.registeredPointCloud = PcPtr(new Pc(*P_current_transformed));
  r_.transformation = T;
  r_.scale = Sophus::Sim3f(T).scale();
}



// Explicit template instantiation
template class
Icp_<float, Eigen::Matrix<float, 6, 1>, pcl::PointXYZ, pcl::PointXYZ, ErrorPointToPointXYZ, MEstimatorHubertXYZ>;
template class
Icp_<float, Eigen::Matrix<float, 6, 1>, pcl::PointXYZRGB, pcl::PointXYZRGB, ErrorPointToPointXYZRGB, MEstimatorHubertXYZRGB>;
template class
Icp_<float, Eigen::Matrix<float, 6, 1>, pcl::PointNormal, pcl::PointNormal, ErrorPointToPlaneNormal, MEstimatorHubertNormal>;
// Sim3
template class
Icp_<float, Eigen::Matrix<float, 7, 1>, pcl::PointXYZ, pcl::PointXYZ, ErrorPointToPointXYZSim3, MEstimatorHubertXYZ>;


}  // namespace icp
