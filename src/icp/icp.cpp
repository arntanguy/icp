#include "icp.hpp"
#include "mestimator_hubert.hpp"
#include "error_point_to_point.hpp"


namespace icp {


template<typename Dtype, typename Error, typename MEstimator>
void Icp<Dtype, Error, MEstimator>::initialize(const Pc::Ptr &model,
    const Pc::Ptr &data,
    const IcpParameters &param) {
  setInputTarget(model);
  setInputSource(data);
  param_ = param;
}

template<typename Dtype, typename Error, typename MEstimator>
void Icp<Dtype, Error, MEstimator>::findNearestNeighbors(const Pc::Ptr &src,
    Dtype max_correspondance_distance,
    std::vector<int> &indices_src,
    std::vector<int> &indices_target,
    std::vector<Dtype> &distances) {
  // We're only interrested in the nearest point
  const int K = 1;
  indices_src.clear();
  indices_target.clear();
  indices_src.reserve(src->size());
  indices_target.reserve(src->size());
  distances.clear();
  distances.reserve(src->size());
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<Dtype> pointNKNSquaredDistance(K);

  for (int i = 0; i < src->size(); i++) {
    auto &pt = src->at(i);
    // Look for the nearest neighbor
    if ( kdtree_.nearestKSearch(pt, K, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0 ) {
      Dtype distance = pointNKNSquaredDistance[0];
      if (distance <= max_correspondance_distance) {
        indices_src.push_back(i);
        indices_target.push_back(pointIdxNKNSearch[0]);
        distances.push_back(distance);
      } else {
        //LOG(INFO) << "Ignoring, distance too big << " << distance;
      }
    } else {
      LOG(WARNING) << "Could not find a nearest neighbor for point " << i;
    }
  }
}

template<typename Dtype, typename Error, typename MEstimator>
void Icp<Dtype, Error, MEstimator>::run() {
  /**
   * Initialization
   **/

  // Initialize the transformation twist to the initial guess
  Vector6 xk = param_.initial_guess;
  // Create transformation matrix from twist
  Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic> T =
    la::expSE3(xk);


  // Contains the best current registration
  Pc::Ptr source_current = Pc::Ptr(new Pc());
  // Transforms the data point cloud according to initial twist
  pcl::transformPointCloud(*source_, *source_current, T);


  /**
   * Nearest neighbor search in KD-Tree
   **/
  std::vector<int> indices_src;
  std::vector<int> indices_target;
  std::vector<Dtype> distances;
  try {
    findNearestNeighbors(source_current, param_.max_correspondance_distance,
                         indices_src, indices_target, distances);
    LOG(INFO) << "i_src: " << indices_src.size() << ", i_target: " <<
              indices_target.size();
  } catch (...) {
    LOG(FATAL) <<
               "Could not find the nearest neighbors in the KD-Tree, impossible to run ICP without them!";
  }

  // Create a point cloud containing all points in model matching data points
  Pc::Ptr target_phi(new Pc());
  Pc::Ptr source_current_phi(new Pc());
  pcltools::subPointCloud<pcl::PointXYZ>(source_current, indices_src,
                                         source_current_phi);
  pcltools::subPointCloud<pcl::PointXYZ>(target_, indices_target, target_phi);

  /**
   * Computing the initial error
   **/
  err_.setInputTarget(target_phi);
  err_.setInputSource(source_current_phi);
  // Initialize mestimator weights from point cloud
  //mestimator_.computeWeights(source_current_phi);
  // Weight every point according to the mestimator to avoid outliers
  //err_.setWeights(mestimator_.getWeights());
  err_.computeError();
  // Vector containing the error for each point
  // [ex_0, ey_0, ez_0, ... , ex_N, ey_N, ez_N]
  Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic> e = err_.getErrorVector();
  // E is the global error
  Dtype E = e.norm();

  Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic> J;
  Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic> Jt;
  Vector6 x;

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

    // Transforms the data point cloud according to new twist
    T = la::expSE3(x) * T;

    pcl::transformPointCloud(*source_, *source_current, T);
    try {
      findNearestNeighbors(source_current, param_.max_correspondance_distance,
                           indices_src, indices_target, distances);
      //LOG(INFO) << "i_src: " << indices_src.size() << ", i_target: " <<
      //          indices_target.size();
    } catch (...) {
      LOG(WARNING) <<
                 "Could not find the nearest neighbors in the KD-Tree, impossible to run ICP without them!";
    }

    // Generate new model point cloud with only the matches in it
    // XXX: Speed improvement possible by using the indices directly instead of
    // generating a new pointcloud. Maybe PCL has stuff to do it.
    pcltools::subPointCloud<pcl::PointXYZ>(target_, indices_target, target_phi);
    pcltools::subPointCloud<pcl::PointXYZ>(source_current, indices_src,
                                           source_current_phi);

    // Update the data point cloud to use the previously estimated one
    err_.setInputSource(source_current_phi);
    err_.setInputTarget(target_phi);


    // Updating the mestimator would only be needed if the source point cloud
    // wasn't rigid, which is the case
    //mestimator_.computeWeights(source_current_phi);
    //err_.setWeights(mestimator_.getWeights());

    // Computes the error for next iteration
    err_.computeError();
    e = err_.getErrorVector();
    Dtype E_new = e.norm();
    if (std::isinf(E_new)) {
      LOG(WARNING) << "Error is infinite!";
    }
    // Check the amount of error deviation to determine when to stop
    error_variation = E - E_new;
    E = E_new;

    r_.registrationError.push_back(E);
  }

  r_.registeredPointCloud = Pc::Ptr(new Pc(*source_current));
  r_.transformation = T;

}



// Explicit template instantiation
template class Icp<float, ErrorPointToPoint<float>, MEstimatorHubert<float>>;

}  // namespace icp
