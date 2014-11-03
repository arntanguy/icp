#include "icp.hpp"
#include "mestimator_hubert.hpp"
#include "errorPointToPoint.hpp"


namespace icp {


template<typename Dtype, typename Error, typename MEstimator>
void Icp<Dtype, Error, MEstimator>::initialize(const Pc::Ptr &model,
    const Pc::Ptr &data,
    const IcpParameters &param) {
  setModelPointCloud(model);
  setDataPointCloud(data);
  param_ = param;
}

template<typename Dtype, typename Error, typename MEstimator>
void Icp<Dtype, Error, MEstimator>::findNearestNeighbors(const Pc::Ptr &src,
    std::vector<int> &indices,
    std::vector<Dtype> &distances) {
  // We're only interrested in the nearest point
  const int K = 1;
  indices.clear();
  indices.resize(src->size());
  distances.clear();
  distances.resize(src->size());
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<Dtype> pointNKNSquaredDistance(K);

  int i = 0;
  for (auto & pt : *src) {
    // Look for the nearest neighbor
    if ( kdtree_.nearestKSearch(pt, 1, pointIdxNKNSearch,
                                pointNKNSquaredDistance) > 0 ) {
      indices[i] =  pointIdxNKNSearch[0];
      distances[i] =  pointNKNSquaredDistance[0];
      i++;
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



  //LOG(INFO) << "T: " << T;
  // Contains the best current registration
  Pc::Ptr pc_r = Pc::Ptr(new Pc());
  // Transforms the data point cloud according to initial twist
  pcl::transformPointCloud(*pc_d_, *pc_r, T);
  //LOG(INFO) << "pc_d_";
  //for(auto p : *pc_d_) {
  //  LOG(INFO) << p;
  //}
  //LOG(INFO) << "pc_r_";
  //for(auto p : *pc_r) {
  //  LOG(INFO) << p;
  //}


  /**
   * Nearest neighbor search in KD-Tree
   **/
  std::vector<int> indices;
  std::vector<Dtype> distances;
  //DLOG(INFO) <<
  //          "Looking for nearest neighbors from data point cloud in model's kd-tree";
  try {
    findNearestNeighbors(pc_r, indices, distances);
  } catch (...) {
    LOG(FATAL) <<
               "Could not find the nearest neighbors in the KD-Tree, impossible to run ICP without them!";
  }
  //DLOG(INFO) << "Found nearest neighbors for " << indices.size() << " points";

  // Create a point cloud containing all points in model matching data points
  Pc::Ptr pc_m_phi(new Pc());
  subPointCloud(pc_m_, indices, pc_m_phi);
  //DLOG(INFO) << "Corresponding model point cloud has " << pc_m_phi->size() <<
  //          " points";

  /**
   * Computing the initial error
   **/
  err_.setModelPointCloud(pc_m_phi);
  err_.setDataPointCloud(pc_r);
  err_.computeError();
  // Vector containing the error for each point
  // [ex_0, ey_0, ez_0, ... , ex_N, ey_N, ez_N]
  Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic> e = err_.getErrorVector();
  // E is the global error
  Dtype E = e.norm();

  Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic> J;
  Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic> Jt;
  Vector6 x;

  /*
   * Cleanup
   **/
  r_.clear();

  unsigned int iter = 0;
  Dtype error_variation = 0;
  //
  // Stopping condition. ICP will stop when one of two things
  // happens
  // - The error variation drops below a small threshold min_variation
  // - The number of iteration reaches the maximum max_iter allowed
  while ( (iter == 0
           || (error_variation >= 0 && error_variation > param_.min_variation))
          && iter < param_.max_iter ) {
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

    xk = xk + x;
    //LOG(INFO) << "\nxk=\n" << xk << "\nx=\n" << x;



    // Transforms the data point cloud according to new twist
    T = la::expSE3(xk);
    pcl::transformPointCloud(*pc_d_, *pc_r, T);
    try {
      findNearestNeighbors(pc_r, indices, distances);
    } catch (...) {
      LOG(FATAL) <<
                 "Could not find the nearest neighbors in the KD-Tree, impossible to run ICP without them!";
    }

    // Update the data point cloud to use the previously estimated one
    err_.setDataPointCloud(pc_r);
    // Computes the error for next iteration
    err_.computeError();
    e = err_.getErrorVector();
    Dtype E_new = e.norm();
    if (std::isinf(E_new) || E_new > 400) {
      LOG(INFO) << "Error is infinite!";
      LOG(INFO) << "pc_d_";
      for (auto p : *pc_d_) {
        LOG(INFO) << p;
      }
      LOG(INFO) << "pc_r_";
      for (auto p : *pc_r) {
        LOG(INFO) << p;
      }
      LOG(INFO) << "T=\n" << T;
      LOG(INFO) << "update x=\n" << x;
      LOG(INFO) << "transform xk=\n" << xk;
    }
    // Check the amount of error deviation to determine when to stop
    error_variation = E - E_new;
    E = E_new;

    r_.registrationError.push_back(E);
  }

  r_.registeredPointCloud = Pc::Ptr(new Pc(*pc_r));
  r_.registrationTwist = xk;

}


template<typename Dtype, typename Error, typename MEstimator>
void Icp<Dtype, Error, MEstimator>::subPointCloud(const Pc::Ptr &src,
    const std::vector<int> &indices,
    Pc::Ptr &dst) {
  dst->clear();
  dst->reserve(indices.size());
  for (int index : indices) {
    dst->push_back((*src)[index]);
  }
}


// Explicit template instantiation
template class Icp<float, ErrorPointToPoint<float>, MEstimatorHubert<float>>;

}  // namespace icp
