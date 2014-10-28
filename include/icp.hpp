//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.


#ifndef ICP_H
#define ICP_H

#include <Eigen/Dense>

#include <glog/logging.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "linear_algebra.hpp" 
#include "eigentools.hpp"

#include <fstream>

namespace icp {

template<typename Dtype>
struct IcpParameters_ {
  //! Rate of convergence
  Dtype lambda;
  //! Maximum number of allowed iterations
  int max_iter;
  //! Stopping condition
  /*! ICP stops when the error variation between two iteration is under
    min_variation. */
  Dtype min_variation;
  //! Twist representing the initial guess for the registration
  Eigen::Matrix<Dtype, 6, 1> initial_guess;

  IcpParameters_() : lambda(0.01), max_iter(100), min_variation(10e-5) {
    initial_guess = Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic>::Zero(6, 1);
  }
};

typedef IcpParameters_<float> IcpParametersf;
typedef IcpParameters_<double> IcpParametersd;

template<typename Dtype>
std::ostream &operator<<(std::ostream &s, const IcpParameters_<Dtype> &p) {
  s << "Lambda: "  << p.lambda
    << "\nMax iterations: " << p.max_iter
    << "\nMin variation: " << p.min_variation
    << "\nInitial guess (twist):\n" << p.initial_guess;
  return s;
}



template<typename Dtype>
struct IcpResults_ {
  typedef pcl::PointCloud<pcl::PointXYZ> Pc;

  //! Point cloud of the registered points
  Pc::Ptr registeredPointCloud;

  //! History of previous registration errors
  /*!
    - First value is the initial error before ICP,
    - Last value is the final error after ICP. */
  std::vector<Dtype> registrationError;

  //! Twist of the final registration transformation
  Eigen::Matrix<Dtype, 6, 1> registrationTwist;

  void clear() {
    registrationError.clear();
    registrationTwist = Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic>::Zero(
                          6, 1);
  }
};

typedef IcpResults_<float> IcpResultsf;
typedef IcpResults_<double> IcpResultsd;

template<typename Dtype>
std::ostream &operator<<(std::ostream &s, const IcpResults_<Dtype> &r) {
  if (!r.registrationError.empty()) {
    s << "Initial error: " << r.registrationError[0]
      << "\nFinal error: " << r.registrationError[r.registrationError.size() - 1]
      << "\nBest twist: \n" << r.registrationTwist
      << "\nFinal transformation: \n"
      << la::expSE3(r.registrationTwist)
      << "\nError history: ";
    for (int i = 0; i < r.registrationError.size(); ++i) {
      s << r.registrationError[i]  << ", ";
    }
  } else {
    s << "Icp: No Results!";
  }
  return s;
}

template<typename Dtype, typename Error, typename MEstimator>
class Icp {
  public:
    typedef pcl::PointCloud<pcl::PointXYZ> Pc;
    typedef IcpParameters_<Dtype> IcpParameters;
    typedef IcpResults_<Dtype> IcpResults;
    typedef typename Eigen::Matrix<Dtype, 6, 1> Vector6;

  protected:
    // Reference (model) point cloud. This is the fixed point cloud to be registered against.
    Pc::Ptr pc_m_;
    // kd-tree of the model point cloud
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
    // Data point cloud. This is the one needing registration
    Pc::Ptr pc_d_;

    // Instance of an error kernel used to compute the error vector, Jacobian...
    Error err_;
    // MEstimator instance, used to improve statistical robusteness against outliers.
    MEstimator mestimator_;

    // Parameters of the algorithm (rate of convergence, stopping condition...)
    IcpParameters param_;

    // Results of the ICP
    IcpResults r_;

  protected:
    void initialize(const Pc::Ptr &model, const Pc::Ptr &data,
                    const IcpParameters &param) {
      setModelPointCloud(model);
      setDataPointCloud(data);
      param_ = param;
    }

    void findNearestNeighbors(const Pc::Ptr &src, std::vector<int> &indices,
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

    void subPointCloud(const Pc::Ptr &src, const std::vector<int> &indices,
                       Pc::Ptr &dst) {
      dst->clear();
      dst->reserve(indices.size());
      for (int index : indices) {
        dst->push_back((*src)[index]);
      }
    }

  public:
    Icp() {
    }
    Icp(const Pc::Ptr &model, const Pc::Ptr &data) : pc_m_(model), pc_d_(data) {
      initialize(model, data, IcpParameters());
    }
    Icp(const Pc::Ptr &model, const Pc::Ptr &data, const IcpParameters &param) {
      initialize(model, data, param);
    }

    /**
       \brief Runs the ICP algorithm with given parameters.
       \retval void You can get a structure containing the results of the ICP by using getResults
    **/

    void run() {
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
        DLOG(INFO) << "Iteration " << iter+1 << "/" << param_.max_iter << 
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
        if(std::isinf(E_new) || E_new > 400) {
          LOG(INFO) << "Error is infinite!";
          LOG(INFO) << "pc_d_";
          for(auto p : *pc_d_) {
            LOG(INFO) << p;
          }
          LOG(INFO) << "pc_r_";
          for(auto p : *pc_r) {
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

    void setParameters(const IcpParameters &param) {
      param_ = param;
    }

    IcpParameters getParameters() const {
      return param_;
    }
    void setModelPointCloud(const Pc::Ptr &pc) {
      pc_m_ = pc;
      kdtree_.setInputCloud(pc_m_);
    }
    void setDataPointCloud(const Pc::Ptr &pc) {
      pc_d_ = pc;
    }
    IcpResults getResults() const {
      return r_;
    }
};

}  // namespace icp

#endif /* ICP_H */
