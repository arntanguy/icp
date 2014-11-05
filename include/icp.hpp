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

/**
 * @brief Optimisation parameters for ICP
 */
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
    initial_guess = Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic>::Zero(6,
                    1);
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



/**
 * @brief Results for the ICP
 */
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

  //! Transformation (SE3) of the final registration transformation
  Eigen::Matrix<Dtype, 4, 4> transformation;

  Dtype getFinalError() const {
    return registrationError[registrationError.size() - 1];
  }

  void clear() {
    registrationError.clear();
    transformation = Eigen::Matrix<Dtype, 4, 4>::Zero(
                          4, 4);
  }
};

typedef IcpResults_<float> IcpResultsf;
typedef IcpResults_<double> IcpResultsd;

template<typename Dtype>
std::ostream &operator<<(std::ostream &s, const IcpResults_<Dtype> &r) {
  if (!r.registrationError.empty()) {
    s << "Initial error: " << r.registrationError[0]
      << "\nFinal error: " << r.registrationError[r.registrationError.size() - 1]
      << "\nFinal transformation: \n"
      << r.transformation 
      << "\nError history: ";
    for (int i = 0; i < r.registrationError.size(); ++i) {
      s << r.registrationError[i]  << ", ";
    }
  } else {
    s << "Icp: No Results!";
  }
  return s;
}

/**
 * @brief Iterative Closest Point Algorithm
 */
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
                    const IcpParameters &param);


    void findNearestNeighbors(const Pc::Ptr &src, std::vector<int> &indices,
                              std::vector<Dtype> &distances);

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
     * \brief Runs the ICP algorithm with given parameters.
     *
     * Runs the ICP according to the templated \c MEstimator and \c Error function,
     * and optimisation parameters \c IcpParameters_
     *
     * \retval void You can get a structure containing the results of the ICP (error, registered point cloud...)
     * by using \c getResults()
    **/
    void run();

    /**
     * @brief Sets the parameters for the optimisation.
     *
     * All parameters are defined within the \c IcpParameters_ structure.
     *
     * @param param
     *  Parameters to the minimisation
     */
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
    /**
     * @brief Gets the result of the ICP.
     *
     *
     * @return
     * Results of the ICP (call \c run() to run the ICP and generate results)
     */
    IcpResults getResults() const {
      return r_;
    }
};

}  // namespace icp

#endif /* ICP_H */
