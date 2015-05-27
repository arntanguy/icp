//  This file is part of the Icp_ Library,
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

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "result.hpp"
#include "error_point_to_point.hpp"
#include "error_point_to_point_sim3.hpp"
#include "error_point_to_plane.hpp"
#include "error_point_to_plane_sim3.hpp"
#include "error_point_to_point_so3.hpp"
#include "error_point_to_plane_so3.hpp"
#include "mestimator_hubert.hpp"

#include <fstream>

#define DEFINE_ICP_TYPES(Scalar, Suffix) \
  typedef Icp_<Scalar, pcl::PointXYZ, pcl::PointXYZ, ErrorPointToPointXYZ, MEstimatorHubertXYZ> IcpPointToPointHubert##Suffix; \
  typedef Icp_<Scalar, pcl::PointXYZ, pcl::PointXYZ, ErrorPointToPointSO3XYZ, MEstimatorHubertXYZ> IcpPointToPointHubertSO3##Suffix; \
  typedef Icp_<Scalar, pcl::PointXYZRGB, pcl::PointXYZRGB, ErrorPointToPointXYZRGB, MEstimatorHubertXYZRGB> IcpPointToPointHubertXYZRGB##Suffix; \
  typedef Icp_<Scalar, pcl::PointXYZ, pcl::PointXYZ, ErrorPointToPointXYZSim3, MEstimatorHubertXYZ> IcpPointToPointHubertSim3##Suffix; \
  typedef Icp_<Scalar, pcl::PointXYZRGB, pcl::PointXYZRGB, ErrorPointToPointXYZRGBSim3, MEstimatorHubertXYZRGB> IcpPointToPointHubertXYZRGBSim3##Suffix; \
  typedef Icp_<Scalar, pcl::PointNormal, pcl::PointNormal, ErrorPointToPlaneNormal, MEstimatorHubertNormal> IcpPointToPlaneHubert##Suffix; \
  typedef Icp_<Scalar, pcl::PointNormal, pcl::PointNormal, ErrorPointToPlaneSim3Normal, MEstimatorHubertNormal> IcpPointToPlaneHubertSim3##Suffix; \
  typedef IcpParameters_<Scalar> IcpParameters##Suffix;


namespace icp {

/**
 * @brief Optimisation parameters for ICP
 */
template<typename Dtype>
struct IcpParameters_ {
  //! Maximum number of allowed iterations
  unsigned int max_iter;
  //! Stopping condition
  /*! ICP stops when the error variation between two iteration is under
    min_variation.
    TODO: Add better convergence criteria */
  Dtype min_variation;
  //! Maximum search distance for correspondances
  /*! Do not look further than this for the kdtree search */
  Dtype max_correspondance_distance;

  //! Initial guess for the registration
  Eigen::MatrixXf initial_guess;

  //! Use MEstimators?
  bool mestimator;

  IcpParameters_() : max_iter(10), min_variation(10e-5),
    max_correspondance_distance(std::numeric_limits<Dtype>::max()), mestimator(false) {
    initial_guess = Eigen::Matrix<Dtype, 4, 4>::Identity();
  }
};

template<typename Dtype>
std::ostream &operator<<(std::ostream &s, const IcpParameters_<Dtype> &p) {
  s << "\nMax iterations: " << p.max_iter
    << "\nMin variation: " << p.min_variation
    << "\nInitial guess (twist):\n" << p.initial_guess;
  return s;
}



/**
 * @brief Iterative Closest Point Algorithm
 */
template<typename Dtype, typename PointReference, typename PointCurrent, typename Error_, typename MEstimator>
class Icp_ {
  public:
    typedef typename pcl::PointCloud<PointReference> Pr;
    typedef typename pcl::PointCloud<PointCurrent> Pc;
    typedef typename Pc::Ptr PcPtr;
    typedef typename Pr::Ptr PrPtr;
    typedef IcpParameters_<Dtype> IcpParameters;
    typedef IcpResults_<Dtype> IcpResults;

    typedef typename Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

  protected:
    // Reference (model) point cloud. This is the cloud that we want to register
    PcPtr P_current_;
    // kd-tree of the model point cloud
    pcl::KdTreeFLANN<PointReference> kdtree_;
    // Reference cloud, upon which others will be registered
    PrPtr P_ref_;
    PrPtr P_ref_init_inv;

    // Instance of an error kernel used to compute the error vector, Jacobian...
    Error_ err_;
    // MEstimator instance, used to improve statistical robusteness against outliers.
    MEstimator mestimator_;

    // Parameters of the algorithm (rate of convergence, stopping condition...)
    IcpParameters param_;

    // Results of the ICP
    IcpResults r_;

    unsigned int iter_;
    Eigen::Matrix<Dtype, 4, 4> T_;

  protected:
    void initialize(const PcPtr &model, const PrPtr &data,
                    const IcpParameters &param);


    /**
     * @brief Finds the nearest neighbors between the current cloud (src) and the kdtree
     * (buit from the reference cloud)
     *
     * @param src
     *  The current cloud
     * @param max_correspondance_distance
     *  Max distance in which closest point has to be looked for (in meters)
     * @param indices_src
     * @param indices_target
     * @param distances
     */
    void findNearestNeighbors(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src,
                              const Dtype max_correspondance_distance,
                              std::vector<int> &indices_src,
                              std::vector<int> &indices_target,
                              std::vector<Dtype> &distances);

    void convergenceFailed() {
      r_.has_converged = false;
      r_.transformation = Eigen::Matrix<Dtype, 4, 4>::Identity();
      r_.relativeTransformation = Eigen::Matrix<Dtype, 4, 4>::Identity();
    }

  public:
    Icp_() : P_current_(new Pc()), P_ref_(new Pr()), P_ref_init_inv(new Pr()), T_(Eigen::Matrix<Dtype, 4, 4>::Identity()) {
    }

    /**
     * \brief Runs the ICP algorithm with given parameters.
     *
     * Runs the ICP according to the templated \c MEstimator and \c Error_ function,
     * and optimisation parameters \c IcpParameters_
     *
     * \retval void You can get a structure containing the results of the ICP (error, registered point cloud...)
     * by using \c getResults()
    **/
    void run();

    /**
     * @brief Run the next iteration of the ICP optimization
     */
    bool step();

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
      //T_ = param_.initial_guess;
      // XXX Suboptimal, to fix problem with initial transformation
      if (P_ref_->size() != 0) {
        Eigen::Matrix<Dtype, 4, 4> init_T_inv = param_.initial_guess.inverse();
        pcl::transformPointCloud(*P_ref_, *P_ref_init_inv, init_T_inv);
        kdtree_.setInputCloud(P_ref_init_inv);
      }
    }

    IcpParameters getParameters() const {
      return param_;
    }
    /** \brief Provide a pointer to the input target (e.g., the point cloud that we want to align).
    * \param[in] cloud the input point cloud target
    */
    void setInputCurrent(const PcPtr &in) {
      if (in->size() == 0) {
        LOG(WARNING) << "You are using an empty source cloud!";
      }
      P_current_ = in;
      mestimator_.setModelCloud(P_current_);
    }
    /**
     * @brief Provide a pointer to the input source (e.g., the target pointcloud
     * that we want to align to)
     *
     * @param[in] cloud	the reference point cloud source
     */
    void setInputReference(const PrPtr &in) {
      if (in->size() == 0) {
        LOG(WARNING) << "You are using an empty reference cloud!";
      }
      if (in->size() != 0) {
        P_ref_ = in;
        Eigen::Matrix<Dtype, 4, 4> init_T_inv = param_.initial_guess.inverse();
        pcl::transformPointCloud(*in, *P_ref_init_inv, init_T_inv);
        kdtree_.setInputCloud(P_ref_init_inv);
      }
      mestimator_.setReferenceCloud(P_ref_init_inv, param_.initial_guess);
    }

    void setError(Error_ err) {
      err_ = err;
    }
    /**
     * @brief Gets the result of the ICP.
     *
     * @return
     * Results of the ICP (call \c run() to run the ICP and generate results)
     */
    IcpResults getResults() const {
      return r_;
    }

    void createMEstimatorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr dstCloud) {
      if (param_.mestimator)
        mestimator_.createWeightColoredCloud(dstCloud);
    }
    void createMEstimatorCloudIntensity(pcl::PointCloud<pcl::PointXYZRGB>::Ptr dstCloud) {
      if (param_.mestimator)
        mestimator_.createWeightColoredCloudIntensity(dstCloud);
    }
};

DEFINE_ICP_TYPES(float, f);
DEFINE_ICP_TYPES(float, );


}  // namespace icp

#endif /* ICP_H */
