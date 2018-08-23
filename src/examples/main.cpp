//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#include <boost/shared_ptr.hpp>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <sstream>
#include "eigentools.hpp"
#include "icp.hpp"
#include "error_point_to_point.hpp"
#include "error_point_to_plane.hpp"
#include "mestimator_hubert.hpp"
#include "logging.hpp"

typedef enum { POINT_TO_POINT, POINT_TO_PLANE, POINT_TO_POINT_SIM3, POINT_TO_PLANE_SIM3, POINT_TO_POINT_CONSTRAIN_X} Type;
Type method = POINT_TO_POINT;      // ok
//Type method = POINT_TO_PLANE;      // wrong
//Type method = POINT_TO_POINT_SIM3; // ok
//Type method = POINT_TO_PLANE_SIM3;
//Type method = POINT_TO_PLANE_SIM3;
//Type method = POINT_TO_POINT_CONSTRAIN_X;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

/**
 * @brief Point picker callback
 *
 * @param event
 */
void pp_callback (const pcl::visualization::PointPickingEvent &event);

icp::IcpResults icp_results;
int main(int argc, char *argv[]) {
  // Initialize Google's logging library.
#if GLOG_ENABLED
  google::InitGoogleLogging(argv[0]);
#endif

  std::cout << "Starting ICP program";

  /*
     Loads test point cloud
     */
  std::cout << "Loading Model pointcloud";
  PointCloudXYZ::Ptr modelCloud(new PointCloudXYZ());
  PointCloudXYZ::Ptr dataCloud(new PointCloudXYZ());
  //std::string model = "../models/ladder_robot/ladder_jr13_arnaud.pcd";
  std::string model = "../models/valve.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (model.c_str(), *modelCloud) == -1) {
    LOG(FATAL) << "Could't read file " << model;
    return (-1);
  }
  std::cout << "Model Point cloud has " << modelCloud->points.size()
            << " points";
  std::string data = "../models/valve_simulation_clean.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (data.c_str(), *dataCloud) == -1) {
    LOG(FATAL) << "Could't read file " << model;
    return (-1);
  }
  std::cout << "Model Point cloud has " << dataCloud->points.size()
            << " points";


  ///*
  //   Creating a second transformed pointcloud
  //   */
  Eigen::Matrix4f transformation;
  //Eigen::Matrix4f transformation
  //  = eigentools::createTransformationMatrix(0.f,
  //      0.05f,
  //      0.f,
  //      static_cast<float>(M_PI) / 20.f,
  //      static_cast<float>(M_PI) / 20.f,
  //      static_cast<float>(M_PI) / 20.f);
  //std::cout << "Transformation:\n" << transformation;

  //// Executing the transformation
  //// Generates a data point cloud to be matched against the model
  //pcl::transformPointCloud(*modelCloud, *dataCloud, transformation);



  PointCloudXYZ::Ptr resultCloud(new PointCloudXYZ());
  PointCloudXYZRGB::Ptr mestimatorWeightsCloud(new PointCloudXYZRGB());

  /*
     Define parameters for the ICP
     */
  icp::IcpParameters icp_param;
  icp_param.max_iter = 20;
  icp_param.min_variation = 10e-5;
  icp_param.initial_guess = Eigen::Matrix4f::Identity();
  // Far
  //icp_param.initial_guess(0, 3) = 1.6;
  //icp_param.initial_guess(1, 3) = 0.6;
  //icp_param.initial_guess(2, 3) = 1.6;
  // Less far
  icp_param.initial_guess(0, 3) = 2;
  icp_param.initial_guess(1, 3) = 0.7;
  icp_param.initial_guess(2, 3) = 1;
  icp_param.initial_guess.block<3,3>(0,0) =
      Eigen::AngleAxisf(0.3, Eigen::Vector3f::UnitX()).matrix()
      * Eigen::AngleAxisf(0.3, Eigen::Vector3f::UnitY()).matrix()
      * Eigen::AngleAxisf(M_PI+0.3, Eigen::Vector3f::UnitZ()).matrix();
  // Almost registered
  //icp_param.initial_guess(0, 3) = 2.176;
  //icp_param.initial_guess(1, 3) = 0.868;
  //icp_param.initial_guess(2, 3) = 1;
  //

  // big valve close
  icp_param.initial_guess(0, 3) = 2.1;
  icp_param.initial_guess(1, 3) = 0.;
  icp_param.initial_guess(2, 3) = 1;
  icp_param.initial_guess.block<3,3>(0,0) =
      1.5 * Eigen::AngleAxisf(0.3, Eigen::Vector3f::UnitX()).matrix()
      * Eigen::AngleAxisf(0.3, Eigen::Vector3f::UnitY()).matrix()
      * Eigen::AngleAxisf(M_PI+0.3, Eigen::Vector3f::UnitZ()).matrix();
  std::cout << "ICP Parameters:\n" << icp_param;


  if (method == POINT_TO_POINT)
  {
    /**
     * Point to point
     **/
    icp::IcpPointToPointHubert icp_algorithm;
    icp_algorithm.setParameters(icp_param);
    icp_algorithm.setInputCurrent(modelCloud);
    icp_algorithm.setInputReference(dataCloud);
    icp_algorithm.run();

    icp_results = icp_algorithm.getResults();
    std::cout << "ICP Results:\n" << icp_results;
    pcl::transformPointCloud(*modelCloud, *resultCloud, icp_results.transformation);
    icp_algorithm.createMEstimatorCloud(mestimatorWeightsCloud);
  } else if (method == POINT_TO_POINT_SIM3) {
    /**
     * Point to point (sim3)
     **/
    /*
       Creating a second transformed pointcloud
       */
    // transformation(0, 0) *= 0.5;
    // transformation(1, 1) *= 0.5;
    // transformation(2, 2) *= 0.5;
    // std::cout << "Transformation:\n" << transformation;
    // // Generates a data point cloud to be matched against the model
    // pcl::transformPointCloud(*modelCloud, *dataCloud, transformation);

    icp::IcpPointToPointHubertSim3 icp_algorithm;
    icp_algorithm.setParameters(icp_param);
    icp_algorithm.setInputCurrent(modelCloud);
    icp_algorithm.setInputReference(dataCloud);
    icp_algorithm.run();

    icp_results = icp_algorithm.getResults();
    std::cout << "ICP Results:\n" << icp_results;
    pcl::transformPointCloud(*modelCloud, *resultCloud, icp_results.transformation);
  } else if (method == POINT_TO_PLANE_SIM3) {
    /*
       Creating a second transformed pointcloud
       */
    transformation(0, 0) *= 0.5;
    transformation(1, 1) *= 0.5;
    transformation(2, 2) *= 0.5;
    std::cout << "Transformation:\n" << transformation;
    // Generates a data point cloud to be matched against the model
    pcl::transformPointCloud(*modelCloud, *dataCloud, transformation);

    /**
     * Point to Plane
     **/
    std::cout << "Computing cloud normals";
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    // Use all neighbors in a sphere of radius 10m
    // WARNING: Wrong value of this parameter may result to having NaN normals in
    // case nearest neighbors aren't found!
    //ne.setRadiusSearch (10);
    ne.setKSearch(100);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new
        pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    ne.setInputCloud (modelCloud);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr mesh_normal_pc (new
        pcl::PointCloud<pcl::Normal>);
    // Compute the features
    ne.compute (*mesh_normal_pc);
    pcl::PointCloud<pcl::PointNormal>::Ptr mesh_pointnormal_pc(
      new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*modelCloud, *mesh_normal_pc, *mesh_pointnormal_pc);

    ne.setInputCloud (dataCloud);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr scene_normal_pc (new
        pcl::PointCloud<pcl::Normal>);
    // Compute the features
    ne.compute (*scene_normal_pc);
    pcl::PointCloud<pcl::PointNormal>::Ptr scene_pointnormal_pc(
      new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*dataCloud, *scene_normal_pc, *scene_pointnormal_pc);

    //// Point to plane ICP SiM3
    icp::IcpPointToPlaneHubertSim3 icp_algorithm;
    icp_algorithm.setParameters(icp_param);
    icp_algorithm.setInputCurrent(mesh_pointnormal_pc);
    icp_algorithm.setInputReference(scene_pointnormal_pc);
    icp_algorithm.run();

    icp_results = icp_algorithm.getResults();
    std::cout << "ICP Results:\n" << icp_results;
    pcl::transformPointCloud(*modelCloud, *resultCloud, icp_results.transformation);
  } else if (method == POINT_TO_PLANE) {
    std::cout << "Computing cloud normals";
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    // Use all neighbors in a sphere of radius 10m
    // WARNING: Wrong value of this parameter may result to having NaN normals in
    // case nearest neighbors aren't found!
    //ne.setRadiusSearch (10);
    ne.setKSearch(100);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    ne.setInputCloud (modelCloud);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr mesh_normal_pc (new
        pcl::PointCloud<pcl::Normal>);
    // Compute the features
    ne.compute (*mesh_normal_pc);
    pcl::PointCloud<pcl::PointNormal>::Ptr mesh_pointnormal_pc(
      new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*modelCloud, *mesh_normal_pc, *mesh_pointnormal_pc);

    ne.setInputCloud (dataCloud);
    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr scene_normal_pc (new
        pcl::PointCloud<pcl::Normal>);
    // Compute the features
    ne.compute (*scene_normal_pc);
    pcl::PointCloud<pcl::PointNormal>::Ptr scene_pointnormal_pc(
      new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*dataCloud, *scene_normal_pc, *scene_pointnormal_pc);

    // Point to plane ICP
    icp::IcpPointToPlaneHubert icp_algorithm;
    icp_algorithm.setParameters(icp_param);
    icp_algorithm.setInputCurrent(mesh_pointnormal_pc);
    icp_algorithm.setInputReference(scene_pointnormal_pc);
    icp_algorithm.run();

    icp_results = icp_algorithm.getResults();
    std::cout << "ICP Results:\n" << icp_results;
    pcl::transformPointCloud(*modelCloud, *resultCloud, icp_results.transformation);
  } else if (method == POINT_TO_POINT_CONSTRAIN_X) {
    std::cout << "Point to point under fixed X axis constraint";
    //Eigen::Matrix4f transformation
    //  = eigentools::createTransformationMatrix(30.f,
    //      10.0f,
    //      0.f,
    //      0.1f * static_cast<float>(M_PI),
    //      0.1f * static_cast<float>(M_PI),
    //      0.1f * static_cast<float>(M_PI));
    //std::cout << "Transformation:\n" << transformation;

    //// Generates a data point cloud to be matched against the model
    //pcl::transformPointCloud(*modelCloud, *dataCloud, transformation);


    /**
     * Point to point
     **/
    boost::shared_ptr<icp::Constraints6> c(new icp::Constraints6());
    icp::FixTranslationConstraint tc;
    tc.setFixedAxes(false, true, false);
    c->setTranslationConstraint(tc);

    icp::ErrorPointToPointXYZ err;
    err.setConstraints(c);
    icp::IcpPointToPointHubert icp_algorithm;
    icp_algorithm.setError(err);
    icp_algorithm.setParameters(icp_param);
    icp_algorithm.setInputCurrent(modelCloud);
    icp_algorithm.setInputReference(dataCloud);
    icp_algorithm.run();

    icp_results = icp_algorithm.getResults();
    std::cout << "ICP Results:\n" << icp_results;
    pcl::transformPointCloud(*modelCloud, *resultCloud, icp_results.transformation);
  }


  /**
   * Visualize
   **/
  std::cout << "\nPoint cloud colors :  white  = original \n"
            "                           red    = transformed\n"
            "                           green  = registered";
  pcl::visualization::PCLVisualizer viewer("Matrix transformation example");
  viewer.registerPointPickingCallback (pp_callback);

  PointCloudXYZ::Ptr initialRegistrationCloud(new PointCloudXYZ());
  pcl::transformPointCloud(*modelCloud, * initialRegistrationCloud, icp_param.initial_guess);

  // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  source_cloud_color_handler(modelCloud, 255, 0, 0);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud(initialRegistrationCloud,
                       source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  transformed_cloud_color_handler(dataCloud, 100, 100, 100);  // Red
  viewer.addPointCloud(dataCloud, transformed_cloud_color_handler,
                       "transformed_cloud");
  viewer.addPointCloud(mestimatorWeightsCloud, "mestimator_weights");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  registered_cloud_color_handler(dataCloud, 20, 230, 20);  // Green
  viewer.addPointCloud(resultCloud,
                       registered_cloud_color_handler,
                       "registered cloud");

  viewer.addCoordinateSystem(1.0, "cloud", 0);
  // Setting background to a dark grey
  //viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
  viewer.setBackgroundColor(1., 1., 1., 0);
  viewer.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "original_cloud");
  viewer.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "registered cloud");
  viewer.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "transformed_cloud");
  viewer.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "mestimator_weights");

  std::stringstream r;
  r << "White: Origial, Red: Transformed, Green: Registered\n";
  r << icp_results;
  viewer.addText(r.str(), 0, 0);
  viewer.setShowFPS(false);

  // Display the visualiser until 'q' key is pressed
  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }
  //}

  return 0;
}






void pp_callback (const pcl::visualization::PointPickingEvent &event)
{
  if (event.getPointIndex () == -1)
    return;
  float x, y, z;
  event.getPoint(x, y, z);
  std::cout << "Point Selected: \n"
            << "\tIndex: " << event.getPointIndex ()
            << "\tCoord: (" <<  x << ", " << y << ", " << z << ")";
}
