//  This file is part of the Icp Library,
//
//  Copyright (C) 2014 by Arnaud TANGUY <arn.tanguy@NOSPAM.gmail.com>
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <sstream>
#include "eigentools.hpp"
#include "icp.hpp"
#include "error_point_to_point.hpp"
#include "error_point_to_plane.hpp"
#include "mestimator_hubert.hpp"

/**
 * @brief Point picker callback
 *
 * @param event
 */
void pp_callback (const pcl::visualization::PointPickingEvent &event);


int main(int argc, char *argv[]) {
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  LOG(INFO) << "Starting ICP program";

  /*
     Loads test point cloud
     */
  LOG(INFO) << "Loading Model pointcloud";
  pcl::PointCloud<pcl::PointXYZ>::Ptr modelCloud(
    new pcl::PointCloud<pcl::PointXYZ>());
  //std::string model = "../models/ladder_robot/ladder_jr13_arnaud.pcd";
  std::string model = "../models/teapot.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (model.c_str(), *modelCloud) == -1) {
    LOG(FATAL) << "Could't read file " << model;
    return (-1);
  }
  LOG(INFO) << "Model Point cloud has " << modelCloud->points.size()
            << " points";


  /*
     Creating a second transformed pointcloud
     */
  Eigen::Matrix4f transformation
    = eigentools::createTransformationMatrix(0.f,
        0.05f,
        0.f,
        static_cast<float>(M_PI) / 200.f,
        static_cast<float>(M_PI) / 200.f,
        0.f);
  LOG(INFO) << "Transformation:\n" << transformation;

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr dataCloud
  (new pcl::PointCloud<pcl::PointXYZ>());
  // Generates a data point cloud to be matched against the model
  pcl::transformPointCloud(*modelCloud, *dataCloud, transformation);



  pcl::PointCloud<pcl::PointXYZ>::Ptr resultCloud(new pcl::PointCloud<pcl::PointXYZ>());


  /*
     Define parameters for the ICP
     */
  //Do it step by step (i has to be set to max_iter param)
  //for (int i = 0; i < 5; i++) {
  //icp::IcpParametersXYZ icp_param;
  //icp_param.lambda = 1.f;
  //icp_param.max_iter = 100;
  //icp_param.min_variation = 10e-5;
  //icp_param.initial_guess = Eigen::MatrixXf::Zero(6, 1);
  //LOG(INFO) << "ICP Parameters:\n" << icp_param;

  ///**
  // * Point to point
  // **/
  //icp::IcpPointToPointHubert icp_algorithm;
  //icp_algorithm.setParameters(icp_param);
  //icp_algorithm.setInputCurrent(modelCloud);
  //icp_algorithm.setInputReference(dataCloud);
  //icp_algorithm.run();
  //
  //icp::IcpResultsXYZ icp_results = icp_algorithm.getResults();
  //LOG(INFO) << "ICP Results:\n" << icp_results;
  //pcl::copyPointCloud(*(icp_results.registeredPointCloud), *resultCloud);

  /**
   * Point to point (sim3)
   **/
  /*
     Creating a second transformed pointcloud
     */
  transformation(0,0) *= 0.5;
  transformation(1,1) *= 0.5;
  transformation(2,2) *= 0.5;
  LOG(INFO) << "Transformation:\n" << transformation;
  // Generates a data point cloud to be matched against the model
  pcl::transformPointCloud(*modelCloud, *dataCloud, transformation);

  icp::IcpParametersXYZSim3 icp_param_sim3;
  icp_param_sim3.lambda = 1.f;
  icp_param_sim3.max_iter = 100;
  icp_param_sim3.min_variation = 10e-5;
  icp_param_sim3.initial_guess = Eigen::MatrixXf::Zero(7,1); 
  LOG(INFO) << "ICP Parameters Sim3:\n" << icp_param_sim3;

  icp::IcpPointToPointHubertSim3 icp_algorithm;
  icp_algorithm.setParameters(icp_param_sim3);
  icp_algorithm.setInputCurrent(modelCloud);
  icp_algorithm.setInputReference(dataCloud);
  icp_algorithm.run();
  
  icp::IcpResultsXYZ icp_results = icp_algorithm.getResults();
  LOG(INFO) << "ICP Results:\n" << icp_results;
  pcl::copyPointCloud(*(icp_results.registeredPointCloud), *resultCloud);


  /**
   * Point to Plane
   **/
  //LOG(INFO) << "Computing cloud normals";
  //// Create the normal estimation class, and pass the input dataset to it
  //pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
  //// Use all neighbors in a sphere of radius 10m
  //// WARNING: Wrong value of this parameter may result to having NaN normals in
  //// case nearest neighbors aren't found!
  ////ne.setRadiusSearch (10);
  //ne.setKSearch(100);
  //// Create an empty kdtree representation, and pass it to the normal estimation object.
  //// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new
  //    pcl::search::KdTree<pcl::PointXYZ> ());
  //ne.setSearchMethod (tree);

  //ne.setInputCloud (modelCloud);
  //// Output datasets
  //pcl::PointCloud<pcl::Normal>::Ptr mesh_normal_pc (new
  //    pcl::PointCloud<pcl::Normal>);
  //// Compute the features
  //ne.compute (*mesh_normal_pc);
  //pcl::PointCloud<pcl::PointNormal>::Ptr mesh_pointnormal_pc(
  //  new pcl::PointCloud<pcl::PointNormal>);
  //pcl::concatenateFields(*modelCloud, *mesh_normal_pc, *mesh_pointnormal_pc);

  //ne.setInputCloud (dataCloud);
  //// Output datasets
  //pcl::PointCloud<pcl::Normal>::Ptr scene_normal_pc (new
  //    pcl::PointCloud<pcl::Normal>);
  //// Compute the features
  //ne.compute (*scene_normal_pc);
  //pcl::PointCloud<pcl::PointNormal>::Ptr scene_pointnormal_pc(
  //  new pcl::PointCloud<pcl::PointNormal>);
  //pcl::concatenateFields(*dataCloud, *scene_normal_pc, *scene_pointnormal_pc);

  //for (unsigned int i = 0; i < mesh_pointnormal_pc->size(); i++) {
  //  LOG(WARNING) << (*scene_normal_pc)[i];
  //}

  //// Point to plane ICP
  //icp::IcpPointToPlaneHubertSim3 icp_algorithm;
  //icp_algorithm.setParameters(icp_param);
  //icp_algorithm.setInputCurrent(scene_pointnormal_pc);
  //icp_algorithm.setInputReference(mesh_pointnormal_pc);
  //icp_algorithm.run();

  //icp::IcpResults_<float, pcl::PointNormal> icp_results = icp_algorithm.getResults();
  //LOG(INFO) << "ICP Results:\n" << icp_results;
  //pcl::copyPointCloud(*(icp_results.registeredPointCloud), *resultCloud);


  /**
   * Visualize
   **/
  LOG(INFO) << "\nPoint cloud colors :  white  = original point cloud\n"
            "                       red  = transformed point cloud\n";
  pcl::visualization::PCLVisualizer viewer("Matrix transformation example");
  viewer.registerPointPickingCallback (pp_callback);


  // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  source_cloud_color_handler(modelCloud, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud(modelCloud,
                       source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  transformed_cloud_color_handler(dataCloud, 230, 20, 20);  // Red
  viewer.addPointCloud(dataCloud, transformed_cloud_color_handler,
                       "transformed_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
  registered_cloud_color_handler(dataCloud, 20, 230, 20);  // Green
  viewer.addPointCloud(resultCloud,
                       registered_cloud_color_handler,
                       "registered cloud");

  viewer.addCoordinateSystem(1.0, "cloud", 0);
  // Setting background to a dark grey
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
  viewer.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");

  std::stringstream r;
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
  LOG(INFO) << "Point Selected: \n"
            << "\tIndex: " << event.getPointIndex ()
            << "\tCoord: (" <<  x << ", " << y << ", " << z << ")";
}
