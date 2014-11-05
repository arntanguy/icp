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
#include <sstream>
#include "eigentools.hpp"
#include "icp.hpp"
#include "error_point_to_point.hpp"
#include "mestimator_hubert.hpp"


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



  /**
    Initial guess
   **/
  Eigen::Matrix<float, 6, 1> initial_guess = Eigen::MatrixXf::Zero(6, 1);


  /*
     Define parameters for the ICP
     */
  //Do it step by step (i has to be set to max_iter param)
  //for (int i = 0; i < 5; i++) {
  icp::IcpParametersf icp_param;
  icp_param.lambda = 1.f;
  icp_param.max_iter = 100;
  icp_param.min_variation = 10e-9;
  icp_param.initial_guess = initial_guess;
  LOG(INFO) << "ICP Parameters:\n" << icp_param;

  icp::Icp<float, icp::ErrorPointToPoint<float>, icp::MEstimatorHubert<float>>
      icp_algorithm;
  icp_algorithm.setParameters(icp_param);
  icp_algorithm.setModelPointCloud(modelCloud);
  icp_algorithm.setDataPointCloud(dataCloud);
  icp_algorithm.run();

  icp::IcpResultsf icp_results = icp_algorithm.getResults();
  LOG(INFO) << "ICP Results:\n" << icp_results;


  /**
   * Visualize
   **/
  LOG(INFO) << "\nPoint cloud colors :  white  = original point cloud\n"
            "                       red  = transformed point cloud\n";
  pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

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
  viewer.addPointCloud(icp_results.registeredPointCloud,
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
