#include <boost/shared_ptr.hpp>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>
#include <sstream>
#include "icp.hpp"
#include "logging.hpp"



typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

PointCloudXYZ::Ptr modelCloud(new PointCloudXYZ());
PointCloudXYZ::Ptr dataCloud(new PointCloudXYZ());
PointCloudXYZ::Ptr initialRegistrationCloud(new PointCloudXYZ());
PointCloudXYZ::Ptr resultCloud(new PointCloudXYZ());
PointCloudXYZRGB::Ptr mestimatorWeightsCloud(new PointCloudXYZRGB());

//icp::IcpPointToPointHubert icp_algorithm;
icp::IcpPointToPointHubertSO3 icp_algorithm;

pcl::visualization::PCLVisualizer viewer("Step by step icp");
//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> mestimator_color_handler(mestimatorWeightsCloud);  // Green
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> result_cloud_color_handler(resultCloud, 0, 255,
    0);  // Green

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void *viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> v =
    *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
  if (event.getKeySym () == "s" && event.keyDown ())
  {
    LOG(INFO) << "s was pressed => running 1 iteration";
    icp_algorithm.step();
    icp::IcpResults icp_results = icp_algorithm.getResults();
    LOG(INFO) << "ICP Results:\n" << icp_results;
    pcl::transformPointCloud(*modelCloud, *resultCloud, icp_results.transformation);
    icp_algorithm.createMEstimatorCloud(mestimatorWeightsCloud);
    LOG(INFO) << "mestimator cloud created";

    result_cloud_color_handler.setInputCloud(resultCloud);
    viewer.removePointCloud("result_cloud");
    viewer.addPointCloud(resultCloud, result_cloud_color_handler, "result_cloud");
    //viewer.updatePointCloud(mestimatorWeightsCloud, "mestimator_weights");
  }
}

int main(int argc, char *argv[])
{
  /*
     Define parameters for the ICP
     */
  icp::IcpParameters icp_param;
  icp_param.max_iter = 20;
  icp_param.min_variation = 10e-5;
  //icp_param.max_correspondance_distance = 10e-5;
  icp_param.initial_guess = Eigen::Matrix4f::Identity();
  icp_param.mestimator = true;
  // Far
  //icp_param.initial_guess(0, 3) = 1.6;
  //icp_param.initial_guess(1, 3) = 0.6;
  //icp_param.initial_guess(2, 3) = 1.6;
  // Less far
  //icp_param.initial_guess(0, 3) = 2;
  //icp_param.initial_guess(1, 3) = 0.7;
  //icp_param.initial_guess(2, 3) = 1;
  // Almost registered
  icp_param.initial_guess(0, 3) = 2.176;
  icp_param.initial_guess(1, 3) = 0.868;
  icp_param.initial_guess(2, 3) = 1;
  LOG(INFO) << "ICP Parameters:\n" << icp_param;


  /*
     Loads test point cloud
     */
  LOG(INFO) << "Loading Model pointcloud";
  //std::string model = "../models/ladder_robot/ladder_jr13_arnaud.pcd";
  std::string model = "../models/valve.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (model.c_str(), *modelCloud) == -1) {
    LOG(FATAL) << "Could't read file " << model;
    return (-1);
  }
  LOG(INFO) << "Model Point cloud has " << modelCloud->points.size()
            << " points";
  std::string data = "../models/valve_simulation.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZ> (data.c_str(), *dataCloud) == -1) {
    LOG(FATAL) << "Could't read file " << model;
    return (-1);
  }
  LOG(INFO) << "Model Point cloud has " << dataCloud->points.size()
            << " points";

  Eigen::Matrix4f rot30x;
  rot30x << 1.0000, 0, 0, 0,
         0,  0.1543, 0.9880, 0,
         0, -0.9880, 0.1543, 0,
         0, 0, 0, 1;

  Eigen::Matrix4f rot50x ;
  rot50x << 1, 0, 0, 0,
        0, 0.9650, 0.2624, 0,
        0, -0.2624,  0.9650, 0,
        0, 0, 0, 1;

  pcl::transformPointCloud(*modelCloud, *modelCloud, rot50x);
  pcl::transformPointCloud(*modelCloud, *initialRegistrationCloud, icp_param.initial_guess);



  icp_algorithm.setParameters(icp_param);
  icp_algorithm.setInputCurrent(modelCloud);
  icp_algorithm.setInputReference(dataCloud);
  //icp_algorithm.run();
  //icp_algorithm.createMEstimatorCloud(mestimatorWeightsCloud);



  // Viewer
  // Setting background to a dark grey
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);

  // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(modelCloud, 255, 255, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> reference_cloud_color_handler(dataCloud, 230, 20, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> result_cloud_color_handler(resultCloud, 0, 255,
      0);  // Green

  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud(initialRegistrationCloud, source_cloud_color_handler, "original_cloud");
  viewer.addPointCloud(dataCloud, reference_cloud_color_handler, "reference_cloud");
  viewer.addPointCloud(resultCloud, result_cloud_color_handler, "result_cloud");
  viewer.addPointCloud(mestimatorWeightsCloud, "mestimator_weights");

  viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "reference_cloud");
  viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "result_cloud");
  viewer.setPointCloudRenderingProperties( pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "mestimator_weights");

  std::stringstream r;
  r << "White: Origial, Red: Transformed, Green: Registered\n";
  //r << icp_results;
  viewer.addText(r.str(), 0, 0);
  viewer.setShowFPS(false);
  viewer.registerKeyboardCallback (keyboardEventOccurred, (void *)&viewer);

  // Display the visualiser until 'q' key is pressed
  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }

  return 0;
}
