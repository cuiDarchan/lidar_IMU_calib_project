/******************************************************************************
 * Author: cuiDarchan
 * Date: 2022-03-22
 * Description: Align lidar to cloud_map to calculate lidar2imu extrinsic
 *****************************************************************************/

#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "parse_yaml.h"
#include "roscpp_tutorials/INSRaw.h"

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;

using std::string;

class AlignToMap {
 public:
  AlignToMap(const ros::NodeHandle& nh);
  void PointCloudFilter(PointCloud::Ptr& cloud);
  bool GetIMUPos(const string& bag_path, Eigen::Matrix4d& imu_trans);
  bool LoadandPublishMap(const string& map_path);
  void MatrixToTranform(const Eigen::Matrix4d& matrix, tf::Transform& trans);
  void PerformNdtOptimize();
  void LidarCallback(const sensor_msgs::PointCloud2& point_cloud_msg);
  void MapCallback(const sensor_msgs::PointCloud2& point_cloud_msg);
  void ParseYamlParam(const string& param_path);
  void Run();
    
 private:
  ros::NodeHandle nh_;
  string map_topic_;
  string param_path_;
  string map_path_;
  string bag_path_;
  string lidar_sub_topic_;
  string extrinsic_path_;
  Eigen::Matrix4d map_trans_;
  int frame_count_;

  ros::Publisher map_pub_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber lidar_map_sub_;
  ros::Publisher lidar_pub_;
  PointCloud::Ptr map_cloud_;
  PointCloud::Ptr multi_frame_cloud_;
  PointCloud::Ptr adjust_cloud_;

  Eigen::Matrix4d utm_to_imu_trans_;
  Eigen::Matrix4d adjust_extrinsic_;
  sensor_msgs::PointCloud2 output_;

  // ndt_params
  float transformation_epsilon_ = 0.01;
  float step_size_ = 0.05;
  float resolution_ =  1.0;
  float max_iterations_ = 35;
  int is_point_filter_ = 1;
  
};