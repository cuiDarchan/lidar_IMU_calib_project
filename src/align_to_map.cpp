#include "align_to_map.h"
#include "common/math.h"
#include "common/string_process.h"

/**
 * AlignToMap: 构造函数，订阅和发布相关topic
 */
AlignToMap::AlignToMap(const ros::NodeHandle& nh) : nh_(nh) {
  param_path_ =
      "/catkin_ws/src/lidar_IMU_calib_based_on_map/config/align_to_map.yaml";
  ParseYamlParam(param_path_);

  map_cloud_.reset(new PointCloud());
  adjust_cloud_.reset(new PointCloud());
  multi_frame_cloud_.reset(new PointCloud());
  if (map_topic_ != "none") {
    lidar_map_sub_ =
        nh_.subscribe(map_topic_, 10, &AlignToMap::MapCallback, this);
  }
  map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_cloud", 1);
  lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("lidar_adjust", 10);
  lidar_sub_ =
      nh_.subscribe(lidar_sub_topic_, 10, &AlignToMap::LidarCallback, this);
}

/**
 * ParseYamlParam: 对config中的yaml文件进行参数解析
 */
void AlignToMap::ParseYamlParam(const std::string& param_path) {
  utm_to_imu_trans_ = Eigen::Matrix4d::Identity();
  adjust_extrinsic_ = Eigen::Matrix4d::Identity();
  map_trans_ = Eigen::Matrix4d::Identity();

  YAML::Node config = YAML::LoadFile(param_path);
  map_path_ = config["map_path"].as<std::string>();
  map_topic_ = config["map_topic"].as<std::string>();
  frame_count_ = config["frame_count"].as<int>();
  auto ndt_param = config["ndt_params"];
  transformation_epsilon_ = ndt_param["transformation_epsilon"].as<float>();
  step_size_ = ndt_param["step_size"].as<float>();
  resolution_ = ndt_param["resolution"].as<float>();
  max_iterations_ = ndt_param["max_iterations"].as<float>();
  ROS_INFO_STREAM("Ndt param: "
                  << "\n"
                  << "   transformation_epsilon:" << transformation_epsilon_
                  << "\n"
                  << "   step_size:" << step_size_ << "\n"
                  << "   resolution:" << resolution_ << "\n"
                  << "   max_iterations:" << max_iterations_ << "\n");
  is_point_filter_ = config["is_point_filter"].as<int>();

  auto trans = config["map_trans"].as<Vec3>();
  map_trans_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  map_trans_.topRightCorner(3, 1) = Vector3d(trans.x, trans.y, trans.z);
  ROS_INFO_STREAM("map_trans: "
                  << "\n"
                  << map_trans_);
  lidar_sub_topic_ = config["adjust_lidar_topic"].as<std::string>();
  extrinsic_path_ = config["extrinsic_path"].as<std::string>();
  bag_path_ = config["bag_path"].as<std::string>();
  if (!GetIMUPos(bag_path_, utm_to_imu_trans_)) {
    std::cerr << "The path " << bag_path_ << " couldn't have bag_file"
              << std::endl;
  }
  ROS_INFO_STREAM("utm_to_imu_trans_: "
                  << "\n"
                  << utm_to_imu_trans_);
}

/**
 * MatrixToTranform: Eigen::Matrix4d到tf::Transform格式转换
 */
void AlignToMap::MatrixToTranform(const Eigen::Matrix4d& matrix,
                                  tf::Transform& trans) {
  tf::Vector3 origin;
  origin.setValue(static_cast<double>(matrix(0, 3)),
                  static_cast<double>(matrix(1, 3)),
                  static_cast<double>(matrix(2, 3)));

  tf::Matrix3x3 tf3d;
  tf3d.setValue(
      static_cast<double>(matrix(0, 0)), static_cast<double>(matrix(0, 1)),
      static_cast<double>(matrix(0, 2)), static_cast<double>(matrix(1, 0)),
      static_cast<double>(matrix(1, 1)), static_cast<double>(matrix(1, 2)),
      static_cast<double>(matrix(2, 0)), static_cast<double>(matrix(2, 1)),
      static_cast<double>(matrix(2, 2)));

  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);

  trans.setOrigin(origin);
  trans.setRotation(tfqt);
}

/**
 * PointCloudFilter: 点云滤波，条件，sor，半径滤波
 */
void AlignToMap::PointCloudFilter(PointCloud::Ptr& cloud) {
  PointCloud::Ptr cloud_filtered(new PointCloud);
  // std::cout << "Before filter point size:" << cloud->points.size() <<
  // std::endl;
  // 1. Condition filter
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond(
      new pcl::ConditionAnd<pcl::PointXYZ>());
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT,
                                              -45.0)));  // -45
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT,
                                              45.0)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT,
                                              -45.0)));
  range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
      new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT,
                                              45.0)));
  pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(cloud);
  condrem.setKeepOrganized(false);  // true，则点数不变，用nan值替换
  condrem.filter(*cloud_filtered);
  // std::cout << "Condition filter: " << cloud_filtered->points.size()
  //           << std::endl;

  // 2. SOR
  // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // sor.setInputCloud (cloud);
  // sor.setMeanK (4);
  // sor.setStddevMulThresh (1.0);
  // sor.filter (*cloud_filtered);

  // 3. Radius filter
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
  outrem.setInputCloud(cloud_filtered);
  outrem.setRadiusSearch(1);
  outrem.setMinNeighborsInRadius(3);
  outrem.setKeepOrganized(true);
  outrem.filter(*cloud_filtered);

  cloud = cloud_filtered;
  // std::cout << "Radius filter:" << cloud_filtered->points.size() << std::endl
  //           << std::endl;
}

/**
 * GetIMUPos: 通过bag获取静态车辆的位置姿态
 */
bool AlignToMap::GetIMUPos(const string& bag_path, Eigen::Matrix4d& imu_trans) {
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("Loading bag failed: " << e.what());
    return false;
  }

  std::vector<std::string> types;
  types.push_back("roscpp_tutorials/INSRaw");
  rosbag::View view(bag, rosbag::TypeQuery(types));

  size_t msg_num = 0;
  for (const rosbag::MessageInstance& m : view) {
    // std::cout << " Loading msg_num: \e[1m" << msg_num++ << "\e[0m from
    // rosbag" << '\r' << std::flush;
    if (msg_num > 11) break;
    roscpp_tutorials::INSRaw tf = *(m.instantiate<roscpp_tutorials::INSRaw>());
    auto time = tf.header.measured_timestamp * 1e-6;
    auto latitude = tf.position[0];   //纬度
    auto longitude = tf.position[1];  //经度
    auto elevation = tf.position[2];
    Vector3d postion(latitude, longitude, elevation);
    auto result = WGS84ToUTM(postion);  // tuple
    auto utm = std::get<0>(result);
    if (msg_num == 10) {
      Eigen::Quaterniond q(tf.orientation.w, tf.orientation.x, tf.orientation.y,
                           tf.orientation.z);
      Eigen::Isometry3d iso(q);
      iso.pretranslate(utm);
      utm_to_imu_trans_ = iso.matrix().inverse();
      ROS_INFO_STREAM("Static INS Pos: "
                      << std::fixed << utm(0) << " " << utm(1) << " " << utm(2)
                      << " " << tf.orientation.w << " " << tf.orientation.x
                      << " " << tf.orientation.y << " " << tf.orientation.z
                      << "\n");
    }
    msg_num++;
  }
  bag.close();
  return true;
}

/**
 * Run: 节点运行的主函数
 */
void AlignToMap::Run() {
  if (map_topic_ == "none") {
    if (!LoadandPublishMap(map_path_)) {
      ROS_INFO("Can't open Lidar map!");
    }
  }
}

/**
 * LoadandPublishMap: 加载地图和发布转换为IMU坐标系下的地图数据
 */
bool AlignToMap::LoadandPublishMap(const string& map_path) {
  // std::cout << "map_cloud_ points size:" << map_cloud_->points.size()
  //           << std::endl;
  if (!map_cloud_->points.size()) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *map_cloud_) == -1) {
      PCL_ERROR("Couldn't read file map_pcd.pcd \n");
      return false;
    }

    Eigen::Matrix4d tmp_trans = utm_to_imu_trans_ * map_trans_;
    pcl::transformPointCloud(*map_cloud_, *map_cloud_, tmp_trans);

    pcl::toROSMsg(*map_cloud_, output_);
    output_.header.frame_id = "novatel";
  }
  map_pub_.publish(output_);
  return true;
}

/**
 * PerformNdtOptimize: 利用NDT方式，不断优化配准
 */
void AlignToMap::PerformNdtOptimize() {
  if (!map_cloud_->points.size()) return;
  std::cout << "Start PerformNdtOptimize" << std::endl;
  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<Point, Point> ndt;

  ndt.setTransformationEpsilon(transformation_epsilon_);  // 搜索最小变化量
  ndt.setStepSize(step_size_);                            //
  ndt.setResolution(resolution_);                         // ND体素尺寸
  ndt.setMaximumIterations(max_iterations_);              // 最大迭代次数

  if (is_point_filter_) {
    PointCloudFilter(multi_frame_cloud_);
  }
  ndt.setInputSource(multi_frame_cloud_);
  ndt.setInputTarget(map_cloud_);

  PointCloud::Ptr output_cloud(new PointCloud);
  ndt.align(*output_cloud, adjust_extrinsic_.cast<float>());

  // ndt评价
  std::cout << "Normal Distributions Transform converged:" << ndt.hasConverged()
            << " score: " << ndt.getFitnessScore()
            << " prob:" << ndt.getTransformationProbability() << std::endl;

  Eigen::Matrix4f tmp = ndt.getFinalTransformation();
  adjust_extrinsic_ = tmp.cast<double>();
  pcl::transformPointCloud(*multi_frame_cloud_, *adjust_cloud_,
                           adjust_extrinsic_);
  ROS_INFO_STREAM("Corresponding transformation matrix:"
                  << "\n"
                  << adjust_extrinsic_ << "\n"
                  << "\n");

  // 打印标定结果
  Eigen::Quaterniond q(adjust_extrinsic_.block<3, 3>(0, 0));
  std::cout << "Lidar extrinsic: x,y,z,qw,qx,qy,qz "
            << "\n"
            << "transform:"
            << "\n"
            << "  translation:"
            << "\n"
            << "    x: " << adjust_extrinsic_(0, 3) << "\n"
            << "    y: " << adjust_extrinsic_(1, 3) << "\n"
            << "    z: " << adjust_extrinsic_(2, 3) << "\n"
            << "  rotation:"
            << "\n"
            << "    x: " << q.x() << "\n"
            << "    y: " << q.y() << "\n"
            << "    z: " << q.z() << "\n"
            << "    w: " << q.w() << std::endl;
}

/**
 * LidarCallback: 点云回调函数，用于点云静态多帧叠加与再发布
 */
void AlignToMap::LidarCallback(
    const sensor_msgs::PointCloud2& point_cloud_msg) {
  static int callback_cnt = 0;

  // 静态点云帧叠加
  if (callback_cnt < frame_count_) {
    PointCloud::Ptr tmp_cloud(new PointCloud());
    pcl::fromROSMsg(point_cloud_msg, *tmp_cloud);
    *multi_frame_cloud_ += *tmp_cloud;
    // std::cout << "sinle_frame_cloud size" << tmp_cloud->points.size()
    //           << std::endl;

    if (adjust_extrinsic_ == Eigen::Matrix4d::Identity()) {
      // child_frame = point_cloud_msg.header.frame_id;
      if (!ParseExtrinsicYaml(extrinsic_path_, adjust_extrinsic_)) {
        std::cerr << "Parse extrinsic yaml file failed!" << std::endl;
      }
    }
  }

  // 再调整发布
  if (callback_cnt > frame_count_) {
    PerformNdtOptimize();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*adjust_cloud_, output);
    // std::cout << "adjust_cloud_ points size" << adjust_cloud_->points.size()
    //           << std::endl;
    output.header.frame_id = "novatel";
    lidar_pub_.publish(output);
  }

  callback_cnt++;
}

/**
 * LidarCallback: 地图回调
 */
void AlignToMap::MapCallback(const sensor_msgs::PointCloud2& point_cloud_msg) {
  static int first_flag = 0;
  if (!first_flag) {
    output_ = point_cloud_msg;
    pcl::fromROSMsg(output_, *map_cloud_);
    // std::cout << "MapCallback map_cloud size:" << map_cloud_->points.size()
    //           << std::endl;
  }
  first_flag++;
  map_pub_.publish(output_);
}