/******************************************************************************
 * Author: cuiDarchan
 * Date: 2021-08-21
 * Description: Parse yaml file
 *****************************************************************************/
#pragma once

#include <string>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/parse.h>

struct LidarConfig{
  std::string lidar_type;
  int ring;
  std::string frame_id;
  std::string lidar_topic;
  std::string extrinsic_path;
};

struct Vec3 {
  double x, y, z;
};

struct TestPoints{
  int index;
  Vec3 positon;
};

namespace YAML {
template<>
struct convert<Vec3> {
  static Node encode(const Vec3& rhs) {
    Node node;
    node.push_back(rhs.x);
    node.push_back(rhs.y);
    node.push_back(rhs.z);
    return node;
  }

  static bool decode(const Node& node, Vec3& rhs) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs.x = node[0].as<double>();
    rhs.y = node[1].as<double>();
    rhs.z = node[2].as<double>();
    return true;
  }
};
}

namespace YAML {
template <>
struct convert<LidarConfig> {
  static Node encode(const LidarConfig& lidar_cfg) {
    Node node;
    node.push_back(lidar_cfg.lidar_type);
    node.push_back(lidar_cfg.ring);
    node.push_back(lidar_cfg.frame_id);
    node.push_back(lidar_cfg.lidar_topic);
    node.push_back(lidar_cfg.extrinsic_path);
    return node;
  }
  static bool decode(const Node& node, LidarConfig& lidar_cfg) {
    lidar_cfg.lidar_type = node["lidar_type"].as<std::string>();
    lidar_cfg.ring = node["ring"].as<int>();
    lidar_cfg.frame_id = node["frame_id"].as<std::string>();
    lidar_cfg.lidar_topic = node["lidar_topic"].as<std::string>();
    lidar_cfg.extrinsic_path = node["extrinsic_path"].as<std::string>();
    return true;
  }
};
}