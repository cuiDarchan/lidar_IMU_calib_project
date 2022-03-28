/******************************************************************************
 * Author: cuiDarchan
 * Date: 2021-03-03
 * Description: Processing string operation  
 *****************************************************************************/
#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <sys/io.h>
#include <dirent.h>

#include "yaml-cpp/yaml.h"

/** 
 * SplitString: 字符串分割函数
 * param1：要分割的字符串；
 * param2：作为分隔符的字符；
 * param3：存放分割后的字符串的vector向量
 */
bool SplitString(const std::string &src, const std::string &separator, std::vector<std::string> &dest)
{

    if(src.empty() || separator.empty()) return false;
    std::string str = src;
    std::string substring;
    std::string::size_type start = 0, index;
    dest.clear();
    index = str.find_first_of(separator, start);
    do
    {
        if (index != std::string::npos)
        {
            substring = str.substr(start, index - start);
            dest.push_back(substring);
            start = index + separator.size();
            index = str.find(separator, start);
            if (start == std::string::npos)
                break;
        }
    } while (index != std::string::npos);

    // the last part
    substring = str.substr(start);
    dest.push_back(substring);
    return true;
}

/** 
 * GetBagFiles: 获取文件中的bag名
 * param1：bag包的路径；
 * param2：路径下所有文件名集合；
 * param3：唯一的一个bag包名
 */
bool GetBagFiles(const std::string& bag_path, std::vector<std::string>& files,
                 std::string& bag_name) {
  // 文件目录
  DIR* pDir;
  struct dirent* ptr;  // dirent
  if (!(pDir = opendir(bag_path.c_str()))) {
    return false;
  }
  int bag_num = 0;
  while ((ptr = readdir(pDir)) != 0) {
    if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0) {
      files.push_back(ptr->d_name);
      // ROS_INFO_STREAM("File name: " << ptr->d_name);
      std::string tmp_file_name = ptr->d_name;
      const std::string bag = ".bag";
      if (tmp_file_name.find(bag) != std::string::npos) {  // 文件类型包含.bag
        bag_name = bag_path + tmp_file_name;
        ROS_INFO_STREAM("bag_name: " << bag_name);
        bag_num++;
        if (bag_num >= 2) {
          ROS_ERROR_STREAM(
              "lidar_imu_calibration/data conclude more than one bag, please "
              "delete bags, make sure one bag in data!");
          return false;
        }
      }
    }
  }
  closedir(pDir);
  if (bag_name == "") return false;
  return true;
}

/** 
 * ParseExtrinsicYaml: 参数解析函数
 */
bool ParseExtrinsicYaml(const std::string &extinsic_file,
                        Eigen::Matrix4d &extrinsic) {
  if (access(extinsic_file.c_str(), 0) != 0) {
    std::cerr << "These is no extrinsic file!" << std::endl;
  }

  try {
    YAML::Node config = YAML::LoadFile(extinsic_file);
    if (!config["transform"]) {
      return false;
    }
    if (config["transform"]["translation"] && config["transform"]["rotation"]) {
      double tx = config["transform"]["translation"]["x"].as<double>();
      double ty = config["transform"]["translation"]["y"].as<double>();
      double tz = config["transform"]["translation"]["z"].as<double>();

      double qx = config["transform"]["rotation"]["x"].as<double>();
      double qy = config["transform"]["rotation"]["y"].as<double>();
      double qz = config["transform"]["rotation"]["z"].as<double>();
      double qw = config["transform"]["rotation"]["w"].as<double>();
      extrinsic = (Eigen::Translation3d(tx, ty, tz) *
                   Eigen::Quaterniond(qw, qx, qy, qz))
                      .matrix();
      // std::cout << "tx:" << tx << " ,ty:" << ty << " ,tz:" << tz
      //           << " ,qw:" << qw << " ,qx:" << qx << " ,qy:" << qy
      //           << " ,qz:" << qz << std::endl;
    }
  } catch (const YAML::Exception &e) {
    std::cerr << "TF error!" << std::endl;
    return false;
  }
  return true;
}