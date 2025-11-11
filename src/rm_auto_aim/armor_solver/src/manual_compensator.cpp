// Copyright (C) FYT Vision Group. Licensed under Apache License 2.0.

#include "armor_solver/manual_compensator.hpp"

namespace rm_auto_aim {
// 更新地图
bool ManualCompensator::updateMap(const LineRegion& d_region,
                                  const LineRegion& h_region,
                                  const double pitch_offset,
                                  const double yaw_offset) {
    // 在angle_offset_map_中查找与d_region相交的节点
    auto target_dist_node = 
      std::find_if(angle_offset_map_.begin(), 
                   angle_offset_map_.end(),
                   [&](const DistMapNode& dist_node) {
     return dist_node.dist_region.checkIntersection(d_region); 
    });

    // 如果没有找到与d_region相交的节点
    if (target_dist_node == angle_offset_map_.end()) {
      // 创建一个新的HeightMapNode节点
      HeightMapNode height_node(h_region, pitch_offset, yaw_offset);
      // 将HeightMapNode节点放入HeightMapNode向量中
      std::vector<HeightMapNode> h_nodes{height_node};
      // 创建一个新的DistMapNode节点
      DistMapNode dist_node(d_region, h_nodes);
      // 将DistMapNode节点放入angle_offset_map_中
      angle_offset_map_.emplace_back(dist_node);
    } else {
      // 在target_dist_node的height_map中查找与h_region相交的节点
      auto target_height_node = 
        std::find_if(target_dist_node->height_map.begin(),
                     target_dist_node->height_map.end(),
                     [&](const HeightMapNode& height_node) {
      return height_node.height_region.checkIntersection(h_region);
      });

      // 如果没有找到与h_region相交的节点
      if (target_height_node == target_dist_node->height_map.end()) {
        // 创建一个新的HeightMapNode节点
        HeightMapNode height_node(h_region, pitch_offset, yaw_offset);
        // 将HeightMapNode节点放入target_dist_node的height_map中
        target_dist_node->height_map.emplace_back(height_node);
      } else {
        // 如果找到了与h_region相交的节点，则返回false
        return false;
      }
    }
    // 如果没有找到与d_region相交的节点，则返回true
    return true;
  }

// 根据距离和高度获取角度补偿
std::vector<double> ManualCompensator::angleHardCorrect(const double dist, 
                                           const double height) {
  // 在angle_offset_map_中查找距离区域
  auto target_dist_node = 
    std::find_if(angle_offset_map_.begin(), 
                  angle_offset_map_.end(),
                  [&](const DistMapNode& dist_node) {
    // 检查距离是否在距离区域中
    return dist_node.dist_region.checkPoint(dist); 
  });

  // 如果找到了距离区域
  if (target_dist_node != angle_offset_map_.end()) {
    // 在目标距离区域的height_map中查找高度区域
    auto target_height_node = 
      std::find_if(target_dist_node->height_map.begin(),
                    target_dist_node->height_map.end(),
                    [&](const HeightMapNode& height_node) {
      // 检查高度是否在高度区域中
      return height_node.height_region.checkPoint(height);
    });

    // 如果找到了高度区域
    if (target_height_node != target_dist_node->height_map.end()) {
      // 返回对应的俯仰和偏航补偿
      return {target_height_node->pitch_offset, target_height_node->yaw_offset};    
    }
  }
  // 如果没有找到对应的距离和高度区域，返回0补偿
  return {0.0, 0.0};
} 
  
// 解析字符串，将字符串中的数字存入向量中
bool ManualCompensator::parseStr(const std::string& str, 
                                 std::vector<double>& nums) {
  // 创建字符串流
  std::stringstream ss(str);
  double num;
  // 循环读取字符串流中的数字
  while (!ss.eof()) {
    ss >> num;
    // 将数字存入向量中
    nums.emplace_back(num);
  }
  
  // 如果向量中的数字个数不等于NORMAL_STR_NUM，返回false
  if (nums.size() != NORMAL_STR_NUM) {
    return false;
  }
  // 否则返回true
  return true;
}

bool ManualCompensator::updateMapByStr(const std::string &str) {
  // 定义一个double类型的向量nums
  std::vector<double> nums;

  // 解析字符串str，将解析后的结果存入nums中
  if (!parseStr(str, nums)) {
    // 如果解析失败，返回false
    return false;
  }

  // 根据解析后的结果，创建d_region和h_region
  LineRegion d_region(nums[0], nums[1]);
  LineRegion h_region(nums[2], nums[3]);
  // 更新地图，如果更新失败，返回false
  if (!updateMap(d_region, h_region, nums[4], nums[5])) {
    return false;
  }
  // 更新成功，返回true
  return true;
}
}  // namespace rm_auto_aim