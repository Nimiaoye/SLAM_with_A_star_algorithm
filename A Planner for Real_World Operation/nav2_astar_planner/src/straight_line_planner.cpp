// /*********************************************************************
//  *
//  * Software License Agreement (BSD License)
//  *
//  *  Copyright (c) 2020 Shivang Patel
//  *  All rights reserved.
//  *
//  *  Redistribution and use in source and binary forms, with or without
//  *  modification, are permitted provided that the following conditions
//  *  are met:
//  *
//  *   * Redistributions of source code must retain the above copyright
//  *     notice, this list of conditions and the following disclaimer.
//  *   * Redistributions in binary form must reproduce the above
//  *     copyright notice, this list of conditions and the following
//  *     disclaimer in the documentation and/or other materials provided
//  *     with the distribution.
//  *   * Neither the name of Willow Garage, Inc. nor the names of its
//  *     contributors may be used to endorse or promote products derived
//  *     from this software without specific prior written permission.
//  *
//  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  *  POSSIBILITY OF SUCH DAMAGE.
//  *
//  * Author: Shivang Patel
//  *
//  * Reference tutorial:
//  * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
//  *********************************************************************/

// #include <cmath>
// #include <string>
// #include <memory>
// #include "nav2_util/node_utils.hpp"

// #include "nav2_straightline_planner/straight_line_planner.hpp"

// namespace nav2_straightline_planner
// {

// void StraightLine::configure(
// const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
// std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
// std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
// {
// node_ = parent.lock();
// name_ = name;
// tf_ = tf;
// costmap_ = costmap_ros->getCostmap();
// global_frame_ = costmap_ros->getGlobalFrameID();

// // Parameter initialization
// nav2_util::declare_parameter_if_not_declared(
//   node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
//     0.1));
// node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
// }

// void StraightLine::cleanup()
// {
// RCLCPP_INFO(
//   node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
//   name_.c_str());
// }

// void StraightLine::activate()
// {
// RCLCPP_INFO(
//   node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
//   name_.c_str());
// }

// void StraightLine::deactivate()
// {
// RCLCPP_INFO(
//   node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
//   name_.c_str());
// }

// nav_msgs::msg::Path StraightLine::createPlan(
// const geometry_msgs::msg::PoseStamped & start,
// const geometry_msgs::msg::PoseStamped & goal)
// {
// nav_msgs::msg::Path global_path;

// // Checking if the goal and start state is in the global frame
// if (start.header.frame_id != global_frame_) {
//   RCLCPP_ERROR(
//     node_->get_logger(), "Planner will only except start position from %s frame",
//     global_frame_.c_str());
//   return global_path;
// }

// if (goal.header.frame_id != global_frame_) {
//   RCLCPP_INFO(
//     node_->get_logger(), "Planner will only except goal position from %s frame",
//     global_frame_.c_str());
//   return global_path;
// }

// global_path.poses.clear();

// global_path.header.stamp = node_->now();
// global_path.header.frame_id = global_frame_;
// // calculating the number of loops for current value of interpolation_resolution_





// int total_number_of_loop = std::hypot(
//   goal.pose.position.x - start.pose.position.x,
//   goal.pose.position.y - start.pose.position.y) /
//   interpolation_resolution_;
// double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
// double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

// for (int i = 0; i < total_number_of_loop; ++i) {
//   geometry_msgs::msg::PoseStamped pose;
//   pose.pose.position.x = start.pose.position.x + x_increment * i;
//   pose.pose.position.y = start.pose.position.y + y_increment * i;
//   pose.pose.position.z = 0.0;
//   pose.pose.orientation.x = 0.0;
//   pose.pose.orientation.y = 0.0;
//   pose.pose.orientation.z = 0.0;
//   pose.pose.orientation.w = 1.0;
//   pose.header.stamp = node_->now();
//   pose.header.frame_id = global_frame_;
//   global_path.poses.push_back(pose);
// }

// geometry_msgs::msg::PoseStamped goal_pose = goal;
// goal_pose.header.stamp = node_->now();
// goal_pose.header.frame_id = global_frame_;
// global_path.poses.push_back(goal_pose);

// return global_path;
// }

// }  // namespace nav2_straightline_planner






// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)



// //costmap_->get


// /*
// unsigned int mx;
// unsigned int my;

// costmap_->WorldtoMap(wx,wy,mx,my);

// costmap_->getCost(mx,my) == nav2_costmap_2d::FREE_SPACE


// costmap_->MaptoWorld(wx,wy,mx,my);

// */

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

// #include <cmath>
// #include <string>
// #include <memory>
// #include "nav2_util/node_utils.hpp"

// #include "nav2_straightline_planner/straight_line_planner.hpp"

// /////////////////////////////////////////////////////////////////////////////////////////////////////////////
// // A* Algorithm to find the path with key points
// #include <iostream>
// #include <vector>
// #include <algorithm>

// using namespace std;

// struct Point {
//     unsigned int x;
//     unsigned int y;
//     int f; // Value

//     Point(unsigned int _x, unsigned int _y, int _f = 0)
//         : x(_x), y(_y), f(_f) {}

//     bool operator==(const Point& other) const {
//         return x == other.x && y == other.y;
//     }
// };

// // Placeholder function to calculate point value
// int calculate_value(unsigned int wx, unsigned int wy) {
//     // Simulated calculation for value (replace with actual logic)
//     // costmap_->WorldtoMap(wx,wy,mx,my);
    
//     // costmap_->getCost(mx,my);
//     return 10-(wx + wy); // Example: simple sum of coordinates
// }

// vector<Point> search_path(unsigned int start_x, unsigned int start_y, unsigned int goal_x, unsigned int goal_y) {
//     vector<Point> path;
//     vector<Point> open_list;
//     vector<Point> closed_list;

//     // Lambda to check if a point is in a list
//     auto is_in_list = [](const vector<Point>& list, unsigned int x, unsigned int y) {
//         return any_of(list.begin(), list.end(), [x, y](const Point& p) {
//             return p.x == x && p.y == y;
//         });
//     };

//     open_list.emplace_back(start_x, start_y);

//     while (!open_list.empty()) {
//         // Find the point with the lowest value in the open list
//         auto current_it = min_element(open_list.begin(), open_list.end(), [](const Point& a, const Point& b) {
//             return a.f < b.f;
//         });
//         Point current = *current_it;
//         open_list.erase(current_it);

//         // Check if the goal is reached
//         if (current.x == goal_x && current.y == goal_y) {
//             path.push_back(current);
//             break;
//         }

//         closed_list.push_back(current);

//         // Check neighbors
//         for (int dx = -1; dx <= 1; ++dx) {
//             for (int dy = -1; dy <= 1; ++dy) {
//                 if (dx == 0 && dy == 0) continue; // Skip the current point
//                 unsigned int nx = current.x + dx;
//                 unsigned int ny = current.y + dy;

//                 if (!is_in_list(closed_list, nx, ny) && !is_in_list(open_list, nx, ny)) {
//                     int value = calculate_value(nx, ny);
//                     open_list.emplace_back(nx, ny, value);
//                 }
//             }
//         }

//         path.push_back(current);
//     }

//     return path;
// }



// unsigned int start_x = 0, start_y = 0;
// unsigned int goal_x = 5, goal_y = 5;

// vector<Point> path = search_path(start_x, start_y, goal_x, goal_y);

//   //  // Output the path
//   //   cout << "Path: ";
//   //   for (const Point& p : path) {
//   //       cout << "(" << p.x << "," << p.y << ") ";
//   //   }
//   //   cout << endl;
// /////////////////////////////////////////////////////////////////////////////////////////////////////////////
// namespace nav2_straightline_planner
// {

// void StraightLine::configure(
// const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
// std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
// std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
// {
// node_ = parent.lock();
// name_ = name;
// tf_ = tf;
// costmap_ = costmap_ros->getCostmap();
// global_frame_ = costmap_ros->getGlobalFrameID();

// // Parameter initialization
// nav2_util::declare_parameter_if_not_declared(
//   node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
//     0.1));
// node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
// }

// void StraightLine::cleanup()
// {
// RCLCPP_INFO(
//   node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
//   name_.c_str());
// }

// void StraightLine::activate()
// {
// RCLCPP_INFO(
//   node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
//   name_.c_str());
// }

// void StraightLine::deactivate()
// {
// RCLCPP_INFO(
//   node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
//   name_.c_str());
// }

// nav_msgs::msg::Path StraightLine::createPlan(
// const geometry_msgs::msg::PoseStamped & start,
// const geometry_msgs::msg::PoseStamped & goal)
// {
// nav_msgs::msg::Path global_path;

// // Checking if the goal and start state is in the global frame
// if (start.header.frame_id != global_frame_) {
//   RCLCPP_ERROR(
//     node_->get_logger(), "Planner will only except start position from %s frame",
//     global_frame_.c_str());
//   return global_path;
// }

// if (goal.header.frame_id != global_frame_) {
//   RCLCPP_INFO(
//     node_->get_logger(), "Planner will only except goal position from %s frame",
//     global_frame_.c_str());
//   return global_path;
// }

// global_path.poses.clear();

// global_path.header.stamp = node_->now();
// global_path.header.frame_id = global_frame_;
// // calculating the number of loops for current value of interpolation_resolution_


// // int main() {
// //     unsigned int start_x = 0, start_y = 0;
// //     unsigned int goal_x = 5, goal_y = 5;

// //     vector<Point> path = search_path(start_x, start_y, goal_x, goal_y);

// //    // Output the path
// //     cout << "Path: ";
// //     for (const Point& p : path) {
// //         cout << "(" << p.x << "," << p.y << ") ";
// //     }
// //     cout << endl;
// //     return 0;
// // }

// ///// Existing straight line interpolation code
// int total_number_of_loop = std::hypot(
//   goal.pose.position.x - start.pose.position.x,
//   goal.pose.position.y - start.pose.position.y) /
//   interpolation_resolution_;
// double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
// double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

// for (int i = 0; i < total_number_of_loop; ++i) {
//   geometry_msgs::msg::PoseStamped pose;
//   pose.pose.position.x = start.pose.position.x + x_increment * i;
//   pose.pose.position.y = start.pose.position.y + y_increment * i;
//   pose.pose.position.z = 0.0;
//   pose.pose.orientation.x = 0.0;
//   pose.pose.orientation.y = 0.0;
//   pose.pose.orientation.z = 0.0;
//   pose.pose.orientation.w = 1.0;
//   pose.header.stamp = node_->now();
//   pose.header.frame_id = global_frame_;
//   global_path.poses.push_back(pose);
// }

// geometry_msgs::msg::PoseStamped goal_pose = goal;
// goal_pose.header.stamp = node_->now();
// goal_pose.header.frame_id = global_frame_;
// global_path.poses.push_back(goal_pose);

// return global_path;
// }

// }  // namespace nav2_straightline_planner






// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)



//costmap_->get





#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include "nav2_util/node_utils.hpp"
#include "nav2_straightline_planner/straight_line_planner.hpp"

namespace nav2_straightline_planner
{

  struct Node
  {
    unsigned int x, y;
    float g_cost, h_cost;
    Node *parent;

    Node(unsigned int x, unsigned int y, float g_cost, float h_cost, Node *parent)
        : x(x), y(y), g_cost(g_cost), h_cost(h_cost), parent(parent) {}

    float f_cost() const { return g_cost + h_cost; }
  };

  struct CompareNode
  {
    bool operator()(const Node *n1, const Node *n2) const
    {
      return n1->f_cost() > n2->f_cost();
    }
  };

  void StraightLine::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {

    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
    node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  }

  void StraightLine::cleanup()
  {
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void StraightLine::activate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void StraightLine::deactivate()
  {
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
        name_.c_str());
  }






  // 创建从起点到终点的路径
nav_msgs::msg::Path StraightLine::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
  
  // 初始化路径消息
  nav_msgs::msg::Path global_path;

  // 检查起点和终点是否在相同的全局坐标系下
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
        node_->get_logger(), "Planner will only accept start position from %s frame",
        global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
        node_->get_logger(), "Planner will only accept goal position from %s frame",
        global_frame_.c_str());
    return global_path;
  }

  // 初始化路径
  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;



// 将起点和终点的世界坐标转换为地图坐标
  unsigned int start_x, start_y, goal_x, goal_y;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);

  // 执行A*搜索算法找到路径
  std::priority_queue<Node*, std::vector<Node*>, CompareNode> open_set;
  std::unordered_map<unsigned int, Node*> all_nodes;
  std::unordered_map<unsigned int, bool> closed_set;

  // 初始化起始节点
  Node* start_node = new Node(start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y), nullptr);
  open_set.push(start_node);
  all_nodes[start_y * costmap_->getSizeInCellsX() + start_x] = start_node;

  Node* goal_node = nullptr;

  // A*搜索的主循环
  while (!open_set.empty()) {
    Node* current_node = open_set.top();
    open_set.pop();

    // 检查是否到达终点
    if (current_node->x == goal_x && current_node->y == goal_y) {
      goal_node = current_node;
      break;
    }

    closed_set[current_node->y * costmap_->getSizeInCellsX() + current_node->x] = true;

    // 遍历当前节点的所有邻居节点
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;

        unsigned int neighbor_x = current_node->x + dx;
        unsigned int neighbor_y = current_node->y + dy;

        // 检查邻居节点是否在地图边界内
        if (neighbor_x >= costmap_->getSizeInCellsX() || neighbor_y >= costmap_->getSizeInCellsY()) continue;

        // 检查邻居节点是否是障碍物
        if (costmap_->getCost(neighbor_x, neighbor_y) == nav2_costmap_2d::LETHAL_OBSTACLE || 
            costmap_->getCost(neighbor_x, neighbor_y) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) continue;

        // 检查邻居节点是否已经在关闭列表中
        if (closed_set[neighbor_y * costmap_->getSizeInCellsX() + neighbor_x]) continue;

        double cur_world_x, cur_world_y, neighbor_world_x, neighbor_world_y;
        costmap_->mapToWorld(current_node->x, current_node->y, cur_world_x, cur_world_y);
        costmap_->mapToWorld(neighbor_x, neighbor_y, neighbor_world_x, neighbor_world_y);

        // 修改部分：增加代价权重
        float inflation_cost = costmap_->getCost(neighbor_x, neighbor_y);
        float cost_weight = (inflation_cost == nav2_costmap_2d::FREE_SPACE) ? 1.0 : inflation_cost / 100.0;

        // 计算邻居节点的代价
        float g_cost = current_node->g_cost + std::hypot(neighbor_world_x - cur_world_x, neighbor_world_y - cur_world_y) * cost_weight;
        float h_cost = heuristic(neighbor_world_x, neighbor_world_y, goal.pose.position.x, goal.pose.position.y);

        Node* neighbor_node = all_nodes[neighbor_y * costmap_->getSizeInCellsX() + neighbor_x];
        if (!neighbor_node) {
          neighbor_node = new Node(neighbor_x, neighbor_y, g_cost, h_cost, current_node);
          all_nodes[neighbor_y * costmap_->getSizeInCellsX() + neighbor_x] = neighbor_node;
          open_set.push(neighbor_node);
        } else if (g_cost < neighbor_node->g_cost) {
          neighbor_node->g_cost = g_cost;
          neighbor_node->parent = current_node;
        }
      }
    }
  }

  // 如果找到了通往终点的路径，则从终点回溯到起点构建路径
  if (goal_node) {
    for (Node* node = goal_node; node != nullptr; node = node->parent) {
      geometry_msgs::msg::PoseStamped pose;
      double wx, wy;
      costmap_->mapToWorld(node->x, node->y, wx, wy);
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);

      RCLCPP_INFO(
          node_->get_logger(), "Path point: (%.2f, %.2f), f_cost: %.2f, h_cost: %.2f",
          wx, wy, node->f_cost(), node->h_cost);
    }
    std::reverse(global_path.poses.begin(), global_path.poses.end());
  } else {
    RCLCPP_WARN(node_->get_logger(), "Failed to find a path to the goal.");
  }

  // 清理动态分配的节点
  for (auto& pair : all_nodes) {
    delete pair.second;
  }

  while (!open_set.empty()) {
    open_set.pop();
  }

  closed_set.clear();

  return global_path;
}

// 启发式函数，用于估算从当前节点到目标节点的代价
float StraightLine::heuristic(double x1, double y1, double x2, double y2) {
  return std::hypot(x2 - x1, y2 - y1);
}

} // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
// 将StraightLine规划器导出为插件
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)