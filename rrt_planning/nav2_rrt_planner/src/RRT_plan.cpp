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

#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <sstream>
#include "nav2_util/node_utils.hpp"

#include "nav2_rrt_planner/rrt_planner.hpp"
// 这些库文件在ros2/navigation下，不能直接在当前文件下运行
using namespace std;
namespace nav2_rrt_planner //利用该空间声明的函数
{
  vector<vector<int>> mapA(mapWidth, vector<int>(mapHeight));//地图对象
  void RRTLine::configure(
    
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
  {
    // 初始化与设置成员变量
    // 执行ROS参数的声明和规划器成员变量的初始化
    // 参数表：父节点的共享指针、规划器名称、tf缓冲指针和代价地图的共享指针
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    // mapWidth=costmap_->getSizeInCellsX();
    // mapHeight = costmap_->getSizeInCellsY();
    global_frame_ = costmap_ros->getGlobalFrameID();

    //读地图
    //////////////////////////////
    // for (int i = 0; i < mapWidth; i++)
    // {
    //   for (int j = 0; j < mapHeight; j++)
    //   {
    //     unsigned int cost = static_cast<int>(costmap_->getCost(j, i));
    //     if (cost == 0)
    //       mapA[i][j] = 1;//通行
    //     else
    //       mapA[i][j] = 0;//占用
          
    //   }
    // }
    // string s;
    
    //   for (int j = mapHeight-1; j >=0;--j) {
    //     for (int i = 0; i < mapWidth; ++i) {
    //         stringstream ss;
    //         ss << mapA[i][j];
    //         s += ss.str() + " ";
    //     }
    //     s +="\n";
        
    // } 
    // ofstream outfile("mu.txt");
    // if (outfile.is_open()) {
    //     outfile << s;
    //     outfile.close();
    //     RCLCPP_ERROR(node_->get_logger(),"-----------OKKKKKKK--------");
    // } else{
    //   RCLCPP_ERROR(node_->get_logger(),"---------NOOOOOOOOOOOOO-----");
    // }
    ifstream mapFile("map.txt");
    
    if (!mapFile.is_open()) {
        cerr << "Unable to open file" << endl;
    }
    string line;
    int j = mapHeight - 1;
    while (getline(mapFile, line) && j >= 0) {
        stringstream ss(line);
        int number;
        int i = 0;
        while (ss >> number && i < mapWidth) {
            mapA[i][j] = number;
            i++;
        }
        j--;
    }
    mapFile.close();


    //////////////////////////////

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        //获取该规划器的ROS参数[interpolation_resolution]
    // node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  }

  void RRTLine::cleanup()
  {
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void RRTLine::activate()// 开始访问路径信息
  {
    RCLCPP_INFO(
        node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
        name_.c_str());
  }

  void RRTLine::deactivate()// 退出访问路径信息
  {
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
        name_.c_str());
  }
  bool isValid(double x0, double y0, const vector<vector<int>>& mapA) {//判断是否会撞到障碍物或超出地图
      int x=int((x0+15)/30*mapWidth);
      int y=int((y0+8.83)/33.83*mapHeight);
      if (x < 0 || x >= mapWidth || y < 0 || y >= mapHeight) return false;
      for (int dx = -carRadius; dx <= carRadius; ++dx) {
          for (int dy = -carRadius; dy <= carRadius; ++dy) {// 正方形小车
              if (sqrt(dx*dx + dy*dy) <= carRadius) {//修正为圆形
                  int nx = x + dx;
                  int ny = y + dy;//放得下小车
                  if (nx < 0 || nx >= mapWidth || ny < 0 || ny >= mapHeight || mapA[nx][ny] == 0) {
                      return false;
                  }
              }
          }
      }
      return true;
  }
  Node* getNearestNode(const vector<Node*>& nodes, int x, int y) {//rrt树上的最近点
      Node* nearest = nullptr;
      double minDist = numeric_limits<double>::max();
      for (Node* node : nodes) {
          double dist = sqrt(pow(node->x - x, 2) + pow(node->y - y, 2));
          if (dist < minDist) {
              minDist = dist;
              nearest = node;
          }
      }
      return nearest;
  }
  nav_msgs::msg::Path RRTLine::createPlan(
      const geometry_msgs::msg::PoseStamped &start,
      const geometry_msgs::msg::PoseStamped &goal)
  {
    // 给定起点终点（位姿参数），输出路径，类型是nav_msgs::msg::Path
    nav_msgs::msg::Path global_path;
    
    
    
    //读地图 换成本地目录到在线订阅目录
    // 默认0自由空间，1占用或未知
    /////////////////////////////////////////
    
    // ifstream mapFile("map.txt");
    // if (!mapFile) {
    //     RCLCPP_INFO(
    //       node_->get_logger(), "NoMapFound");
    //   return global_path;
    // }
    // for (int y = 0; y < mapHeight; ++y) {
    //     for (int x = 0; x < mapWidth; ++x) {
    //         mapFile >> mapA[x][y];
    //     }
    // }
    // mapFile.close();
    /////////////////////////////////////////////


    // Checking if the goal and start state is in the global frame
    // 坐标系确认和路径初始化
    //////////////////////////////////////////////////////////////
    //(x,y)
    //(-15,25)                 (15,25)
    //
    //
    //
    //
    //
    //  ^
    //  |
    //  y
    //
    //
    //
    //
    //
    //
    //(-15,-8.83)         x->       (15,-8.83)
    if (start.header.frame_id != global_frame_)
    {
      RCLCPP_ERROR(
          node_->get_logger(), "Planner will only except start position from %s frame",
          global_frame_.c_str());
      return global_path;//空路径
    }

    if (goal.header.frame_id != global_frame_)
    {
      RCLCPP_INFO(
          node_->get_logger(), "Planner will only except goal position from %s frame",
          global_frame_.c_str());
      return global_path;
    }

    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;
    /////////////////////////////////////////////////////////////////
    // rrt计算
    //////////////////////////////////////////////
    Node* startl = new Node{start.pose.position.x,start.pose.position.y,0,0 ,nullptr};  // 起点，根据需要修改
    Node* goall = new Node{goal.pose.position.x,goal.pose.position.y,0,0, nullptr};  // 终点，根据需要修改

    // RCLCPP_ERROR(node_->get_logger(),"---%f %f %f %f-",startl->x,startl->y,goall->x,goall->y);
    // RCLCPP_ERROR(node_->get_logger(),"---(%d %d) (%d %d)-",int((startl->x+15)/30*mapWidth),int((startl->y+8.83)/33.83*mapHeight),int((goall->x+15)/30*mapWidth),int((goall->y+8.83)/33.83*mapHeight));
    
    if (!isValid(startl->x, startl->y, mapA) || !isValid(goall->x, goall->y, mapA)) {
        RCLCPP_INFO(
          node_->get_logger(), "---Start or Goal Illegal");
      return global_path;
    }//膨胀后不满足要求
    vector<Node*> tree;
    //为了方便使用push_back方法，故从goal向start遍历，
    tree.push_back(goall);
    srand(time(nullptr));//随机性
    while (true) {//遍历点

      double randX = rand()% mapWidth;
      double randY = rand() % mapHeight;
      randX=randX/mapWidth*30.0-15;
      randY=randY/mapHeight*33.83-8.83;
      // RCLCPP_ERROR(node_->get_logger(),"---(%d %d) ------",int((randX +15)/30*mapWidth),int((randY+8.83)/33.83*mapHeight));
      if (!isValid(randX, randY, mapA)) continue;//选中的该随机点是自由空间
      Node* nearest = getNearestNode(tree, randX, randY);//获得下一步的位置
      double theta = atan2(randY - nearest->y, randX - nearest->x);//方向
      double newX = nearest->x + smallsteps*step * cos(theta);// 最近点坐标加上步长得到下一个点
      double newY = nearest->y + smallsteps*step * sin(theta);//

      if (!isValid(newX, newY, mapA)) continue;//下一步目标点是自由空间
      // cout<< " Valid"<<'\n';
      // cout << nearest->x<<','<<nearest->y<<"---"<<newX<<','<<newY<<'\n' ;
      RCLCPP_ERROR(node_->get_logger(),"---(%d %d) ------",int((newX +15)/30*mapWidth),int((newY+8.83)/33.83*mapHeight));
      Node* newNode = new Node{newX, newY, cos(theta),sin(theta),nearest};
      tree.push_back(newNode);
      // drawPath(img,newNode);

      //到达终点
      if (sqrt(pow(newX - startl->x, 2) + pow(newY - startl->y, 2)) <= carRadius/10.0) {
          startl->parent = newNode;
          
          RCLCPP_ERROR(node_->get_logger(),"---FIN----------");
          break;
      }
        
    }
    //调试输出
    // Node* current = startl;
    // while (current) {
    //   cout << "(" << current->x << ", " << current->y << ")" << endl;
    //   current = current->parent;
    // }
    //内存释放，可以删除这段
    // for (Node* node : tree) {
    //     delete node;
    // }


    ////////////////////////////////////////////
    //至此，得到了goal-p1-p2----start的一条路径
    
    //// calculating the number of loops for current value of interpolation_resolution_
    //加到路径队列里
    ////////////////////////////////////////////////////////////
    RCLCPP_ERROR(node_->get_logger(),"--------------Phase4----------");
    Node* q;
    q=startl;
    while(q->parent!=nullptr)//遍历轨迹上所有点
    {
      // for (int i=0;i<smallsteps;++i){//插值给出散点，并加到
      //   geometry_msgs::msg::PoseStamped pose;//点的位姿对象
      //   pose.pose.position.x = q->x + i*step * q->costheta;
      //   pose.pose.position.y = q->y+ i*step * q->sintheta;
      //   //四元数？怎么算
      //   pose.pose.position.z = 0.0;
      //   pose.pose.orientation.x = 0.0;
      //   pose.pose.orientation.y = 0.0;
      //   pose.pose.orientation.z = 0.0;
      //   pose.pose.orientation.w = 1.0;
      //   pose.header.stamp = node_->now();
      //   pose.header.frame_id = global_frame_;
      //   global_path.poses.push_back(pose);//入栈，放进路径中
      // }
      geometry_msgs::msg::PoseStamped pose;//点的位姿对象
      pose.pose.position.x = q->x + smallsteps*step * q->costheta;
      pose.pose.position.y = q->y+ smallsteps*step * q->sintheta;
      //四元数？怎么算
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      pose.header.stamp = node_->now();
      pose.header.frame_id = global_frame_;
      global_path.poses.push_back(pose);//入栈，放进路径中
      q=q->parent;
    }
    geometry_msgs::msg::PoseStamped pose;//点的位姿对象
    pose.pose.position.x = goal.pose.position.x;
    pose.pose.position.y = goal.pose.position.x;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);//入栈，放进路径中
    //////////////////////////////////////////////////////////////
    RCLCPP_ERROR(node_->get_logger(),"--------------Phase5----------");
    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header.stamp = node_->now();
    goal_pose.header.frame_id = global_frame_;
    global_path.poses.push_back(goal_pose);
    

    return global_path;
  }

} 

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrt_planner::RRTLine, nav2_core::GlobalPlanner)
