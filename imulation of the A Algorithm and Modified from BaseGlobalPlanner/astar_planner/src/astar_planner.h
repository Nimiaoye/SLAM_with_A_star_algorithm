#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <fstream>

#include <vector>
#include <queue>
#define infinity 1.0e10
using namespace std;


struct Node{
  float cost;
  int index;
};

namespace astar_planner {
    class AstarPlanner : public nav_core::BaseGlobalPlanner{
        public:
            AstarPlanner();
            AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
            

      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
      
      int width;
      int height;
      int map_size;
      vector<bool> OGM;

      double getHeuristic(int cell_index, int goal_index);
      
      vector<int> get_neighbors(int current_cell);
      double getMoveCost(int firstIndex, int secondIndex);

      bool isInBounds(int x, int y);    
      /**
      * @brief  Publish a path for visualization purposes
      */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

      ros::Publisher plan_pub_;
      std::string frame_id_;
	  bool initialized_;
	  costmap_2d::Costmap2DROS* costmap_ros_;
	  costmap_2d::Costmap2D* costmap_;
    };
};
