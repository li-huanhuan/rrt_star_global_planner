#ifndef RRT_STAR_ROS_H
#define RRT_STAR_ROS_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <cmath>
#include <vector>
#include <visualization_msgs/Marker.h>

struct Node
{
  double x;
  double y;
  int node_id;
	int parent_id;
  double cost;
  
  bool operator ==(const Node& node) 
  {
    return (fabs(x - node.x) < 0.0001) && (fabs(y - node.y) < 0.0001) && (node_id == node.node_id) && (parent_id == node.parent_id) && (fabs(cost - node.cost) < 0.0001) ;
  }

  bool operator !=(const Node& node) 
  {
    if((fabs(x - node.x) > 0.0001) || (fabs(y - node.y) > 0.0001) || (node_id != node.node_id) || (parent_id != node.parent_id) || (fabs(cost - node.cost) > 0.0001))
      return true;
    else
      return false;
  }
}; 

enum GetPlanMode
{
  TREE1 = 1,
  TREE2 = 2,
  CONNECT1TO2 = 3,
  CONNECT2TO1 = 4,
};

namespace RRTstar_planner 
{

  class RRTstarPlannerROS : public nav_core::BaseGlobalPlanner 
  {

    public:
      /**
      * @brief Default constructor of the plugin
      */
      RRTstarPlannerROS();

      RRTstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      ~RRTstarPlannerROS();

      /**
      * @brief  Initialization function for the PlannerCore object
      * @param  name The name of this planner
      * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
      */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan);

      void getPathFromTree1ConnectTree2(std::vector< Node >& tree1,
                                        std::vector< Node >& tree2,
                                        Node& connect_node,
                                        std::vector<geometry_msgs::PoseStamped>& plan);

      void getPathFromTree(std::vector< Node >& tree1,
                           std::vector< Node >& tree2,
                           Node& connect_node,
                           std::vector<geometry_msgs::PoseStamped>& plan,
                           GetPlanMode mode);
      
      /*
      * @brief Compute the euclidean distance (straight-line distance) between two points
      * @param px1 point 1 x
      * @param py1 point 1 y
      * @param px2 point 2 x
      * @param py2 point 2 y
      * @return the distance computed
      */
      double distance(double px1, double py1, double px2, double py2);

      /**
       * @brief Generate random points.
       * @return the a random point in the map.
      */
      std::pair<double, double> sampleFree(); //随机采样点

      /**
       * @brief Check if there is a collision.
       * @param x coordinate (cartesian system)
       * @param y coordinate (cartesian system)
       * @return True is the point collides and false otherwise
      */
      bool collision(double x, double y); //是否为障碍物

      /**
       * @brief Check whether there are obstacles around.
       * @param x coordinate
       * @param y coordinate
       * @return True is the there are obstacles around
      */
      bool isAroundFree(double wx, double wy);

      bool isConnect(Node new_node,std::vector< Node >& another_tree,std::vector< Node >& current_tree,Node& connect_node);

      /**
       * @brief Given the nodes set and an point the function returns the closest node of the node
       * @param nodes the set of nodes
       * @param p_rand the random point (x,y) in the plane
       * return the closest node
      */
      Node getNearest(std::vector<Node> nodes, std::pair<double, double> p_rand); //搜索最近的节点

      /**
       * @brief Select the best parent. Check if there is any node around the newnode with cost less than its parent node cost. 
       * If yes choose this less cost node as the new parent of the newnode.
       * @param nn the parent of the newnode
       * @param newnode the node that will checked if there is a better parent for it
       * @param nodes the set of nodes
       * @return the same newnode with the best parent node
       * 
      */
      Node chooseParent(Node nn, Node newnode, std::vector<Node> nodes); //选择父节点

      /*
      * 该功能检查周围所有节点的父节点的开销是否仍小于新节点。
      * 如果存在父节点成本较高的节点，则该节点的新父节点现在是newnode。
      * 参数:nodes节点集。
      * 参数:newnode 新节点
      */
      void rewire(std::vector<Node>& nodes, Node newnode);

      /*
       * @brief The function generate the new point between the epsilon_min and epsilon_max along the line p_rand and nearest node. 
       *        This new point is a node candidate. It will a node if there is no obstacles between its nearest node and itself.
       * @param px1 point 1 x
       * @param py1 point 1 y
       * @param px2 point 2 x
       * @param py2 point 2 y
      * @return the new point
      */
      std::pair<double, double> steer(double x1, double y1,double x2, double y2); //生成新的树枝

      bool obstacleFree(Node node_nearest, double px, double py); //检查树枝是否碰撞障碍物

      /**
       * @brief Check if the distance between the goal and the newnode is less than the goal_radius. If yes the newnode is the goal.
       * @param px1 point 1 x
       * @param py1 point 1 y
       * @param px2 point 2 x
       * @param py2 point 2 y
       * *@return True if distance is less than the xy tolerance (GOAL_RADIUS), False otherwise
      */
      bool pointCircleCollision(double x1, double y1, double x2, double y2, double radius);

      void optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan);

      void insertPointForPath(std::vector< std::pair<double, double> >& pathin,double param);

      int optimizationPath(std::vector< std::pair<double, double> >& plan,double movement_angle_range = M_PI/4); //优化路径

      bool isLineFree(const std::pair<double, double> p1,const std::pair<double, double> p2);

      void cutPathPoint(std::vector< std::pair<double, double> >& plan); //优化路径

      double inline normalizeAngle(double val,double min = -M_PI,double max = M_PI); //标准化角度

      void pubTreeMarker(ros::Publisher& marker_pub,visualization_msgs::Marker marker,int id);
    protected:

      costmap_2d::Costmap2D* costmap_;
      costmap_2d::Costmap2DROS* costmap_ros_;
      std::string frame_id_;
      ros::Publisher plan_pub_;

    private:

      visualization_msgs::Marker marker_tree_;
      visualization_msgs::Marker marker_tree_2_;
      ros::Publisher marker_pub_;

      size_t max_nodes_num_;
      double plan_time_out_;
      double search_radius_;
      double goal_radius_;
      double epsilon_min_;
      double epsilon_max_;

      //路径优化参数
      double path_point_spacing_;
      double angle_difference_;

      double resolution_;
      bool initialized_;

  };
} // RRTstar_planner namespace
#endif
