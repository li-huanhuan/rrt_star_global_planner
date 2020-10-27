#include <rrt_star_global_planner/rrt_star_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

namespace RRTstar_planner
{

class RRTstarPlannerWithCostmap : public RRTstarPlannerROS
{
    public:
        RRTstarPlannerWithCostmap(std::string name, costmap_2d::Costmap2DROS* cmap);
        ~RRTstarPlannerWithCostmap();

    private:
        void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
        costmap_2d::Costmap2DROS* cmap_;
        ros::Subscriber pose_sub_;
};


void RRTstarPlannerWithCostmap::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    geometry_msgs::PoseStamped robot_pose;
    cmap_->getRobotPose(robot_pose);
    std::vector<geometry_msgs::PoseStamped> path;
    unsigned int mx = 0,my = 0;
    if(!this->costmap_->worldToMap(goal->pose.position.x,goal->pose.position.y,mx,my))
    {
      std::cout << "worldToMap error" << std::endl;
      return;
    }
    if(this->costmap_->getCost(mx,my) != costmap_2d::FREE_SPACE)
    {
      std::cout << "The target point is unreachable." << std::endl;
      return;
    }
    makePlan(robot_pose, *goal, path);
}

RRTstarPlannerWithCostmap::RRTstarPlannerWithCostmap(std::string name, costmap_2d::Costmap2DROS* cmap) :
        RRTstarPlannerROS(name, cmap)
{
    ros::NodeHandle private_nh("move_base_simple");
    cmap_ = cmap;
    pose_sub_ = private_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &RRTstarPlannerWithCostmap::poseCallback, this);
}

RRTstarPlannerWithCostmap::~RRTstarPlannerWithCostmap()
{}

} // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rrt_star_planner");
    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    costmap_2d::Costmap2DROS gcm("global_costmap", buffer);
    RRTstar_planner::RRTstarPlannerWithCostmap pppp("RRTstarPlannerROS", &gcm);
    ros::spin();
    return 0;
}

