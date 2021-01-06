#include <rrt_star_global_planner/rrt_star_ros.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <tf/tf.h>

PLUGINLIB_EXPORT_CLASS(RRTstar_planner::RRTstarPlannerROS, nav_core::BaseGlobalPlanner)

namespace RRTstar_planner
{

  RRTstarPlannerROS::RRTstarPlannerROS() :
    costmap_(nullptr),
    initialized_(false)
  {}

  RRTstarPlannerROS::RRTstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros) :
    costmap_ros_(costmap_ros),
    initialized_(false)
  {
    initialize(name, costmap_ros);
  }

  RRTstarPlannerROS::~RRTstarPlannerROS()
  {}

  //变量初始化
  void RRTstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if (!initialized_)
    {
      // Initialize map
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros->getCostmap();
      frame_id_ = costmap_ros->getGlobalFrameID();
  
      ros::NodeHandle private_nh("~/" + name);
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan",1); //发布全局计算
      marker_pub_ = private_nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 ); //发布可视化扩展过程
  
      resolution_ = costmap_->getResolution(); //地图分辨率

      double max_nodes_num_tem = 0.0;
      private_nh.param("search_radius",search_radius_,1.0); //搜索附近的节点的搜索范围
      private_nh.param("goal_radius",goal_radius_,0.2); //认为搜索到目标点的范围
      private_nh.param("epsilon_min",epsilon_min_,0.001); //节点之间的最小允许距离
      private_nh.param("epsilon_max",epsilon_max_,0.1); //节点之间的最大允许距离
      private_nh.param("max_nodes_num",max_nodes_num_tem,2000000000.0); //节点数的最大值，最大迭代次数
      private_nh.param("plan_time_out",plan_time_out_,10.0); //规划超时。默认10s
      max_nodes_num_ = static_cast<size_t>(max_nodes_num_tem);

      //路径优化参数
      path_point_spacing_ = 0.025; //路径点间隔
      angle_difference_ = M_PI/20; //前后相邻点向量角度差

      ROS_INFO("RRT* planner initialized successfully");
      initialized_ = true;
    }
    else
    {
      ROS_WARN("This planner has already been initialized... doing nothing");
    }
  }

  //标准化角度
  double RRTstarPlannerROS::normalizeAngle(double val,double min,double max) //标准化角度
  {
    double norm = 0.0;
    if (val >= min)
      norm = min + fmod((val - min), (max-min));
    else
      norm = max - fmod((min - val), (max-min));

    return norm;
  }

  //发布整个树的可视化扩展
  void RRTstarPlannerROS::pubTreeMarker(ros::Publisher &marker_pub, visualization_msgs::Marker marker,int id)
  {
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "marker_namespace";
    marker.id = id;
    //marker.type = visualization_msgs::Marker::SPHERE;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
//         marker.scale.y = 0.1;
//         marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
    marker.frame_locked = false;

    marker_pub.publish(marker);
  }

  //路径规划，最终move_base调用的函数
  bool RRTstarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   std::vector<geometry_msgs::PoseStamped>& plan)
  {
    plan.clear();

    if(this->collision(start.pose.position.x, start.pose.position.y))
    {
      ROS_WARN("failed to get a path.start point is obstacle.");
      return false;
    }

    if(this->collision(goal.pose.position.x, goal.pose.position.y))
    {
      ROS_WARN("failed to get a path.goal point is obstacle.");
      return false;
    }

    this->marker_tree_.points.clear();
    this->marker_tree_2_.points.clear();
    plan.clear();
    std::vector< std::pair<double, double> > path;

    std::vector< Node > nodes; //第一棵树
    Node start_node;
    start_node.x = start.pose.position.x;
    start_node.y = start.pose.position.y;
    start_node.node_id = 0;
    start_node.parent_id = -1; // None parent node
    start_node.cost = 0.0;
    nodes.push_back(start_node);

    std::vector< Node > nodes_2; //第二棵树
    Node goal_node;
    goal_node.x = goal.pose.position.x;
    goal_node.y = goal.pose.position.y;
    goal_node.node_id = 0;
    goal_node.parent_id = -1; // None parent node
    goal_node.cost = 0.0;
    nodes_2.push_back(goal_node);

    std::pair<double, double> p_rand; //随机采样的可行点
    std::pair<double, double> p_new; //第一棵树的新节点
    std::pair<double, double> p_new_2; //第二棵树的新节点
    Node connect_node_on_tree1; //第二课树与第一课树连接到一起时第一课树上距离第二课树最近的节点
    Node connect_node_on_tree2; //第一课树与第二课树连接到一起时第二课树上距离第二课树最近的节点
    bool is_connect_to_tree1 = false;
    bool is_connect_to_tree2 = false;

    Node node_nearest;

    unsigned int seed = 0;
    double start_time = ros::Time::now().toSec();
    while (ros::ok() && nodes.size() + nodes_2.size() < max_nodes_num_)
    {
      if( (ros::Time::now().toSec()-start_time) > plan_time_out_)
      {
        ROS_WARN("failed to get a path.time out.");
        return false;
      }
      // 第一棵树
      while (ros::ok())
      {
        srand(ros::Time::now().toNSec() + seed++);//修改种子
        unsigned int rand_nu = rand()%10;
        if(rand_nu > 1) // 0.8的概率使用随机采样扩展
        {
          p_rand = sampleFree(); // random point in the free space
        }
        else // 0.2的概率使用启发扩展
        {
          p_rand.first = goal.pose.position.x;
          p_rand.second = goal.pose.position.y;
        }

        node_nearest = getNearest(nodes, p_rand); // The nearest node of the random point
        p_new = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second); // new point and node candidate.
        if (obstacleFree(node_nearest, p_new.first, p_new.second))
        {//树枝无碰撞
          Node newnode;
          newnode.x = p_new.first;
          newnode.y = p_new.second;
          newnode.node_id = nodes.size(); // index of the last element after the push_bask below
          newnode.parent_id = node_nearest.node_id;
          newnode.cost = 0.0;

          // 优化
          newnode = chooseParent(node_nearest, newnode, nodes); // Select the best parent
          nodes.push_back(newnode);
          rewire(nodes, newnode);

          geometry_msgs::Point point_tem;
          point_tem.x = nodes[newnode.parent_id].x;
          point_tem.y = nodes[newnode.parent_id].y;
          point_tem.z = 0;
          this->marker_tree_.points.push_back(point_tem);

          point_tem.x = newnode.x;
          point_tem.y = newnode.y;
          point_tem.z = 0;
          this->marker_tree_.points.push_back(point_tem);

          if(nodes.size() % 10 == 0)
          {
            this->pubTreeMarker(this->marker_pub_,this->marker_tree_,1);
          }

          if(this->isConnect(newnode,nodes_2,nodes, connect_node_on_tree2))
          {
            is_connect_to_tree2 = true;
          }

          break;
        }
      }

      //两棵树连接在了一起,第一棵树搜索到了第二棵树上的节点
      if(is_connect_to_tree2)
      {
        std::cout << "两棵树连接在了一起 1->2 耗时：" << ros::Time::now().toSec() - start_time << "秒" << std::endl;

        getPathFromTree(nodes,nodes_2,connect_node_on_tree2,plan,GetPlanMode::CONNECT1TO2);

        plan[0].pose.orientation = start.pose.orientation;
        plan[plan.size()-1].pose.orientation = goal.pose.orientation;

        nav_msgs::Path path_pose;
        path_pose.header.frame_id = this->frame_id_;
        path_pose.header.stamp = ros::Time::now();
        path_pose.poses = plan;
        plan_pub_.publish(path_pose);
        return true;
      }

      // 第一棵树搜索到目标点
      if (pointCircleCollision(p_new.first, p_new.second, goal.pose.position.x , goal.pose.position.y, goal_radius_) )
      {
        std::cout << "第一棵树搜索到目标点,耗时：" << ros::Time::now().toSec() - start_time << "秒" << std::endl;

        getPathFromTree(nodes,nodes_2,nodes.back(),plan,GetPlanMode::TREE1);

        plan[0].pose.orientation = start.pose.orientation;
        plan[plan.size()-1].pose.orientation = goal.pose.orientation;

        nav_msgs::Path path_pose;
        path_pose.header.frame_id = this->frame_id_;
        path_pose.header.stamp = ros::Time::now();
        path_pose.poses = plan;
        plan_pub_.publish(path_pose);
        return true;
      }

      // 第二棵树
      p_rand.first = p_new.first;
      p_rand.second = p_new.second;
      while (ros::ok())
      {
        node_nearest = getNearest(nodes_2, p_rand); // The nearest node of the random point
        p_new_2 = steer(node_nearest.x, node_nearest.y, p_rand.first, p_rand.second); // new point and node candidate.
        if (obstacleFree(node_nearest, p_new_2.first, p_new_2.second))
        {
          Node newnode;
          newnode.x = p_new_2.first;
          newnode.y = p_new_2.second;
          newnode.node_id = nodes_2.size(); // index of the last element after the push_bask below
          newnode.parent_id = node_nearest.node_id;
          newnode.cost = 0.0;

          // Optimize
          newnode = chooseParent(node_nearest, newnode, nodes_2); // Select the best parent
          nodes_2.push_back(newnode);
          rewire(nodes_2, newnode);

          geometry_msgs::Point point_tem;
          point_tem.x = nodes_2[newnode.parent_id].x;
          point_tem.y = nodes_2[newnode.parent_id].y;
          point_tem.z = 0;
          this->marker_tree_2_.points.push_back(point_tem);

          point_tem.x = newnode.x;
          point_tem.y = newnode.y;
          point_tem.z = 0;
          this->marker_tree_2_.points.push_back(point_tem);

          if(nodes_2.size() % 10 == 0)
          {
            pubTreeMarker(this->marker_pub_,this->marker_tree_2_,2);
          }

          if(this->isConnect(newnode,nodes,nodes_2, connect_node_on_tree1))
          {
            is_connect_to_tree1 = true;;
          }

          break;
        }
        else
        {
          srand(ros::Time::now().toNSec() + seed++);//修改种子
          unsigned int rand_nu = rand()%10;
          if(rand_nu > 1) // 0.8的概率使用随机采样扩展
          {
            p_rand = sampleFree(); // random point in the free space
          }
          else // 0.2的概率使用启发扩展
          {
            p_rand.first = start.pose.position.x;
            p_rand.second = start.pose.position.y;
          }
        }
      }

      //两棵树连接在了一起，第二棵树搜索到了第一棵树上的节点
      if(is_connect_to_tree1)
      {
        std::cout << "两棵树连接在了一起 2->1 耗时：" << ros::Time::now().toSec() - start_time << "秒" << std::endl;

        getPathFromTree(nodes,nodes_2,connect_node_on_tree1,plan,GetPlanMode::CONNECT2TO1);

        plan[0].pose.orientation = start.pose.orientation;
        plan[plan.size()-1].pose.orientation = goal.pose.orientation;

        nav_msgs::Path path_pose;
        path_pose.header.frame_id = this->frame_id_;
        path_pose.header.stamp = ros::Time::now();
        path_pose.poses = plan;
        plan_pub_.publish(path_pose);
        return true;
      }

      //第二棵树搜索到目标点
      if (pointCircleCollision(p_new_2.first, p_new_2.second, start.pose.position.x , start.pose.position.y, goal_radius_) )
      {
        std::cout << "第二棵树搜索到目标点,耗时：" << ros::Time::now().toSec() - start_time << "秒" << std::endl;

        getPathFromTree(nodes,nodes_2,nodes.front(),plan,GetPlanMode::TREE2);

        plan[0].pose.orientation = start.pose.orientation;
        plan[plan.size()-1].pose.orientation = goal.pose.orientation;

        nav_msgs::Path path_pose;
        path_pose.header.frame_id = this->frame_id_;
        path_pose.header.stamp = ros::Time::now();
        path_pose.poses = plan;
        plan_pub_.publish(path_pose);
        return true;
      }

    }
    ROS_WARN("failed to get a path.");
    return false;
  }

  //得到路径，两棵树连接在了一起
  void RRTstarPlannerROS::getPathFromTree(std::vector< Node >& tree1,
                                          std::vector< Node >& tree2,
                                          Node& connect_node,
                                          std::vector<geometry_msgs::PoseStamped>& plan,
                                          GetPlanMode mode)
  {
    std::pair<double, double> point;
    std::vector< std::pair<double, double> > path;
    Node current_node;

    //第一棵树搜索到第二课树上的节点
    if(mode == GetPlanMode::CONNECT1TO2 )
    {
      current_node = tree1.back();
    }

    //第一棵树搜索到路径点
    if(GetPlanMode::TREE1)
    {
      current_node = tree1.back();
    }

    // 第二棵树搜索到第一棵上的节点
    if(mode == GetPlanMode::CONNECT2TO1)
    {
      current_node = connect_node;
    }

    //第二棵树搜索到路径
    if(mode == GetPlanMode::TREE2)
    {
      current_node = tree1[0];
    }

    // Final Path
    while (current_node.parent_id != tree1[0].parent_id)
    {
      point.first = current_node.x;
      point.second = current_node.y;
      path.insert(path.begin(), point); //在开头插入一个元素
      current_node = tree1[current_node.parent_id];
    }

    if(mode == GetPlanMode::CONNECT1TO2) //1->2
    {
      current_node = connect_node;
    }

    if(mode == GetPlanMode::TREE1) //1->goal
    {
      current_node = tree2[0];
    }

    if(mode == GetPlanMode::TREE2) //2->start
    {
      current_node = tree2.back();
    }

    if(mode == GetPlanMode::CONNECT2TO1) //2->1
    {
      current_node = tree2.back();
    }

    while (current_node.parent_id != tree2[0].parent_id)
    {
      point.first = current_node.x;
      point.second = current_node.y;
      path.push_back(point);
      current_node = tree2[current_node.parent_id];
    }

    point.first = tree1[0].x;
    point.second = tree1[0].y;
    path.insert(path.begin(), point); //开始点
    point.first = tree2[0].x;
    point.second = tree2[0].y;
    path.push_back(point); //目标点

    cutPathPoint(path);

    insertPointForPath(path,this->path_point_spacing_);

    optimizationPath(path,this->angle_difference_);

    // convert the points to poses
    ros::Time plan_time = ros::Time::now();
    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    for (size_t i = 0; i < path.size(); i++)
    {
      pose.header.stamp = plan_time;
      pose.header.frame_id = this->frame_id_;
      pose.pose.position.x = path[i].first;
      pose.pose.position.y = path[i].second;
      plan.push_back(pose);
    }

    optimizationOrientation(plan);
  }


  void RRTstarPlannerROS::optimizationOrientation(std::vector<geometry_msgs::PoseStamped> &plan)
  {
    size_t num = plan.size()-1;
    if(num < 1)
      return;
    for(size_t i=0;i<num;i++)
    {
      plan[i].pose.orientation = tf::createQuaternionMsgFromYaw( atan2( plan[i+1].pose.position.y - plan[i].pose.position.y,
                                                                 plan[i+1].pose.position.x - plan[i].pose.position.x ) );
    }
  }

  //计算新节点与目标点的距离是否小于位置允许误差
  bool RRTstarPlannerROS::pointCircleCollision(double x1, double y1, double x2, double y2, double goal_radius)
  {
    double dist = distance(x1, y1, x2, y2);
    if (dist < goal_radius)
      return true;
    else
      return false;
  }

  //路径插点
  void RRTstarPlannerROS::insertPointForPath(std::vector< std::pair<double, double> >& path_in,double param)
  {
      std::vector< std::pair<double, double> > path_out;
      size_t size = path_in.size() - 1;
      std::pair<double, double> point;
      double pp_dist = param;
      for(size_t i=0;i<size;i++)
      {
        double theta = atan2(path_in[i+1].second - path_in[i].second,path_in[i+1].first - path_in[i].first);
        size_t insert_size = static_cast<size_t>(this->distance(path_in[i+1].first,path_in[i+1].second,path_in[i].first,path_in[i].second) / pp_dist + 0.5);
        for(size_t j=0;j<insert_size;j++)
        {
          point.first = path_in[i].first + j * pp_dist * cos(theta);
          point.second = path_in[i].second + j * pp_dist * sin(theta);
          path_out.push_back(point);
        }
      }
      path_out.push_back( path_in.back() );
      path_in.clear();
      size = path_out.size();
      path_in.resize(size);
      for(size_t i=0;i<size;i++)
      {
        path_in[i] = path_out[i];
      }
  }

  int RRTstarPlannerROS::optimizationPath(std::vector< std::pair<double, double> >& plan,double movement_angle_range)
  {
    if(plan.empty())
      return 0;
    size_t point_size = plan.size() - 1;
    double px,py,cx,cy,nx,ny,a_p,a_n;
    bool is_run = false;
    int ci = 0;
    for(ci=0;ci<1000;ci++)
    {
      is_run = false;
      for(size_t i=1;i<point_size;i++)
      {
        px = plan[i-1].first;
        py = plan[i-1].second;

        cx = plan[i].first;
        cy = plan[i].second;

        nx = plan[i+1].first;
        ny = plan[i+1].second;

        a_p = normalizeAngle(atan2(cy-py,cx-px),0,2*M_PI);
        a_n = normalizeAngle(atan2(ny-cy,nx-cx),0,2*M_PI);

        if(std::max(a_p,a_n)-std::min(a_p,a_n) > movement_angle_range)
        {
          plan[i].first = (px + nx)/2;
          plan[i].second = (py + ny)/2;
          is_run = true;
        }
      }
      if(!is_run)
        return ci;
    }
    return ci;
  }

  bool RRTstarPlannerROS::isLineFree(const std::pair<double, double> p1, const std::pair<double, double> p2)
  {
   std::pair<double, double> ptmp;
    ptmp.first = 0.0;
    ptmp.second = 0.0;

    double dist = sqrt( (p2.second-p1.second) * (p2.second-p1.second) +
                        (p2.first-p1.first) * (p2.first-p1.first) );
    if (dist < this->resolution_)
    {
        return true;
    }
    else
    {
      int value = int(floor(dist/this->resolution_));
      double theta = atan2(p2.second - p1.second,
                           p2.first - p1.first);
      int n = 1;
      for (int i = 0;i < value; i++)
      {
        ptmp.first = p1.first + this->resolution_*cos(theta) * n;
        ptmp.second = p1.second + this->resolution_*sin(theta) * n;
        if (collision(ptmp.first, ptmp.second))
          return false;
        n++;
      }
      return true;
    }
  }

  void RRTstarPlannerROS::cutPathPoint(std::vector< std::pair<double, double> >& plan)
  {
    size_t current_index = 0;
    size_t check_index = current_index+2;
    while(ros::ok())
    {
      if( current_index >= plan.size()-2 )
        return;
      if( this->isLineFree(plan[current_index], plan[check_index]) ) //点之间无障碍物
      {
        std::vector< std::pair<double, double> >::iterator it = plan.begin()+ static_cast<int>(current_index + 1) ;
        if(check_index-current_index-1 == 1)
        {
          plan.erase(it);
        }
        else
        {
          plan.erase(it,it+static_cast<int>( check_index-current_index-1) );
          check_index = current_index + 2;
        }
      }
      else
      {
        if(check_index < plan.size()-1 )
          check_index++;
        else
        {
          current_index++;
          check_index = current_index + 2;
        }
      }
    }
  }

  //计算两点间距离
  double RRTstarPlannerROS::distance(double px1, double py1, double px2, double py2)
  {
    return sqrt((px1 - px2)*(px1 - px2) + (py1 - py2)*(py1 - py2));
  }

  //随机采样
  std::pair<double, double> RRTstarPlannerROS::sampleFree()
  {
    std::pair<double, double> random_point;
    unsigned int mx = 0,my = 0;
    double wx = 0.0,wy = 0.0;
    unsigned int map_size_x = costmap_->getSizeInCellsX();
    unsigned int map_size_y = costmap_->getSizeInCellsY();
    unsigned int seed = 0;
    while(ros::ok())
    {
      srand(ros::Time::now().toNSec() + seed++);//修改种子
      mx = rand() % map_size_x;
      srand(ros::Time::now().toNSec() + seed++);//修改种子
      my = rand() % map_size_y;
      if(this->costmap_->getCost(mx,my) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        break;
    }
    this->costmap_->mapToWorld(mx,my,wx,wy);
    random_point.first = wx;
    random_point.second = wy;
    return random_point;
  }

  //检查点是否为障碍物
  bool RRTstarPlannerROS::collision(double x, double y)
  {
    unsigned int mx,my;
    if(!this->costmap_->worldToMap(x,y,mx,my))
      return true;
    if ((mx >= costmap_->getSizeInCellsX()) || (my >= costmap_->getSizeInCellsY()))
      return true;
    if (costmap_->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      return true;
    return false;
  }
  
  //检查点的周围是否有障碍物
  bool RRTstarPlannerROS::isAroundFree(double wx, double wy)
  {
    unsigned int mx, my;
    if(!this->costmap_->worldToMap(wx,wy,mx,my))
      return false;
    if(mx <= 1 || my <= 1 || mx >= this->costmap_->getSizeInCellsX()-1 || my >= this->costmap_->getSizeInCellsY()-1)
      return false;
    int x,y;
    for(int i=-1;i<=1;i++)
    {
      for(int j=-1;j<=1;j++)
      {
        x = static_cast<int>(mx) + i;
        y = static_cast<int>(my) + j;
        if(this->costmap_->getCost(static_cast<unsigned int>(x),static_cast<unsigned int>(y)) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
          return false;
      }
    }
    return true;
  }

  bool RRTstarPlannerROS::isConnect(Node new_node,std::vector< Node >& another_tree,std::vector< Node >& current_tree,Node& connect_node)
  {
    size_t node_size = another_tree.size();
    double min_distance = 10000000.0;
    double distance = 0.0;
    size_t min_distance_index = 0;
    for(size_t i=0;i<node_size;i++)
    {
      distance = this->distance(new_node.x,new_node.y,another_tree[i].x,another_tree[i].y);
      if(distance < min_distance)
      {
        min_distance = distance;
        min_distance_index = i;
      }
    }

    distance = this->distance(new_node.x,new_node.y,another_tree[min_distance_index].x,another_tree[min_distance_index].y);

    if(distance < this->goal_radius_)
    {
      connect_node = another_tree[min_distance_index];
      Node newnode = another_tree[min_distance_index];
      // Optimize
      newnode = chooseParent(current_tree.back(), newnode, current_tree); // Select the best parent
      current_tree.push_back(newnode);
      rewire(current_tree, newnode);
      return true;
    }
    return false;
  }

  //搜索树上距离最近的节点
  Node RRTstarPlannerROS::getNearest(std::vector< Node > nodes, std::pair<double, double> p_rand)
  {
    double dist_min = distance(nodes[0].x, nodes[0].y, p_rand.first, p_rand.second);
    double dist_curr = 0;
    size_t index_min = 0;
    for (size_t i = 1; i < nodes.size(); i++)
    {
      dist_curr = distance(nodes[i].x, nodes[i].y, p_rand.first, p_rand.second);
      if (dist_curr < dist_min)
      {
        dist_min = dist_curr;
        index_min = i;
      }
    }
    return nodes[index_min];
  }

  //选择最优的父节点
  Node RRTstarPlannerROS::chooseParent(Node nn, Node newnode, std::vector<Node> nodes)
  {
    double dist_nnn = distance(nn.x, nn.y, newnode.x, newnode.y);
    for (size_t i = 0; i < nodes.size(); i++)
    {
      if (distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < search_radius_ &&
          nodes[i].cost + distance(nodes[i].x, nodes[i].y, newnode.x, newnode.y) < nn.cost + dist_nnn &&
          obstacleFree(nodes[i], newnode.x, newnode.y))
      {
        nn = nodes[i];
      }
    }
    newnode.cost = nn.cost + distance(nn.x, nn.y, newnode.x, newnode.y);
    if(!this->isAroundFree(newnode.x,newnode.y))
      newnode.cost += 0.3;
    newnode.parent_id = nn.node_id;
    return newnode;
  }

  //优化新节点附近的节点，选择最优的父节点
  void RRTstarPlannerROS::rewire(std::vector<Node>& nodes, Node newnode)
  {
    for (size_t i = 0; i < nodes.size(); i++)
    {
      Node& node = nodes[i];
      if (node != nodes[newnode.parent_id] &&
          distance(node.x, node.y, newnode.x, newnode.y) < search_radius_ &&
          newnode.cost + distance(node.x, node.y, newnode.x, newnode.y) < node.cost &&
          obstacleFree(node, newnode.x, newnode.y))
      {
        node.parent_id = newnode.node_id;
        node.cost = newnode.cost + distance(node.x, node.y, newnode.x, newnode.y);
        if(!this->isAroundFree(node.x,node.y))
          node.cost += 0.3;
      }
    }
  }

  //在树上扩展新节点
  std::pair<double, double> RRTstarPlannerROS::steer(double x1, double y1,double x2, double y2)
  {
    std::pair<double, double> p_new;
    double dist = distance(x1, y1, x2, y2);
    if (dist < epsilon_max_ && dist > epsilon_min_)
    {
      p_new.first = x1;
      p_new.second = y1;
    }
    else
    {
      double theta = atan2(y2-y1, x2-x1);
      p_new.first = x1 + epsilon_max_*cos(theta);
      p_new.second = y1 + epsilon_max_*sin(theta);
    }
    return p_new;
  }

  //判断两节点之间是否有障碍物
  bool RRTstarPlannerROS::obstacleFree(Node node_nearest, double px, double py)
  {
    std::pair<double, double> p_n;
    p_n.first = 0.0;
    p_n.second = 0.0;

    double dist = distance(node_nearest.x, node_nearest.y, px, py);
    if (dist < resolution_)
    {
      if (collision(px, py))
        return false;
      else
        return true;
    }
    else
    {
      int value = int(floor(dist/resolution_));
      double theta = atan2(py - node_nearest.y, px - node_nearest.x);
      int n = 1;
      for (int i = 0;i < value; i++)
      {
        p_n.first = node_nearest.x + n*resolution_*cos(theta);
        p_n.second = node_nearest.y + n*resolution_*sin(theta);
        if (collision(p_n.first, p_n.second))
          return false;
        n++;
      }
      return true;
    }
  }


}; // RRTstar_planner namespace
