#include <custom_planner/custom_planner.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav_msgs/Path.h>
#include <custom_planner/SBPLLatticePlannerStats_.h>

#include <costmap_2d/inflation_layer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

using namespace std;
using namespace ros;

PLUGINLIB_EXPORT_CLASS(custom_planner::CustomPlanner, nav_core::BaseGlobalPlanner)

namespace geometry_msgs
{
  bool operator==(const Point &p1, const Point &p2)
  {
    return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
  }
}

namespace custom_planner
{

  class LatticeSCQ : public StateChangeQuery
  {
  public:
    LatticeSCQ(EnvironmentNAVXYTHETALAT *env, std::vector<nav2dcell_t> const &changedcellsV)
        : env_(env), changedcellsV_(changedcellsV)
    {
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const *getPredecessors() const
    {
      if (predsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
      return &predsOfChangedCells_;
    }

    // lazy init, because we do not always end up calling this method
    virtual std::vector<int> const *getSuccessors() const
    {
      if (succsOfChangedCells_.empty() && !changedcellsV_.empty())
        env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
      return &succsOfChangedCells_;
    }

    EnvironmentNAVXYTHETALAT *env_;
    std::vector<nav2dcell_t> const &changedcellsV_;
    mutable std::vector<int> predsOfChangedCells_;
    mutable std::vector<int> succsOfChangedCells_;
  };

  CustomPlanner::CustomPlanner()
      : initialized_(false), costmap_ros_(NULL)
  {
  }

  CustomPlanner::CustomPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
      : initialized_(false), costmap_ros_(NULL)
  {
    initialize(name, costmap_ros);
  }

  void CustomPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
  {
    if (!initialized_)
    {
      ros::NodeHandle private_nh("~/" + name);
      ros::NodeHandle p_nh("~");
      ROS_INFO("Name is %s", name.c_str());

      userParams_ = new userParams();
      pathway = new Pathway();
      startPose = new Pose();
      input_spline_inf = new Spline_Inf();
      CurveDesign = new Curve_common();

      private_nh.param("planner_type", planner_type_, string("ARAPlanner"));
      private_nh.param("allocated_time", allocated_time_, 10.0);
      private_nh.param("initial_epsilon", initial_epsilon_, 3.0);
      private_nh.param("environment_type", environment_type_, string("XYThetaLattice"));
      private_nh.param("forward_search", forward_search_, bool(false));
      p_nh.param("primitive_filename", primitive_filename_, string(""));
      private_nh.param("force_scratch_limit", force_scratch_limit_, 500);

      double nominalvel_mpersecs, timetoturn45degsinplace_secs;
      private_nh.param("nominalvel_mpersecs", nominalvel_mpersecs, 0.4);
      private_nh.param("timetoturn45degsinplace_secs", timetoturn45degsinplace_secs, 0.6);

      int lethal_obstacle;
      private_nh.param("lethal_obstacle", lethal_obstacle, 20);
      lethal_obstacle_ = (unsigned char)lethal_obstacle;
      inscribed_inflated_obstacle_ = lethal_obstacle_ - 1;
      sbpl_cost_multiplier_ = (unsigned char)(costmap_2d::INSCRIBED_INFLATED_OBSTACLE / inscribed_inflated_obstacle_ + 1);
      ROS_INFO("SBPL: lethal: %uz, inscribed inflated: %uz, multiplier: %uz", lethal_obstacle, inscribed_inflated_obstacle_, sbpl_cost_multiplier_);

      private_nh.param("publish_footprint_path", publish_footprint_path_, bool(true));
      private_nh.param<int>("visualizer_skip_poses", visualizer_skip_poses_, 5);

      private_nh.param("allow_unknown", allow_unknown_, bool(true));

      private_nh.param("directory_to_save_paths", userParams_->directory_to_save_paths, userParams_->directory_to_save_paths);
      private_nh.param("pathway_filename", userParams_->pathway_filename, userParams_->pathway_filename);
      private_nh.param("current_pose_topic_name", userParams_->current_pose_topic_name, userParams_->current_pose_topic_name);
      private_nh.param("map_frame_id", userParams_->map_frame_id, userParams_->map_frame_id);
      private_nh.param("base_frame_id", userParams_->base_frame_id, userParams_->base_frame_id);

      name_ = name;
      costmap_ros_ = costmap_ros;

      footprint_ = costmap_ros_->getRobotFootprint();

      if ("XYThetaLattice" == environment_type_)
      {
        ROS_DEBUG("Using a 3D costmap for theta lattice\n");
        env_ = new EnvironmentNAVXYTHETALAT();
      }
      else
      {
        ROS_ERROR("XYThetaLattice is currently the only supported environment!\n");
        exit(1);
      }

      circumscribed_cost_ = computeCircumscribedCost();

      if (circumscribed_cost_ == 0)
      {
        // Unfortunately, the inflation_radius is not taken into account by
        // inflation_layer->computeCost(). If inflation_radius is smaller than
        // the circumscribed radius, SBPL will ignore some obstacles, but we
        // cannot detect this problem. If the cost_scaling_factor is too large,
        // SBPL won't run into obstacles, but will always perform an expensive
        // footprint check, no matter how far the nearest obstacle is.
        ROS_WARN("The costmap value at the robot's circumscribed radius (%f m) is 0.", costmap_ros_->getLayeredCostmap()->getCircumscribedRadius());
        ROS_WARN("SBPL performance will suffer.");
        ROS_WARN("Please decrease the costmap's cost_scaling_factor.");
      }
      if (!env_->SetEnvParameter("cost_inscribed_thresh", costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
      {
        ROS_ERROR("Failed to set cost_inscribed_thresh parameter");
        exit(1);
      }
      if (!env_->SetEnvParameter("cost_possibly_circumscribed_thresh", circumscribed_cost_))
      {
        ROS_ERROR("Failed to set cost_possibly_circumscribed_thresh parameter");
        exit(1);
      }
      int obst_cost_thresh = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);
      vector<sbpl_2Dpt_t> perimeterptsV;
      perimeterptsV.reserve(footprint_.size());
      for (size_t ii(0); ii < footprint_.size(); ++ii)
      {
        sbpl_2Dpt_t pt;
        pt.x = footprint_[ii].x;
        pt.y = footprint_[ii].y;
        perimeterptsV.push_back(pt);
      }

      bool ret;
      try
      {
        // ret = env_->InitializeEnv(costmap_ros_->getCostmap()->getSizeInCellsX(), // width
        //                           costmap_ros_->getCostmap()->getSizeInCellsY(), // height
        //                           0,                                             // mapdata
        //                           0, 0, 0,                                       // start (x, y, theta, t)
        //                           0, 0, 0,                                       // goal (x, y, theta)
        //                           0, 0, 0,                                       // goal tolerance
        //                           perimeterptsV, costmap_ros_->getCostmap()->getResolution(), nominalvel_mpersecs,
        //                           timetoturn45degsinplace_secs, obst_cost_thresh,
        //                           primitive_filename_.c_str());
        current_env_width_ = costmap_ros_->getCostmap()->getSizeInCellsX();
        current_env_height_ = costmap_ros_->getCostmap()->getSizeInCellsY();        
      }
      catch (SBPL_Exception *e)
      {
        ROS_ERROR("SBPL encountered a fatal exception: %s", e->what());
        ret = false;
      }
      // if (!ret)
      // {
      //   ROS_ERROR("SBPL initialization failed!");
      //   exit(1);
      // }
      // for (ssize_t ix(0); ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ++ix)
      //   for (ssize_t iy(0); iy < costmap_ros_->getCostmap()->getSizeInCellsY(); ++iy)
      //     env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix, iy)));

      // if ("ARAPlanner" == planner_type_)
      // {
      //   ROS_INFO("Planning with ARA*");
      //   planner_ = new ARAPlanner(env_, forward_search_);
      // }
      // else if ("ADPlanner" == planner_type_)
      // {
      //   ROS_INFO("Planning with AD*");
      //   planner_ = new ADPlanner(env_, forward_search_);
      // }
      // else
      // {
      //   ROS_ERROR("ARAPlanner and ADPlanner are currently the only supported planners!\n");
      //   exit(1);
      // }

      ROS_INFO("[custom_planner] Initialized successfully");      
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
      stats_publisher_ = private_nh.advertise<custom_planner::SBPLLatticePlannerStats_>("sbpl_lattice_planner_stats", 1);
      sbpl_plan_footprint_pub_ = private_nh.advertise<visualization_msgs::Marker>("footprint_markers", 1);
      string pathway_fullfilename = userParams_->directory_to_save_paths + "/" + userParams_->pathway_filename;        
      if(loadPathwayData(pathway_fullfilename)) cout<< "Success in load pathway file: "<<pathway_fullfilename<<endl;
      else std::cout<<pathway_fullfilename<<" is not existed"<<std::endl;
      // order_msg_sub_ = private_nh.subscribe("/order",1000,&CustomPlanner::order_msg_handle,this);
      service_servers_.push_back(p_nh.advertiseService("set_plan_with_order", &CustomPlanner::HandleSetPlanWithOrder, this));

      // vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> control_point;
      // control_point.push_back(Eigen::Vector3d(18.383729, 10.68481159, 0));
      // control_point.push_back(Eigen::Vector3d(18.34523, 9.21587, 0));
      // control_point.push_back(Eigen::Vector3d(18.31081, 7.40332, 0));
      // control_point.push_back(Eigen::Vector3d(18.328079, 5.4313039, 0));
      // std::vector<double> knot_vector = {0, 0, 0, 0.5, 1, 1, 1};
      // std::vector<double> weight_vector = {1, 1, 1, 1};
      // if(curveIsValid(2, knot_vector, control_point))
      // {
      //   double t_intervel = 0.02;  
      //   uint32_t num_of_point =0;
      //   posesOnPathWay.clear();
      //   input_spline_inf->control_point.clear();
      //   input_spline_inf->knot_vector.clear();
      //   input_spline_inf->weight.clear();      
      //   CurveDesign->ReadSplineInf(input_spline_inf, 3, control_point, knot_vector);
      //   CurveDesign->ReadSplineInf(input_spline_inf, weight_vector, false);
      //   for(double u_test = 0; u_test <= 1; u_test += t_intervel)
      //   { 
      //     geometry_msgs::Point curve_point;
      //     curve_point = CurveDesign->CalculateCurvePoint(input_spline_inf, u_test, true);
      //     if(!std::isnan(curve_point.x)&&!std::isnan(curve_point.y))
      //     posesOnPathWay.push_back(Pose(curve_point.x, curve_point.y, 0.0));
      //     ROS_WARN("posesOnPathWay: %f, %f   at u: %f",curve_point.x, curve_point.y, u_test);
      //   }
      //   num_of_point = (uint32_t)posesOnPathWay.size();
      //   ROS_WARN("num_of_point: %d", num_of_point);
      //   setYawAllPosesOnEdge(posesOnPathWay, false);

      //   if(posesOnPathWay.front().getX()==input_spline_inf->control_point[0].x() && 
      //     posesOnPathWay.front().getY()==input_spline_inf->control_point[0].y())
      //   {
      //     ROS_WARN("posesOnPathWay.front() and control_point[0] are the same point");
      //   }
      //   setYawAllPosesOnEdge(posesOnPathWay, false);

      //   if(posesOnPathWay.front().getX()==input_spline_inf->control_point.back().x() && 
      //     posesOnPathWay.front().getY()==input_spline_inf->control_point.back().y())
      //   {
      //     ROS_WARN("posesOnPathWay.back() and control_point.back() are the same point");
      //   }

      //   if(posesOnPathWay.back().getYaw()==0.0123443210){
      //     for(auto &p:posesOnPathWay)
      //     {
      //       ROS_WARN(" before insert: %f, %f, %f",p.getX(), p.getY(), p.getYaw());
      //     }
      //     posesOnPathWay.insert(posesOnPathWay.begin(), Pose(11.2, 9.305829, -0.063551));
      //     posesOnPathWay.insert(posesOnPathWay.begin(), Pose(11.0, 9.305829, -0.063551));
      //     vector<Pose> test_poses;
      //     test_poses.push_back(Pose(18.292795, 5.5, 0));
      //     test_poses.push_back(Pose(18.292795, 5.2, 0));
      //     test_poses.push_back(Pose(18.292795, 5, 0));
      //     posesOnPathWay.insert(posesOnPathWay.end(), test_poses.begin() + 1, test_poses.end());
      //     for(auto &p:posesOnPathWay)
      //     {
      //       ROS_WARN(" after insert: %f, %f, %f",p.getX(), p.getY(), p.getYaw());
      //     } 
      //   }
      // }

      // std::vector<int> source = {1, 2, 3, 4, 5, 6, 7, 88, 9, 10, 11, 12, 13, 14};
      // int n = 0;
      // vector<int> result;
      // for(int i = (int)source.size()-1; i>=0; i--)
      // {
      //   if(source[i] == 88)
      //   {          
      //     result.assign(source.begin()+i, source.end());
      //   }
      // }
      // std::reverse(result.begin(), result.end());
      // for (int num : result) {
      //     ROS_WARN("num: %d", num);
      // }
      initialized_ = true;
    }
  }

  // Taken from Sachin's sbpl_cart_planner
  // This rescales the costmap according to a rosparam which sets the obstacle cost
  unsigned char CustomPlanner::costMapCostToSBPLCost(unsigned char newcost)
  {
    if (newcost == costmap_2d::LETHAL_OBSTACLE || (!allow_unknown_ && newcost == costmap_2d::NO_INFORMATION))
      return lethal_obstacle_;
    else if (newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      return inscribed_inflated_obstacle_;
    else if (newcost == 0)
      return 0;
    else if(newcost == costmap_2d::NO_INFORMATION)
      return  costmap_2d::FREE_SPACE/sbpl_cost_multiplier_;
    else
    {
      unsigned char sbpl_cost = newcost / sbpl_cost_multiplier_;
      if (sbpl_cost == 0)
        sbpl_cost = 1;
      return sbpl_cost;
    }
  }

  void CustomPlanner::publishStats(int solution_cost, int solution_size,
                                        const geometry_msgs::PoseStamped &start,
                                        const geometry_msgs::PoseStamped &goal)
  {
    // Fill up statistics and publish
    custom_planner::SBPLLatticePlannerStats_ stats;
    stats.initial_epsilon = initial_epsilon_;
    stats.plan_to_first_solution = false;
    stats.final_number_of_expands = planner_->get_n_expands();
    stats.allocated_time = allocated_time_;

    stats.time_to_first_solution = planner_->get_initial_eps_planning_time();
    stats.actual_time = planner_->get_final_eps_planning_time();
    stats.number_of_expands_initial_solution = planner_->get_n_expands_init_solution();
    stats.final_epsilon = planner_->get_final_epsilon();

    stats.solution_cost = solution_cost;
    stats.path_size = solution_size;
    stats.start = start;
    stats.goal = goal;
    stats_publisher_.publish(stats);
  }

  unsigned char CustomPlanner::computeCircumscribedCost()
  {
    unsigned char result = 0;

    if (!costmap_ros_)
    {
      ROS_ERROR("Costmap is not initialized");
      return 0;
    }

    // check if the costmap has an inflation layer
    for (std::vector<boost::shared_ptr<costmap_2d::Layer>>::const_iterator layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
         layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end();
         ++layer)
    {
      boost::shared_ptr<costmap_2d::InflationLayer> inflation_layer = boost::dynamic_pointer_cast<costmap_2d::InflationLayer>(*layer);
      if (!inflation_layer)
        continue;

      result = costMapCostToSBPLCost(inflation_layer->computeCost(costmap_ros_->getLayeredCostmap()->getCircumscribedRadius() / costmap_ros_->getCostmap()->getResolution()));
    }
    return result;
  }

  bool CustomPlanner::makePlan(const geometry_msgs::PoseStamped &start,
                                    const geometry_msgs::PoseStamped &goal,
                                    std::vector<geometry_msgs::PoseStamped> &plan)
  {
    if (!initialized_)
    {
      ROS_ERROR("Global planner is not initialized");
      return false;
    }
    
    bool do_init = false;
    if (current_env_width_ != costmap_ros_->getCostmap()->getSizeInCellsX() ||
        current_env_height_ != costmap_ros_->getCostmap()->getSizeInCellsY())
    {
      ROS_INFO("Costmap dimensions have changed from (%d x %d) to (%d x %d), reinitializing custom_planner.",
               current_env_width_, current_env_height_,
               costmap_ros_->getCostmap()->getSizeInCellsX(), costmap_ros_->getCostmap()->getSizeInCellsY());
      do_init = true;
    }
    else if (footprint_ != costmap_ros_->getRobotFootprint())
    {
      ROS_INFO("Robot footprint has changed, reinitializing custom_planner.");
      do_init = true;
    }
    else if (circumscribed_cost_ != computeCircumscribedCost())
    {
      ROS_INFO("Cost at circumscribed radius has changed, reinitializing custom_planner.");
      do_init = true;
    }

    if (do_init)
    {
      initialized_ = false;
      // delete planner_;
      // planner_ = NULL;
      // delete env_;
      // env_ = NULL;
      initialize(name_, costmap_ros_);
    }

    plan.clear();

    if(!posesOnPathWay.empty())
    {
      geometry_msgs::PoseStamped pose_start_on_path;
      Pose start_pose;
      start_pose.setX(start.pose.position.x);
      start_pose.setY(start.pose.position.y);
      start_pose.setYaw(getYaw(start.pose.orientation.x, start.pose.orientation.y, start.pose.orientation.z, start.pose.orientation.w));
      if(findNearestPoseOfPath(posesOnPathWay, start_pose, start_on_path))
      {
        ROS_INFO("[custom_planner] getting start point (%g,%g) goal point (%g,%g) start_pose_on_path (%g,%g)",
              start_pose.getX(), start_pose.getY(), goal.pose.position.x, goal.pose.position.y, start_on_path.getX(), start_on_path.getY());
        pose_start_on_path.pose.position.x = start_on_path.getX();
        pose_start_on_path.pose.position.y = start_on_path.getY();
        pose_start_on_path.pose.position.z = 0;
        pose_start_on_path.pose.orientation = tf::createQuaternionMsgFromYaw(start_on_path.getYaw());
        pose_start_on_path.header = goal.header;
      }   

      unsigned int mx_start, my_start;
      unsigned int mx_end, my_end;
      if(!costmap_ros_->getCostmap()->worldToMap( start.pose.position.x, 
                                                  start.pose.position.y, 
                                                  mx_start, my_start)

        || !costmap_ros_->getCostmap()->worldToMap(  goal.pose.position.x,  
                                                      goal.pose.position.y,  
                                                      mx_end, my_end))
      {
        ROS_ERROR("[custom_planner] can not convert world to Map 'start point' or 'goal point'");
        // return false;
      }
      unsigned char start_cost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(mx_start, my_start));
      unsigned char end_cost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(mx_end, my_end));
      if(  start_cost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || start_cost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        || end_cost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || end_cost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
      {
        ROS_WARN("[custom_planner] base_pootprint in infated obstacle ");
        // return false;
      }   
      
      ros::Time plan_time = ros::Time::now();
      // Kiểm tra xem điểm Start và End đang cách Plan bao nhiêu
      double x_Start_to_Plan = posesOnPathWay[start_on_path_index].getX() - start.pose.position.x;
      double y_Start_to_Plan = posesOnPathWay[start_on_path_index].getY() - start.pose.position.y;
      double d_Start_to_Plan = sqrt(x_Start_to_Plan*x_Start_to_Plan + y_Start_to_Plan*y_Start_to_Plan);
      
      double x_Plan_to_End = goal.pose.position.x - posesOnPathWay.back().getX();
      double y_Plan_to_End = goal.pose.position.y - posesOnPathWay.back().getY();
      double d_Plan_to_End = sqrt(x_Plan_to_End*x_Plan_to_End + y_Plan_to_End*y_Plan_to_End);
      
      // create a message for the plan
      nav_msgs::Path gui_path;
      if(d_Start_to_Plan <= 0.1 && d_Plan_to_End <= 0.1)
      {
        int gui_path_size = 2 + (int)posesOnPathWay.size() - start_on_path_index;
        gui_path.poses.resize(gui_path_size);
        gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
        gui_path.header.stamp = plan_time;

        geometry_msgs::PoseStamped pose_start;
        pose_start.header.stamp = plan_time;
        pose_start.header.frame_id = costmap_ros_->getGlobalFrameID();
        pose_start.pose.position.x = start.pose.position.x;
        pose_start.pose.position.y = start.pose.position.y;
        pose_start.pose.position.z = start.pose.position.z;
        pose_start.pose.orientation = start.pose.orientation;
        plan.push_back(pose_start);

        for (unsigned int i = start_on_path_index; i < posesOnPathWay.size(); i++)
        {
          geometry_msgs::PoseStamped pose;
          pose.header.stamp = plan_time;
          pose.header.frame_id = costmap_ros_->getGlobalFrameID();

          pose.pose.position.x = posesOnPathWay[i].getX();
          pose.pose.position.y = posesOnPathWay[i].getY();
          pose.pose.position.z = 0;
          pose.pose.orientation = tf::createQuaternionMsgFromYaw(posesOnPathWay[i].getYaw());
          plan.push_back(pose);
        }

        geometry_msgs::PoseStamped pose_goal;
        pose_goal.header.stamp = plan_time;
        pose_goal.header.frame_id = costmap_ros_->getGlobalFrameID();
        pose_goal.pose.position.x = goal.pose.position.x;
        pose_goal.pose.position.y = goal.pose.position.y;
        pose_goal.pose.position.z = goal.pose.position.z;
        pose_goal.pose.orientation = goal.pose.orientation;
        plan.push_back(pose_goal);
      }
      else if(d_Start_to_Plan > 0.1 && d_Plan_to_End <= 0.1)
      {
        vector<Pose> StartToPlanCurve;
        Pose pose_A;
        pose_A.setX(start.pose.position.x);
        pose_A.setY(start.pose.position.y);
        pose_A.setYaw(getYaw(start.pose.orientation.x, start.pose.orientation.y, start.pose.orientation.z, start.pose.orientation.w));
        if(makeCurvePlan(pose_A, posesOnPathWay[start_on_path_index],StartToPlanCurve))
        {
          int gui_path_size = 1 + (int)StartToPlanCurve.size() - 1 + (int)posesOnPathWay.size() - start_on_path_index;
          gui_path.poses.resize(gui_path_size);
          gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
          gui_path.header.stamp = plan_time;

          for (unsigned int i = 0; i < ((int)StartToPlanCurve.size() - 1); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = StartToPlanCurve[i].getX();
            pose.pose.position.y = StartToPlanCurve[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(StartToPlanCurve[i].getYaw());
            plan.push_back(pose);
          }
          for (unsigned int i = start_on_path_index; i < posesOnPathWay.size(); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = posesOnPathWay[i].getX();
            pose.pose.position.y = posesOnPathWay[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(posesOnPathWay[i].getYaw());
            plan.push_back(pose);
          }
          geometry_msgs::PoseStamped pose_goal;
          pose_goal.header.stamp = plan_time;
          pose_goal.header.frame_id = costmap_ros_->getGlobalFrameID();
          pose_goal.pose.position.x = goal.pose.position.x;
          pose_goal.pose.position.y = goal.pose.position.y;
          pose_goal.pose.position.z = goal.pose.position.z;
          pose_goal.pose.orientation = goal.pose.orientation;
          plan.push_back(pose_goal);
        }
        else
        {
          int gui_path_size = 2 + (int)posesOnPathWay.size() - start_on_path_index;
          gui_path.poses.resize(gui_path_size);
          gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
          gui_path.header.stamp = plan_time;

          geometry_msgs::PoseStamped pose_start;
          pose_start.header.stamp = plan_time;
          pose_start.header.frame_id = costmap_ros_->getGlobalFrameID();
          pose_start.pose.position.x = start.pose.position.x;
          pose_start.pose.position.y = start.pose.position.y;
          pose_start.pose.position.z = start.pose.position.z;
          pose_start.pose.orientation = start.pose.orientation;
          plan.push_back(pose_start);

          for (unsigned int i = start_on_path_index; i < posesOnPathWay.size(); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = posesOnPathWay[i].getX();
            pose.pose.position.y = posesOnPathWay[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(posesOnPathWay[i].getYaw());
            plan.push_back(pose);
          }
          geometry_msgs::PoseStamped pose_goal;
          pose_goal.header.stamp = plan_time;
          pose_goal.header.frame_id = costmap_ros_->getGlobalFrameID();
          pose_goal.pose.position.x = goal.pose.position.x;
          pose_goal.pose.position.y = goal.pose.position.y;
          pose_goal.pose.position.z = goal.pose.position.z;
          pose_goal.pose.orientation = goal.pose.orientation;
          plan.push_back(pose_goal);
        }
      }
      else if(d_Start_to_Plan <= 0.1 && d_Plan_to_End > 0.1)
      {
        vector<Pose> PlanToEndCurve;
        Pose pose_B;
        pose_B.setX(goal.pose.position.x);
        pose_B.setY(goal.pose.position.y);
        pose_B.setYaw(getYaw(goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w));
        if(makeCurvePlan(posesOnPathWay.back(), pose_B, PlanToEndCurve))
        {
          int gui_path_size = 1 + (int)PlanToEndCurve.size() - 1 + (int)posesOnPathWay.size() - start_on_path_index;
          gui_path.poses.resize(gui_path_size);
          gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
          gui_path.header.stamp = plan_time;
          
          geometry_msgs::PoseStamped pose_start;
          pose_start.header.stamp = plan_time;
          pose_start.header.frame_id = costmap_ros_->getGlobalFrameID();
          pose_start.pose.position.x = start.pose.position.x;
          pose_start.pose.position.y = start.pose.position.y;
          pose_start.pose.position.z = start.pose.position.z;
          pose_start.pose.orientation = start.pose.orientation;
          plan.push_back(pose_start);

          for (unsigned int i = start_on_path_index; i < posesOnPathWay.size(); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = posesOnPathWay[i].getX();
            pose.pose.position.y = posesOnPathWay[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(posesOnPathWay[i].getYaw());
            plan.push_back(pose);
          }
          for (unsigned int i = 1; i < (int)PlanToEndCurve.size(); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = PlanToEndCurve[i].getX();
            pose.pose.position.y = PlanToEndCurve[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(PlanToEndCurve[i].getYaw());
            plan.push_back(pose);
          }
        }
        else
        {
          int gui_path_size = 2 + (int)posesOnPathWay.size() - start_on_path_index;
          gui_path.poses.resize(gui_path_size);
          gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
          gui_path.header.stamp = plan_time;

          geometry_msgs::PoseStamped pose_start;
          pose_start.header.stamp = plan_time;
          pose_start.header.frame_id = costmap_ros_->getGlobalFrameID();
          pose_start.pose.position.x = start.pose.position.x;
          pose_start.pose.position.y = start.pose.position.y;
          pose_start.pose.position.z = start.pose.position.z;
          pose_start.pose.orientation = start.pose.orientation;
          plan.push_back(pose_start);

          for (unsigned int i = start_on_path_index; i < posesOnPathWay.size(); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = posesOnPathWay[i].getX();
            pose.pose.position.y = posesOnPathWay[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(posesOnPathWay[i].getYaw());
            plan.push_back(pose);
          }
          geometry_msgs::PoseStamped pose_goal;
          pose_goal.header.stamp = plan_time;
          pose_goal.header.frame_id = costmap_ros_->getGlobalFrameID();
          pose_goal.pose.position.x = goal.pose.position.x;
          pose_goal.pose.position.y = goal.pose.position.y;
          pose_goal.pose.position.z = goal.pose.position.z;
          pose_goal.pose.orientation = goal.pose.orientation;
          plan.push_back(pose_goal);
        }
      }
      else
      {
        vector<Pose> StartToPlanCurve;
        Pose pose_A;
        pose_A.setX(start.pose.position.x);
        pose_A.setY(start.pose.position.y);
        pose_A.setYaw(getYaw(start.pose.orientation.x, start.pose.orientation.y, start.pose.orientation.z, start.pose.orientation.w));
        vector<Pose> PlanToEndCurve;
        Pose pose_B;
        pose_B.setX(goal.pose.position.x);
        pose_B.setY(goal.pose.position.y);
        pose_B.setYaw(getYaw(goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w));
        if(makeCurvePlan(pose_A, posesOnPathWay[start_on_path_index],StartToPlanCurve) && 
           makeCurvePlan(posesOnPathWay.back(), pose_B, PlanToEndCurve))
        {
          int gui_path_size = (int)StartToPlanCurve.size() - 1 + (int)PlanToEndCurve.size() - 1 + (int)posesOnPathWay.size() - start_on_path_index;
          gui_path.poses.resize(gui_path_size);
          gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
          gui_path.header.stamp = plan_time;

          for (unsigned int i = 0; i < ((int)StartToPlanCurve.size() - 1); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = StartToPlanCurve[i].getX();
            pose.pose.position.y = StartToPlanCurve[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(StartToPlanCurve[i].getYaw());
            plan.push_back(pose);
          }
          for (unsigned int i = start_on_path_index; i < posesOnPathWay.size(); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = posesOnPathWay[i].getX();
            pose.pose.position.y = posesOnPathWay[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(posesOnPathWay[i].getYaw());
            plan.push_back(pose);
          }
          for (unsigned int i = 1; i < (int)PlanToEndCurve.size(); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = PlanToEndCurve[i].getX();
            pose.pose.position.y = PlanToEndCurve[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(PlanToEndCurve[i].getYaw());
            plan.push_back(pose);
          }
        }
        else if(makeCurvePlan(pose_A, posesOnPathWay[start_on_path_index],StartToPlanCurve))
        {
          int gui_path_size = 1 + (int)StartToPlanCurve.size() - 1 + (int)posesOnPathWay.size() - start_on_path_index;
          gui_path.poses.resize(gui_path_size);
          gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
          gui_path.header.stamp = plan_time;

          for (unsigned int i = 0; i < ((int)StartToPlanCurve.size() - 1); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = StartToPlanCurve[i].getX();
            pose.pose.position.y = StartToPlanCurve[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(StartToPlanCurve[i].getYaw());
            plan.push_back(pose);
          }
          for (unsigned int i = start_on_path_index; i < posesOnPathWay.size(); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = posesOnPathWay[i].getX();
            pose.pose.position.y = posesOnPathWay[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(posesOnPathWay[i].getYaw());
            plan.push_back(pose);
          }
          geometry_msgs::PoseStamped pose_goal;
          pose_goal.header.stamp = plan_time;
          pose_goal.header.frame_id = costmap_ros_->getGlobalFrameID();
          pose_goal.pose.position.x = goal.pose.position.x;
          pose_goal.pose.position.y = goal.pose.position.y;
          pose_goal.pose.position.z = goal.pose.position.z;
          pose_goal.pose.orientation = goal.pose.orientation;
          plan.push_back(pose_goal);
        }
        else if(makeCurvePlan(posesOnPathWay.back(), pose_B, PlanToEndCurve))
        {
          int gui_path_size = 1 + (int)PlanToEndCurve.size() - 1 + (int)posesOnPathWay.size() - start_on_path_index;
          gui_path.poses.resize(gui_path_size);
          gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
          gui_path.header.stamp = plan_time;
          
          geometry_msgs::PoseStamped pose_start;
          pose_start.header.stamp = plan_time;
          pose_start.header.frame_id = costmap_ros_->getGlobalFrameID();
          pose_start.pose.position.x = start.pose.position.x;
          pose_start.pose.position.y = start.pose.position.y;
          pose_start.pose.position.z = start.pose.position.z;
          pose_start.pose.orientation = start.pose.orientation;
          plan.push_back(pose_start);

          for (unsigned int i = start_on_path_index; i < posesOnPathWay.size(); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = posesOnPathWay[i].getX();
            pose.pose.position.y = posesOnPathWay[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(posesOnPathWay[i].getYaw());
            plan.push_back(pose);
          }
          for (unsigned int i = 1; i < (int)PlanToEndCurve.size(); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = PlanToEndCurve[i].getX();
            pose.pose.position.y = PlanToEndCurve[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(PlanToEndCurve[i].getYaw());
            plan.push_back(pose);
          }
        }
        else
        {
          int gui_path_size = 2 + (int)posesOnPathWay.size() - start_on_path_index;
          gui_path.poses.resize(gui_path_size);
          gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
          gui_path.header.stamp = plan_time;

          geometry_msgs::PoseStamped pose_start;
          pose_start.header.stamp = plan_time;
          pose_start.header.frame_id = costmap_ros_->getGlobalFrameID();
          pose_start.pose.position.x = start.pose.position.x;
          pose_start.pose.position.y = start.pose.position.y;
          pose_start.pose.position.z = start.pose.position.z;
          pose_start.pose.orientation = start.pose.orientation;
          plan.push_back(pose_start);

          for (unsigned int i = start_on_path_index; i < posesOnPathWay.size(); i++)
          {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = plan_time;
            pose.header.frame_id = costmap_ros_->getGlobalFrameID();

            pose.pose.position.x = posesOnPathWay[i].getX();
            pose.pose.position.y = posesOnPathWay[i].getY();
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(posesOnPathWay[i].getYaw());
            plan.push_back(pose);
          }
          geometry_msgs::PoseStamped pose_goal;
          pose_goal.header.stamp = plan_time;
          pose_goal.header.frame_id = costmap_ros_->getGlobalFrameID();
          pose_goal.pose.position.x = goal.pose.position.x;
          pose_goal.pose.position.y = goal.pose.position.y;
          pose_goal.pose.position.z = goal.pose.position.z;
          pose_goal.pose.orientation = goal.pose.orientation;
          plan.push_back(pose_goal);
        }
      }
      gui_path.poses = plan;
      plan_pub_.publish(gui_path);
    }
    else
    {
      ROS_ERROR("[custom_planner] posesOnPathWay is empty");
      return false;
    }
    // For deverlop (To do: improve make plan with sbpl lattice)
    /*
    else
    {
      
      ROS_INFO("[custom_planner] getting start point (%g,%g) goal point (%g,%g)",
              start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
      double theta_start = 2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
      double theta_goal = 2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

      try
      {
        int ret = env_->SetStart(start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_start);
        if (ret < 0 || planner_->set_start(ret) == 0)
        {
          ROS_ERROR("ERROR: failed to set start state\n");
          return false;
        }
      }
      catch (SBPL_Exception *e)
      {
        ROS_ERROR("SBPL encountered a fatal exception while setting the start state");
        return false;
      }

      try
      {
        int ret = env_->SetGoal(goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX(), goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY(), theta_goal);
        if (ret < 0 || planner_->set_goal(ret) == 0)
        {
          ROS_ERROR("ERROR: failed to set goal state\n");
          return false;
        }
      }
      catch (SBPL_Exception *e)
      {
        ROS_ERROR("SBPL encountered a fatal exception while setting the goal state");
        return false;
      }

      int offOnCount = 0;
      int onOffCount = 0;
      int allCount = 0;
      vector<nav2dcell_t> changedcellsV;
      
      for (unsigned int ix = 0; ix < costmap_ros_->getCostmap()->getSizeInCellsX(); ix++)
      {
        for (unsigned int iy = 0; iy < costmap_ros_->getCostmap()->getSizeInCellsY(); iy++)
        {

          unsigned char oldCost = env_->GetMapCost(ix, iy);
          unsigned char newCost = costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix, iy));

          if (oldCost == newCost)
            continue;

          allCount++;

          // first case - off cell goes on

          if ((oldCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && oldCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
              (newCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || newCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
          {
            offOnCount++;
          }

          if ((oldCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) || oldCost == costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
              (newCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) && newCost != costMapCostToSBPLCost(costmap_2d::INSCRIBED_INFLATED_OBSTACLE)))
          {
            onOffCount++;
          }
          env_->UpdateCost(ix, iy, costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix, iy)));

          nav2dcell_t nav2dcell;
          nav2dcell.x = ix;
          nav2dcell.y = iy;
          changedcellsV.push_back(nav2dcell);
        }
      }

      try
      {
        if (!changedcellsV.empty())
        {
          StateChangeQuery *scq = new LatticeSCQ(env_, changedcellsV);
          planner_->costs_changed(*scq);
          delete scq;
        }

        if (allCount > force_scratch_limit_)
          planner_->force_planning_from_scratch();
      }
      catch (SBPL_Exception *e)
      {
        ROS_ERROR("SBPL failed to update the costmap");
        return false;
      }
    
      // setting planner parameters
      ROS_DEBUG("allocated:%f, init eps:%f\n", allocated_time_, initial_epsilon_);
      planner_->set_initialsolution_eps(initial_epsilon_);
      planner_->set_search_mode(false);

      ROS_DEBUG("[custom_planner] run planner");
      vector<int> solution_stateIDs;
      int solution_cost;
      try
      {
        int ret = planner_->replan(allocated_time_, &solution_stateIDs, &solution_cost);
        if (ret)
          ROS_DEBUG("Solution is found\n");
        else
        {
          ROS_INFO("Solution not found\n");
          publishStats(solution_cost, 0, start, goal);
          return false;
        }
      }
      catch (SBPL_Exception *e)
      {
        ROS_ERROR("SBPL encountered a fatal exception while planning");
        return false;
      }

      ROS_DEBUG("size of solution=%d", (int)solution_stateIDs.size());

      vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
      try
      {
        env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
      }
      catch (SBPL_Exception *e)
      {
        ROS_ERROR("SBPL encountered a fatal exception while reconstructing the path");
        return false;
      }
      // if the plan has zero points, add a single point to make move_base happy
      if (sbpl_path.size() == 0)
      {
        EnvNAVXYTHETALAT3Dpt_t s(
            start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(),
            start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(),
            theta_start);
        sbpl_path.push_back(s);
      }

      ROS_DEBUG("Plan has %d points.\n", (int)sbpl_path.size());
      if (sbpl_path.back().x != goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX() ||
          sbpl_path.back().y != goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY() ||
          sbpl_path.back().theta != theta_goal)

      {
        EnvNAVXYTHETALAT3Dpt_t goal_stemp(goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX(),
                                          goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY(),
                                          theta_goal);
        sbpl_path.push_back(goal_stemp);
      }

      ros::Time plan_time = ros::Time::now();

      if (publish_footprint_path_)
      {
        visualization_msgs::Marker sbpl_plan_footprint;
        getFootprintList(sbpl_path, costmap_ros_->getGlobalFrameID(), sbpl_plan_footprint);
        sbpl_plan_footprint_pub_.publish(sbpl_plan_footprint);
      }

      // create a message for the plan
      nav_msgs::Path gui_path;
      gui_path.poses.resize(sbpl_path.size());
      gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
      gui_path.header.stamp = plan_time;
      for (unsigned int i = 0; i < sbpl_path.size(); i++)
      {
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = costmap_ros_->getGlobalFrameID();

        pose.pose.position.x = sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
        pose.pose.position.y = sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
        pose.pose.position.z = start.pose.position.z;

        tf2::Quaternion temp;
        temp.setRPY(0, 0, sbpl_path[i].theta);
        pose.pose.orientation.x = temp.getX();
        pose.pose.orientation.y = temp.getY();
        pose.pose.orientation.z = temp.getZ();
        pose.pose.orientation.w = temp.getW();

        plan.push_back(pose);

        gui_path.poses[i] = plan[i];
      }
      plan_pub_.publish(gui_path);
      publishStats(solution_cost, sbpl_path.size(), start, goal);
    }
    */
    return true;
  }

  void CustomPlanner::getFootprintList(const std::vector<EnvNAVXYTHETALAT3Dpt_t> &sbpl_path,
                                            const std::string &path_frame_id, visualization_msgs::Marker &ma)
  {
    ma.header.frame_id = path_frame_id;
    ma.header.stamp = ros::Time();
    ma.ns = "sbpl_robot_footprint";
    ma.id = 0;
    ma.type = visualization_msgs::Marker::LINE_LIST;
    ma.action = visualization_msgs::Marker::ADD;
    ma.scale.x = 0.05;
    ma.color.a = 1.0;
    ma.color.r = 0.0;
    ma.color.g = 0.0;
    ma.color.b = 1.0;
    ma.pose.orientation.w = 1.0;

    for (unsigned int i = 0; i < sbpl_path.size(); i = i + visualizer_skip_poses_)
    {
      std::vector<geometry_msgs::Point> transformed_rfp;
      geometry_msgs::Pose robot_pose;
      robot_pose.position.x = sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
      ;
      robot_pose.position.y = sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
      ;
      robot_pose.position.z = 0.0;
      tf::Quaternion quat;
      quat.setRPY(0.0, 0.0, sbpl_path[i].theta);
      tf::quaternionTFToMsg(quat, robot_pose.orientation);
      transformFootprintToEdges(robot_pose, footprint_, transformed_rfp);

      for (auto &point : transformed_rfp)
        ma.points.push_back(point);
    }
  }

  void CustomPlanner::transformFootprintToEdges(const geometry_msgs::Pose &robot_pose,
                                                     const std::vector<geometry_msgs::Point> &footprint,
                                                     std::vector<geometry_msgs::Point> &out_footprint)
  {
    out_footprint.resize(2 * footprint.size());
    double yaw = tf::getYaw(robot_pose.orientation);
    for (unsigned int i = 0; i < footprint.size(); i++)
    {
      out_footprint[2 * i].x = robot_pose.position.x + cos(yaw) * footprint[i].x - sin(yaw) * footprint[i].y;
      out_footprint[2 * i].y = robot_pose.position.y + sin(yaw) * footprint[i].x + cos(yaw) * footprint[i].y;
      if (i == 0)
      {
        out_footprint.back().x = out_footprint[i].x;
        out_footprint.back().y = out_footprint[i].y;
      }
      else
      {
        out_footprint[2 * i - 1].x = out_footprint[2 * i].x;
        out_footprint[2 * i - 1].y = out_footprint[2 * i].y;
      }
    }
  }
  

  inline double CustomPlanner::getYaw(double x, double y, double z, double w){
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
  }

  bool CustomPlanner::loadPathwayData(const string& filename)
  {
    bool result = false;
    // Check if saved path file is existed
    ifstream file;
    file.open(filename);
    if(file){
        pathway->LoadPathFromFile(filename);
        pathway->syncPosesAndPath();
        posesOnPathWay = pathway->getPosesOnPath();
        result = true;
    }
    else
        result = false;
    return result;
  }  

  bool CustomPlanner::findNearestPoseOfPath(vector<Pose>& posesOnPathWay, Pose& PoseToCheck, Pose& PoseResult)
  {
    bool result = true;
    start_on_path_index = 0;    
    uint16_t start_on_path_index_tmp = 0;
    uint16_t it = 0;
    if(!posesOnPathWay.empty()){
      double minDistance = sqrt(pow((posesOnPathWay[0].getX() - PoseToCheck.getX()),2) + 
      pow((posesOnPathWay[0].getY() - PoseToCheck.getY()),2));
      for(auto &Pose: posesOnPathWay)
      {        
        double dx = Pose.getX() - PoseToCheck.getX();
        double dy = Pose.getY() - PoseToCheck.getY();
        double distance = sqrt(dx*dx + dy*dy);
        if(distance<minDistance)
        {
          minDistance = distance;
          PoseResult = Pose;
          start_on_path_index = it;
        }      
        it++;  
      }
      start_on_path_index_tmp = start_on_path_index;
      ROS_WARN("[custom_planner][findNearestPoseOfPath] start_on_path_index_tmp: %d, start_on_path_index: %d", start_on_path_index_tmp, start_on_path_index);
      double SumDistanceCheck = 0;
      if(start_on_path_index==0)
      {
        double SumDistanceCheck2 = 0;
        bool computeSegment1Good = false;
        bool computeSegment2Good = false;
        uint16_t start_on_path_index_1 = start_on_path_index_tmp;
        uint16_t start_on_path_index_2 = start_on_path_index_tmp;
        double deltaAngle_1_min = computeDeltaAngle(PoseToCheck, posesOnPathWay[0]);
        double deltaAngle_2_min = 10;
        double deltaAngleToRotate_1 = fabs(PoseToCheck.getYaw()-posesOnPathWay[0].getYaw());
        if(deltaAngle_1_min <= 0.872664626) // <= 50 degree
        {
          if(deltaAngleToRotate_1 <= 0.872664626) // <= 50 degree
          {
            // computeSegment1Good = true;
            start_on_path_index_1 = 0;
          }
        }
        for(int i = 0; i< (int)posesOnPathWay.size(); i++)
        {
          if(i!=0)
          {
            double dx = posesOnPathWay[i].getX() - posesOnPathWay[i-1].getX();
            double dy = posesOnPathWay[i].getY() - posesOnPathWay[i-1].getY();
            SumDistanceCheck2 += sqrt(dx*dx + dy*dy);
          }
          if(SumDistanceCheck2<2.5) //search distance < 2 m
          {
            double deltaAngle_2 = computeDeltaAngle(PoseToCheck, posesOnPathWay[i]);
            if(deltaAngle_2 <= 0.7853981634) // <= 45 degree
            {
              double deltaAngleToRotate_2 = fabs(PoseToCheck.getYaw()-posesOnPathWay[i].getYaw());
              if(deltaAngleToRotate_2 <= 1.5707963268) // <= 90 degree
              {
                if(deltaAngle_2<deltaAngle_2_min)
                {          
                  deltaAngle_2_min = deltaAngle_2;
                  start_on_path_index_2 = i;
                  computeSegment2Good = true;
                }
              }
            }
          }
        }
        SumDistanceCheck2 = 0;
        if(computeSegment1Good==true&&computeSegment2Good==true)
        {
          if(deltaAngle_2_min<deltaAngle_1_min)
          {
            start_on_path_index = start_on_path_index_2;
          }
          else if(deltaAngle_2_min==deltaAngle_1_min)
          {
            start_on_path_index = start_on_path_index_2;
          }
          else
          {
            start_on_path_index = start_on_path_index_1;
          }
        }
        else if(computeSegment1Good==true)
        {
          start_on_path_index = start_on_path_index_1;
        }
        else if(computeSegment2Good==true)
        {
          start_on_path_index = start_on_path_index_2;
        }
        else
        {
          start_on_path_index = start_on_path_index_tmp+5;
        }
        ROS_WARN("[custom_planner][findNearestPoseOfPath] TH1 start_on_path_index_1: %d, start_on_path_index_2: %d, start_on_path_index: %d", 
        start_on_path_index_1, start_on_path_index_2, start_on_path_index);
      }
      else if(start_on_path_index!=((int)posesOnPathWay.size()-1))
      {
        double SumDistanceCheck1 = 0;
        double SumDistanceCheck2 = 0;
        bool computeSegment1Good = false;
        bool computeSegment2Good = false;
        uint16_t start_on_path_index_1 = start_on_path_index_tmp;
        uint16_t start_on_path_index_2 = start_on_path_index_tmp;
        double deltaAngle_1_min = 10;
        double deltaAngle_2_min = 10;

        for(int i = start_on_path_index; i >= 0; i--)
        {
          if(i!= start_on_path_index)
          {
            double dx = posesOnPathWay[i].getX() - posesOnPathWay[i+1].getX();
            double dy = posesOnPathWay[i].getY() - posesOnPathWay[i+1].getY();
            SumDistanceCheck1 += sqrt(dx*dx + dy*dy);
          }          
          if(SumDistanceCheck1<2) //search distance < 2 m
          {
            double deltaAngle_1 = computeDeltaAngle(PoseToCheck, posesOnPathWay[i]);
            if(deltaAngle_1 <= 0.7853981634) // <= 45 degree
            {
              double deltaAngleToRotate_1 = fabs(PoseToCheck.getYaw()-posesOnPathWay[i].getYaw());
              if(deltaAngleToRotate_1 <= 1.5707963268) // <= 90 degree
              {
                // ROS_WARN("index: %d deltaAngle_1: %f",i,deltaAngle_1); 
                if(deltaAngle_1<deltaAngle_1_min)
                {                                   
                  deltaAngle_1_min = deltaAngle_1;
                  start_on_path_index_1 = i;
                  // computeSegment1Good = true;
                }
              }
            }
          }
          else
          {
            break;
          }          
        }
        SumDistanceCheck1 = 0;
        for(int i = start_on_path_index; i<(int)posesOnPathWay.size(); i++)
        {
          if(i!=start_on_path_index)
          {
            double dx = posesOnPathWay[i].getX() - posesOnPathWay[i-1].getX();
            double dy = posesOnPathWay[i].getY() - posesOnPathWay[i-1].getY();
            SumDistanceCheck2 += sqrt(dx*dx + dy*dy);
          }
          if(SumDistanceCheck2<2.5) //search distance < 2 m
          {
            double deltaAngle_2 = computeDeltaAngle(PoseToCheck, posesOnPathWay[i]);
            if(deltaAngle_2 <= 0.7853981634) // <= 45 degree
            {
              double deltaAngleToRotate_2 = fabs(PoseToCheck.getYaw()-posesOnPathWay[i].getYaw());
              if(deltaAngleToRotate_2 <= 1.5707963268) // <= 90 degree
              {
                // ROS_WARN("index: %d deltaAngle_2: %f",i,deltaAngle_2);
                if(deltaAngle_2<deltaAngle_2_min)
                {                  
                  deltaAngle_2_min = deltaAngle_2;
                  start_on_path_index_2 = i;
                  computeSegment2Good = true;
                }
              }
            }
          }
          else
          {
            break;
          }
        }
        SumDistanceCheck2 = 0;
        if(computeSegment1Good==true&&computeSegment2Good==true)
        {
          if(deltaAngle_2_min<deltaAngle_1_min)
          {
            start_on_path_index = start_on_path_index_2;
            // ROS_WARN("1 good and 2 good but 2 is better, start_on_path_index: %d",start_on_path_index);
          }
          else if(deltaAngle_2_min==deltaAngle_1_min)
          {
            start_on_path_index = start_on_path_index_2;
            // ROS_WARN("1 good and 2 good, 1 is equal to 2, start_on_path_index: %d",start_on_path_index);
          }
          else
          {
            // ROS_WARN("1 good and 2 good but 1 is better, start_on_path_index: %d",start_on_path_index);
            start_on_path_index = start_on_path_index_1;
          }
        }
        else if(computeSegment1Good==true)
        {
          // ROS_WARN("1 good");
          start_on_path_index = start_on_path_index_1;
        }
        else if(computeSegment2Good==true)
        {
          // ROS_WARN("2 good");
          start_on_path_index = start_on_path_index_2;
        }
        else
        {
          // ROS_WARN("both is not good");
          start_on_path_index = start_on_path_index_tmp+5;
        }
        ROS_WARN("[custom_planner][findNearestPoseOfPath] TH2 start_on_path_index_1: %d, start_on_path_index_2: %d, start_on_path_index: %d", 
        start_on_path_index_1, start_on_path_index_2, start_on_path_index);
        ROS_WARN("[custom_planner][findNearestPoseOfPath] TH2  deltaAngle_1_min: %f, deltaAngle_2_min: %f", 
        deltaAngle_1_min, deltaAngle_2_min);
      }      
      if(start_on_path_index==((int)posesOnPathWay.size()-1)||start_on_path_index==start_on_path_index_tmp) // find to last element or index is not change
      {
        start_on_path_index = start_on_path_index_tmp; // set to closet pose
      }
      // ROS_WARN("[custom_planner][findNearestPoseOfPath] Final start_on_path_index: %d", start_on_path_index);
      result = true;
    }
    else
    {
      result = false;
    }

    return result;
  }

  void CustomPlanner::order_msg_handle(const vda5050_msgs::Order::ConstPtr& msg)
  {
    uint8_t status;
    string message;
    if(makePlanWithOrder(*msg, status, message, false))
    {
      ROS_INFO("Success to make plan with order");
      test_print_plan_result();
    }
    else if(makePlanWithOrder(*msg, status, message, true))
    {
      ROS_INFO("Success to make plan with order");
      test_print_plan_result();
    }
    else
    {
      ROS_WARN("%s",message.c_str());
    }
  }

  bool CustomPlanner::HandleSetPlanWithOrder(
        custom_planner::PlanWithOrder::Request& request,
        custom_planner::PlanWithOrder::Response& response)
  {
    uint8_t status;
    string message;
    if(makePlanWithOrder(request.order, status, message, false))
    {
      response.status = status;
      response.message = message;
      response.success = true;
    }
    else if(makePlanWithOrder(request.order, status, message, true))
    {
      response.status = status;
      response.message = message;
      response.success = true;
    }
    else
    {
      response.status = status;
      response.message = message;
      response.success = false;
    }
    return true;
  }

  bool CustomPlanner::makePlanWithOrder(vda5050_msgs::Order msg, uint8_t& status, string& message, bool is_move_backward)
  {
    orderNodes.clear();
    posesOnPathWay.clear();
    if((int)msg.nodes.size()==0&&(int)msg.edges.size()==0)
    {
      status = 1;
      message = "Nodes and Edges in Order is empty";
      return false;
    }
    for(int i=0;i<(int)msg.nodes.size();i++)
    {
      orderNodes[msg.nodes[i].nodeId] = {msg.nodes[i].nodeId, msg.nodes[i].sequenceId, 
      (double)msg.nodes[i].nodePosition.x, (double)msg.nodes[i].nodePosition.y, (double)msg.nodes[i].nodePosition.theta};
    }
    for(int i=0;i<(int)msg.edges.size();i++)
    {      
      auto start_nodeId_it = orderNodes.find(msg.edges[i].startNodeId);
      auto end_nodeId_it = orderNodes.find(msg.edges[i].endNodeId);
      if(start_nodeId_it!=orderNodes.end()&&end_nodeId_it!=orderNodes.end())
      {
        vector<Pose> posesOnEdge;
        vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> control_points;
        std::vector<double> knot_vector;
        std::vector<double> weight_vector;
        int degree = 0;
        int order = 0;
        control_points.reserve(msg.edges[i].trajectory.controlPoints.size());
        knot_vector.reserve(msg.edges[i].trajectory.knotVector.size());
        weight_vector.reserve(msg.edges[i].trajectory.controlPoints.size());
        for(int j = 0;j<(int)msg.edges[i].trajectory.controlPoints.size();j++)
        {
          control_points.push_back(Eigen::Vector3d(msg.edges[i].trajectory.controlPoints[j].x, msg.edges[i].trajectory.controlPoints[j].y, 0));
          weight_vector.push_back(msg.edges[i].trajectory.controlPoints[j].weight);
        }
        for(int k = 0 ;k < (int)msg.edges[i].trajectory.knotVector.size();k++)
        {
          knot_vector.push_back(msg.edges[i].trajectory.knotVector[k]);
        }
        degree = (int)msg.edges[i].trajectory.degree;
        if(curveIsValid(degree, knot_vector, control_points))
        {
          double t_intervel = 0.01;  
          order = degree + 1;
          input_spline_inf->control_point.clear();
          input_spline_inf->knot_vector.clear();
          input_spline_inf->weight.clear();  
          CurveDesign->ReadSplineInf(input_spline_inf, order, control_points, knot_vector);
          CurveDesign->ReadSplineInf(input_spline_inf, weight_vector, false);
          for(double u_test = 0; u_test <= 1; u_test += t_intervel)
          {  
            geometry_msgs::Point curve_point;
            curve_point = CurveDesign->CalculateCurvePoint(input_spline_inf, u_test, true);
            if(!std::isnan(curve_point.x)&&!std::isnan(curve_point.y))
            posesOnEdge.push_back(Pose(curve_point.x, curve_point.y, 0));
            // ROS_INFO("curve_point: %f, %f   at u: %f",curve_point.x, curve_point.y, u_test);
          }
          if(!isThetaValid(orderNodes[msg.edges[i].startNodeId].theta)&&!isThetaValid(orderNodes[msg.edges[i].endNodeId].theta))
          {
            // if startNode of this edge and start element of posesOnEdge are the different point
            if(orderNodes[msg.edges[i].startNodeId].position_x!=posesOnEdge.front().getX()||
              orderNodes[msg.edges[i].startNodeId].position_y!=posesOnEdge.front().getY())
            {
              posesOnEdge.insert(posesOnEdge.begin(), Pose(orderNodes[msg.edges[i].startNodeId].position_x, orderNodes[msg.edges[i].startNodeId].position_y, 0.0123443210));              
            }
            // if endNode of this edge and end element of posesOnEdge are the different point
            if(orderNodes[msg.edges[i].endNodeId].position_x!=posesOnEdge.back().getX()||
              orderNodes[msg.edges[i].endNodeId].position_y!=posesOnEdge.back().getY())
            {
              posesOnEdge.insert(posesOnEdge.end(), Pose(orderNodes[msg.edges[i].endNodeId].position_x, orderNodes[msg.edges[i].endNodeId].position_y, 0.0123443210));              
            }
            if(!posesOnPathWay.empty())
            {
              if(computeDeltaAngleStartNode(posesOnPathWay.back().getYaw(), posesOnEdge.front(), posesOnEdge[1]) <= 0.872664626)
              {
                setYawAllPosesOnEdge(posesOnEdge, false);
              }
              else if(computeDeltaAngleStartNode(posesOnPathWay.back().getYaw(), posesOnEdge.front(), posesOnEdge[1]) >= 2.2689280276)
              {
                setYawAllPosesOnEdge(posesOnEdge, true);
              }
              else
              {
                ROS_WARN("Trajectory of Edge: %s, startNode: %s, endNode: %s is not good", 
                msg.edges[i].edgeId.c_str(), msg.edges[i].startNodeId.c_str(),  msg.edges[i].endNodeId.c_str());
                status = 3;
                message = "Trajectory of Edge: " + msg.edges[i].edgeId + ", startNode: " + msg.edges[i].startNodeId.c_str() +                   
                ", endNode: " + msg.edges[i].endNodeId.c_str() + " is not good";
                return false;
                break;
              }
            }
            else
            {
              if(is_move_backward==false)
              {
                setYawAllPosesOnEdge(posesOnEdge, false);
              }
              else{
                setYawAllPosesOnEdge(posesOnEdge, true);
              }
            }
            if(!posesOnPathWay.empty())  // posesOnPathWay has datas 
            {
              if(posesOnEdge.front().getX()==posesOnPathWay.back().getX()&&
                  posesOnEdge.front().getY()==posesOnPathWay.back().getY())
              {
                if(i!=1){ // don't check angle of edge 1
                  if(computeDeltaAngleStartNode(posesOnPathWay.back().getYaw(), posesOnEdge.front().getYaw(), posesOnEdge.front()) <= 0.872664626) // <= 50 degree
                  {
                      // if yaw angle of the end pose in posesOnPathWay is default, set it to yaw angle of start pose in posesOnEdge
                      posesOnPathWay.back().setYaw(posesOnEdge.front().getYaw());
                      posesOnPathWay.insert(posesOnPathWay.end(), posesOnEdge.begin()+1, posesOnEdge.end());                
                  }
                  else
                  {
                    ROS_WARN("Trajectory of Edge: %s, startNode: %s, endNode: %s is not good", 
                    msg.edges[i].edgeId.c_str(), msg.edges[i].startNodeId.c_str(),  msg.edges[i].endNodeId.c_str());
                    status = 3;
                    message = "Trajectory of Edge: " + msg.edges[i].edgeId + ", startNode: " + msg.edges[i].startNodeId.c_str() +                   
                    ", endNode: " + msg.edges[i].endNodeId.c_str() + " is not good";
                    return false;
                    break;                  
                  }
                }
                else
                {
                  // if yaw angle of the end pose in posesOnPathWay is default, set it to yaw angle of start pose in posesOnEdge
                  posesOnPathWay.back().setYaw(posesOnEdge.front().getYaw());
                  posesOnPathWay.insert(posesOnPathWay.end(), posesOnEdge.begin()+1, posesOnEdge.end());
                }
              }
              else
              {
                ROS_WARN("Trajectory of Edge: %s. startNode: %s has posision invalid", 
                msg.edges[i].edgeId.c_str(), msg.edges[i].startNodeId.c_str());
                status = 3;
                message = "Trajectory of Edge: " + msg.edges[i].edgeId + ", startNode: " + msg.edges[i].startNodeId.c_str() +                   
                ", endNode: " + msg.edges[i].endNodeId.c_str() + " is not good";
                return false;
                break;                
              }
            }
            else // posesOnPathWay is empty
            {
              posesOnPathWay.insert(posesOnPathWay.end(), posesOnEdge.begin(), posesOnEdge.end());
            }      
          }
          else if(!isThetaValid(orderNodes[msg.edges[i].startNodeId].theta)&&isThetaValid(orderNodes[msg.edges[i].endNodeId].theta))
          {
            // if startNode of this edge and start element of posesOnEdge are the different point
            if(orderNodes[msg.edges[i].startNodeId].position_x!=posesOnEdge.front().getX()||
              orderNodes[msg.edges[i].startNodeId].position_y!=posesOnEdge.front().getY())
            {
              posesOnEdge.insert(posesOnEdge.begin(), Pose(orderNodes[msg.edges[i].startNodeId].position_x, orderNodes[msg.edges[i].startNodeId].position_y, 0.0123443210));              
            }
            // if endNode of this edge and end element of posesOnEdge are the different point
            if(orderNodes[msg.edges[i].endNodeId].position_x!=posesOnEdge.back().getX()||
              orderNodes[msg.edges[i].endNodeId].position_y!=posesOnEdge.back().getY())
            {
              posesOnEdge.insert(posesOnEdge.end(), 
              Pose(orderNodes[msg.edges[i].endNodeId].position_x, orderNodes[msg.edges[i].endNodeId].position_y, orderNodes[msg.edges[i].endNodeId].theta));              
            }
            if(i==((int)msg.edges.size()-1))
            {
              if(computeDeltaAngleEndNode(orderNodes[msg.edges[i].endNodeId].theta, posesOnEdge.back(), posesOnEdge[posesOnEdge.size()-2]) <= 1.5707963268) // <= 90 degree
              {
                setYawAllPosesOnEdge(posesOnEdge, false);
                posesOnEdge.back().setYaw(orderNodes[msg.edges[i].endNodeId].theta); // set yaw angle of the end pose to endNode theta
              }
              else
              {
                setYawAllPosesOnEdge(posesOnEdge, true);
                posesOnEdge.back().setYaw(orderNodes[msg.edges[i].endNodeId].theta); // set yaw angle of the end pose to endNode theta
              }
            }
            else
            {            
              if(computeDeltaAngleEndNode(orderNodes[msg.edges[i].endNodeId].theta, posesOnEdge.back(), posesOnEdge[posesOnEdge.size()-2]) <= 0.872664626) // <= 50 degree
              {
                setYawAllPosesOnEdge(posesOnEdge, false);
                posesOnEdge.back().setYaw(orderNodes[msg.edges[i].endNodeId].theta); // set yaw angle of the end pose to endNode theta
              }
              else if(computeDeltaAngleEndNode(orderNodes[msg.edges[i].endNodeId].theta, posesOnEdge.back(), posesOnEdge[posesOnEdge.size()-2]) >= 2.2689280276) // >= 130 degree
              {
                setYawAllPosesOnEdge(posesOnEdge, true);
                posesOnEdge.back().setYaw(orderNodes[msg.edges[i].endNodeId].theta); // set yaw angle of the end pose to endNode theta
              }
              else
              {
                ROS_WARN("Trajectory of Edge: %s, startNode: %s, endNode: %s is not good", 
                msg.edges[i].edgeId.c_str(), msg.edges[i].startNodeId.c_str(),  msg.edges[i].endNodeId.c_str());
                status = 3;
                message = "Trajectory of Edge: " + msg.edges[i].edgeId + ", startNode: " + msg.edges[i].startNodeId.c_str() +                   
                ", endNode: " + msg.edges[i].endNodeId.c_str() + " is not good";
                return false;
                break;                
              }
            }            
            if(!posesOnPathWay.empty())  // posesOnPathWay has datas 
            {
              if(posesOnEdge.front().getX()==posesOnPathWay.back().getX()&&
                  posesOnEdge.front().getY()==posesOnPathWay.back().getY())
              {
                if(computeDeltaAngleStartNode(posesOnPathWay.back().getYaw(), posesOnEdge.front().getYaw(), posesOnEdge.front()) <= 0.872664626) // <= 50 degree
                {
                    // if yaw angle of the end pose in posesOnPathWay is default, set it to yaw angle of start pose in posesOnEdge
                    posesOnPathWay.back().setYaw(posesOnEdge.front().getYaw());
                    posesOnPathWay.insert(posesOnPathWay.end(), posesOnEdge.begin()+1, posesOnEdge.end());                
                }
                else
                {
                  ROS_WARN("Trajectory of Edge: %s, startNode: %s, endNode: %s is not good", 
                  msg.edges[i].edgeId.c_str(), msg.edges[i].startNodeId.c_str(),  msg.edges[i].endNodeId.c_str());
                  status = 3;
                  message = "Trajectory of Edge: " + msg.edges[i].edgeId + ", startNode: " + msg.edges[i].startNodeId.c_str() +                   
                  ", endNode: " + msg.edges[i].endNodeId.c_str() + " is not good";
                  return false;
                  break;                  
                }
              }
              else
              {
                ROS_WARN("Trajectory of Edge: %s. startNode: %s has posision invalid", 
                msg.edges[i].edgeId.c_str(), msg.edges[i].startNodeId.c_str());
                status = 3;
                message = "Trajectory of Edge: " + msg.edges[i].edgeId + ", startNode: " + msg.edges[i].startNodeId.c_str() +                   
                ", endNode: " + msg.edges[i].endNodeId.c_str() + " is not good";
                return false;
                break;                
              }
            }
            else // posesOnPathWay is empty
            {
              posesOnPathWay.insert(posesOnPathWay.end(), posesOnEdge.begin(), posesOnEdge.end());
            }  
          }
          else if(isThetaValid(orderNodes[msg.edges[i].startNodeId].theta)&&!isThetaValid(orderNodes[msg.edges[i].endNodeId].theta))
          { 
            // if startNode of this edge and start element of posesOnEdge are the different point
            if(orderNodes[msg.edges[i].startNodeId].position_x!=posesOnEdge.front().getX()||
              orderNodes[msg.edges[i].startNodeId].position_y!=posesOnEdge.front().getY())
            {
              posesOnEdge.insert(posesOnEdge.begin(), 
              Pose(orderNodes[msg.edges[i].startNodeId].position_x, orderNodes[msg.edges[i].startNodeId].position_y, orderNodes[msg.edges[i].startNodeId].theta));              
            }
            // if endNode of this edge and end element of posesOnEdge are the different point
            if(orderNodes[msg.edges[i].endNodeId].position_x!=posesOnEdge.back().getX()||
              orderNodes[msg.edges[i].endNodeId].position_y!=posesOnEdge.back().getY())
            {
              posesOnEdge.insert(posesOnEdge.end(), 
              Pose(orderNodes[msg.edges[i].endNodeId].position_x, orderNodes[msg.edges[i].endNodeId].position_y, 0.0123443210));          
            }
            if(computeDeltaAngleStartNode(orderNodes[msg.edges[i].startNodeId].theta, posesOnEdge.front(), posesOnEdge[1]) <= 0.872664626) // <= 50 degree)
            {
              setYawAllPosesOnEdge(posesOnEdge, false);
              posesOnEdge.front().setYaw(orderNodes[msg.edges[i].startNodeId].theta); // set yaw angle of the start pose to startNode theta
            }
            else if(computeDeltaAngleStartNode(orderNodes[msg.edges[i].startNodeId].theta, posesOnEdge.front(), posesOnEdge[1]) >= 2.2689280276) // >= 130 degree
            {
              setYawAllPosesOnEdge(posesOnEdge, true);
              posesOnEdge.front().setYaw(orderNodes[msg.edges[i].startNodeId].theta); // set yaw angle of the start pose to startNode theta
            }
            else
            {
              ROS_WARN("Trajectory of Edge: %s, startNode: %s, endNode: %s is not good", 
              msg.edges[i].edgeId.c_str(), msg.edges[i].startNodeId.c_str(),  msg.edges[i].endNodeId.c_str());
              status = 3;
              message = "Trajectory of Edge: " + msg.edges[i].edgeId + ", startNode: " + msg.edges[i].startNodeId.c_str() +                   
              ", endNode: " + msg.edges[i].endNodeId.c_str() + " is not good";
              return false;
              break;              
            }
            if(!posesOnPathWay.empty())  // posesOnPathWay has datas 
            {
              if(posesOnEdge.front().getX()==posesOnPathWay.back().getX()&&
                  posesOnEdge.front().getY()==posesOnPathWay.back().getY())
              {
                if(computeDeltaAngleStartNode(posesOnPathWay.back().getYaw(), posesOnEdge.front().getYaw(), posesOnEdge.front()) <= 0.872664626) // <= 50 degree
                {
                    // if yaw angle of the end pose in posesOnPathWay is default, set it to yaw angle of start pose in posesOnEdge
                    posesOnPathWay.back().setYaw(posesOnEdge.front().getYaw());
                    posesOnPathWay.insert(posesOnPathWay.end(), posesOnEdge.begin()+1, posesOnEdge.end());                
                }
                else
                {
                  ROS_WARN("Trajectory of Edge: %s, startNode: %s, endNode: %s is not good", 
                  msg.edges[i].edgeId.c_str(), msg.edges[i].startNodeId.c_str(),  msg.edges[i].endNodeId.c_str());
                  status = 3;
                  message = "Trajectory of Edge: " + msg.edges[i].edgeId + ", startNode: " + msg.edges[i].startNodeId.c_str() +                   
                  ", endNode: " + msg.edges[i].endNodeId.c_str() + " is not good";
                  return false;
                  break;                  
                }
              }
              else
              {
                ROS_WARN("Trajectory of Edge: %s. startNode: %s has posision invalid", 
                msg.edges[i].edgeId.c_str(), msg.edges[i].startNodeId.c_str());
                status = 3;
                message = "Trajectory of Edge: " + msg.edges[i].edgeId + ", startNode: " + msg.edges[i].startNodeId.c_str() +                   
                ", endNode: " + msg.edges[i].endNodeId.c_str() + " is not good";
                return false;
                break;                
              }
            }
            else // posesOnPathWay is empty
            {
              posesOnPathWay.insert(posesOnPathWay.end(), posesOnEdge.begin(), posesOnEdge.end());
            } 
          }
          else // startNode and endNode have valid theta
          {
            // if startNode of this edge and start element of posesOnEdge are the different point
            if(orderNodes[msg.edges[i].startNodeId].position_x!=posesOnEdge.front().getX()||
              orderNodes[msg.edges[i].startNodeId].position_y!=posesOnEdge.front().getY())
            {
              posesOnEdge.insert(posesOnEdge.begin(), 
              Pose(orderNodes[msg.edges[i].startNodeId].position_x, orderNodes[msg.edges[i].startNodeId].position_y, orderNodes[msg.edges[i].startNodeId].theta));              
            }
            // if endNode of this edge and end element of posesOnEdge are the different point
            if(orderNodes[msg.edges[i].endNodeId].position_x!=posesOnEdge.back().getX()||
              orderNodes[msg.edges[i].endNodeId].position_y!=posesOnEdge.back().getY())
            {
              posesOnEdge.insert(posesOnEdge.end(), 
              Pose(orderNodes[msg.edges[i].endNodeId].position_x, orderNodes[msg.edges[i].endNodeId].position_y, orderNodes[msg.edges[i].endNodeId].theta));          
            }
            // DeltaAngleStart <= 50 degree and DeltaAngleEnd <= 50 degree
            if(computeDeltaAngleStartNode(orderNodes[msg.edges[i].startNodeId].theta, posesOnEdge.front(), posesOnEdge[1]) <= 0.872664626 &&
              computeDeltaAngleEndNode(orderNodes[msg.edges[i].endNodeId].theta, posesOnEdge.back(), posesOnEdge[posesOnEdge.size()-2]) <= 0.872664626)
            {
              setYawAllPosesOnEdge(posesOnEdge, false);
              posesOnEdge.front().setYaw(orderNodes[msg.edges[i].startNodeId].theta); // set yaw angle of the start pose to startNode theta
              posesOnEdge.back().setYaw(orderNodes[msg.edges[i].endNodeId].theta); // set yaw angle of the end pose to endNode theta
            }
            // DeltaAngleStart >= 130 degree and DeltaAngleEnd >= 130 degree
            else if(computeDeltaAngleStartNode(orderNodes[msg.edges[i].startNodeId].theta, posesOnEdge.front(), posesOnEdge[1]) >= 2.2689280276 &&
                    computeDeltaAngleEndNode(orderNodes[msg.edges[i].endNodeId].theta, posesOnEdge.back(), posesOnEdge[posesOnEdge.size()-2]) >= 2.2689280276)
            {
              setYawAllPosesOnEdge(posesOnEdge, true);
              posesOnEdge.front().setYaw(orderNodes[msg.edges[i].startNodeId].theta); // set yaw angle of the start pose to startNode theta
              posesOnEdge.back().setYaw(orderNodes[msg.edges[i].endNodeId].theta); // set yaw angle of the end pose to endNode theta
            }
            if(!posesOnPathWay.empty())  // posesOnPathWay has datas 
            {
              if(posesOnEdge.front().getX()==posesOnPathWay.back().getX()&&
                  posesOnEdge.front().getY()==posesOnPathWay.back().getY())
              {
                if(computeDeltaAngleStartNode(posesOnPathWay.back().getYaw(), posesOnEdge.front().getYaw(), posesOnEdge.front()) <= 0.872664626) // <= 50 degree
                {
                    // if yaw angle of the end pose in posesOnPathWay is default, set it to yaw angle of start pose in posesOnEdge
                    posesOnPathWay.back().setYaw(posesOnEdge.front().getYaw());
                    posesOnPathWay.insert(posesOnPathWay.end(), posesOnEdge.begin()+1, posesOnEdge.end());                
                }
                else
                {
                  ROS_WARN("Trajectory of Edge: %s, startNode: %s, endNode: %s is not good", 
                  msg.edges[i].edgeId.c_str(), msg.edges[i].startNodeId.c_str(),  msg.edges[i].endNodeId.c_str());
                  status = 3;
                  message = "Trajectory of Edge: " + msg.edges[i].edgeId + ", startNode: " + msg.edges[i].startNodeId.c_str() +                   
                  ", endNode: " + msg.edges[i].endNodeId.c_str() + " is not good";
                  return false;
                  break;                  
                }
              }
              else
              {
                ROS_WARN("Trajectory of Edge: %s. startNode: %s has posision invalid", 
                msg.edges[i].edgeId.c_str(), msg.edges[i].startNodeId.c_str());
                status = 3;
                message = "Trajectory of Edge: " + msg.edges[i].edgeId + ", startNode: " + msg.edges[i].startNodeId.c_str() +                   
                ", endNode: " + msg.edges[i].endNodeId.c_str() + " is not good";
                return false;
                break;                
              }
            }
            else // posesOnPathWay is empty
            {
              posesOnPathWay.insert(posesOnPathWay.end(), posesOnEdge.begin(), posesOnEdge.end());
            }
          }
        }
        else{
          ROS_WARN("Trajectory of Edge: %s, startNodeId: %s, endNodeId: %s is invalid", msg.edges[i].edgeId.c_str(), 
          msg.edges[i].startNodeId.c_str(), msg.edges[i].endNodeId.c_str());
          status = 2;
          message = "Trajectory of Edge: " + msg.edges[i].edgeId + ", startNode: " + msg.edges[i].startNodeId.c_str() +                   
          ", endNode: " + msg.edges[i].endNodeId.c_str() + " is invalid NURBS-curve";
          return false;
          break;          
        }
      }
      else
      {
        ROS_WARN("Edge: %s not found startNodeId: %s or endNodeId: %s", msg.edges[i].edgeId.c_str(), 
        msg.edges[i].startNodeId.c_str(), msg.edges[i].endNodeId.c_str());
        status = 1;
        message = "Edge: " + msg.edges[i].edgeId + " not found startNodeId: " + msg.edges[i].startNodeId.c_str() +                   
          " or endNodeId: " + msg.edges[i].endNodeId.c_str();
        return false;
        break;        
      }
      ROS_INFO("Finish to compute at Edge: %s", msg.edges[i].edgeId.c_str());
    }
    status = 0;
    message = "Success to make plan: StartNode: " + msg.edges[0].startNodeId + ", EndNode: " + msg.edges[msg.edges.size()-1].endNodeId;
    return true;
  }

  bool CustomPlanner::makePlanForRetry(std::vector<geometry_msgs::PoseStamped>& current_plan, 
    int indexOfPoseA, geometry_msgs::PoseStamped& pose_B, 
    geometry_msgs::PoseStamped& pose_C, std::vector<geometry_msgs::PoseStamped>& result_plan)
  {    
    bool result = false;
    vector<geometry_msgs::PoseStamped> PlanRetry_1;
    vector<geometry_msgs::PoseStamped> PlanRetry_2;

    if(current_plan.empty()||current_plan.size()<2)
    {
      ROS_WARN("current_plan is empty");
      return false;
    }

    geometry_msgs::PoseStamped pose_A;
    pose_A = current_plan[indexOfPoseA];

    // Tính ra PlanRetry_1 điểm retry tại Pose_A
 
    PlanRetry_1.assign(current_plan.begin()+indexOfPoseA, current_plan.end());

    if(!PlanRetry_1.empty()){
      std::reverse(PlanRetry_1.begin(), PlanRetry_1.end());
    }

    // Tính ra PlanRetry_2 với biên dạng cung tròn đi qua pose_A và pose_B, có tâm tại pose_C

    double xAB = pose_B.pose.position.x - pose_A.pose.position.x;
    double yAB = pose_B.pose.position.y - pose_A.pose.position.y;
    double d_AB = sqrt(xAB*xAB + yAB*yAB);
    if(d_AB<=0.1)
    {
      ROS_WARN("Curve AB is too short, cannot compute plan");
      return false;
    }
    
    // nếu hướng của vector AB và hướng của pose_B tạo với nhau một góc ~0 độ hoặc ~180 độ -> cung tròn AB sẽ gần như là một đọan thẳng
    if((computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
        pose_B.pose, pose_A.pose) >= 3.13 && 
        computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
        pose_B.pose, pose_A.pose) <= M_PI) ||
       (computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
        pose_B.pose, pose_A.pose) <= 0.1745 && 
        computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
        pose_B.pose, pose_A.pose) >= 0))
    {
      vector<geometry_msgs::PoseStamped> planSegment_AB;
      planSegment_AB = divideSegment(pose_A, pose_B, 0.05);
      PlanRetry_2.assign(planSegment_AB.begin(), planSegment_AB.end());
    }
    else
    {
      double xCA = pose_A.pose.position.x - pose_C.pose.position.x;
      double yCA = pose_A.pose.position.y - pose_C.pose.position.y;
      double xCB = pose_B.pose.position.x - pose_C.pose.position.x;
      double yCB = pose_B.pose.position.y - pose_C.pose.position.y;
      double rCA = sqrt(xCA*xCA + yCA*yCA);
      double rCB = sqrt(xCB*xCB + yCB*yCB);
      if(fabs(rCA-rCB)>0.008)
      {
        ROS_WARN("pose_C is not Center of Curve AB");
        return false;
      }            

      double cos_ACB = (xCA*xCB + yCA*yCB)/(rCA*rCB);
      if(cos_ACB>1) cos_ACB = 1;
      else if(cos_ACB<(-1)) cos_ACB = -1;
      double angleACB = acos(cos_ACB);
      double angle_interval = 0.005;
      // tính góc của vector CA:
      double angleCA = atan2(yCA, xCA);

      // check thử xem chiều góc quét từ A -> B thì angleCA + delta_angle hay angleCA - delta_angle
      bool is_increase_angle = false;
      double check_angle = angleCA + 50*angle_interval*angleACB;
      double xA1 = pose_C.pose.position.x + rCA*cos(check_angle);
      double yA1 = pose_C.pose.position.y + rCA*sin(check_angle);
      double xCA1 = xA1 - pose_C.pose.position.x;
      double yCA1 = yA1 - pose_C.pose.position.y;
      double cos_A1CB = (xCA1*xCB + yCA1*yCB)/(rCA*rCB);
      if(cos_A1CB>1) cos_A1CB = 1;
      else if(cos_A1CB<(-1)) cos_A1CB = -1;
      double angleA1CB = acos(cos_A1CB);
      if(angleA1CB>angleACB)
      {
        is_increase_angle = false;
      }
      else if(angleA1CB<angleACB)
      {
        is_increase_angle = true;
      }
      else
      {
        ROS_WARN("Curve AB is too short, cannot compute plan");
        return false;
      }
      if(is_increase_angle)
      {
        for(double i = 0; i<=1; i+= angle_interval)
        {
          double angle_tmp = angleCA + angleACB*i;
          double xP = pose_C.pose.position.x + rCA*cos(angle_tmp);
          double yP = pose_C.pose.position.y + rCA*sin(angle_tmp);
          geometry_msgs::PoseStamped p;
          p.pose.position.x = xP;
          p.pose.position.y = yP;
          p.pose.position.z = 0;
          PlanRetry_2.push_back(p);
        }
      }
      else
      {
        for(double i = 0; i<=1; i+= angle_interval)
        {
          double angle_tmp = angleCA - angleACB*i;
          double xP = pose_C.pose.position.x + rCA*cos(angle_tmp);
          double yP = pose_C.pose.position.y + rCA*sin(angle_tmp);
          geometry_msgs::PoseStamped p;
          p.pose.position.x = xP;
          p.pose.position.y = yP;
          p.pose.position.z = 0;
          PlanRetry_2.push_back(p);
        }
      }
      if(!PlanRetry_2.empty()&&PlanRetry_2.size()>2)
      {
        for(int i = 0 ; i < PlanRetry_2.size(); i++)
        {
          ROS_INFO("Pose %d in PlanRetry : %f, %f", i, PlanRetry_2[i].pose.position.x, PlanRetry_2[i].pose.position.y);
        }
        if(computeDeltaAngleStartOfPlan(getYaw(pose_A.pose.orientation.x, pose_A.pose.orientation.y, pose_A.pose.orientation.z, pose_A.pose.orientation.w),
          PlanRetry_2.front().pose, PlanRetry_2[1].pose) <= 0.872664626 &&  
          computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
          PlanRetry_2.back().pose, PlanRetry_2[PlanRetry_2.size() - 2].pose) <= 0.872664626
          ) // <= 50 degree
        {
          for(int i = 0; i<((int)PlanRetry_2.size()-1); i++)
          {
              double theta = calculateAngle(PlanRetry_2[i].pose.position.x, PlanRetry_2[i].pose.position.y, 
                                            PlanRetry_2[i+1].pose.position.x, PlanRetry_2[i+1].pose.position.y);
              PlanRetry_2[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
          }                        
          PlanRetry_2.back().pose.orientation = pose_B.pose.orientation;
        }
        else if(computeDeltaAngleStartOfPlan(getYaw(pose_A.pose.orientation.x, pose_A.pose.orientation.y, pose_A.pose.orientation.z, pose_A.pose.orientation.w),
          PlanRetry_2.front().pose, PlanRetry_2[1].pose) >= 2.2689280276 &&  
          computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
          PlanRetry_2.back().pose, PlanRetry_2[PlanRetry_2.size() - 2].pose) >= 2.2689280276) // >= 130 degree
        {
          for(int i = (int)PlanRetry_2.size() -1; i>0; i--)
          {
              double theta = calculateAngle(PlanRetry_2[i].pose.position.x, PlanRetry_2[i].pose.position.y, 
                                            PlanRetry_2[i-1].pose.position.x, PlanRetry_2[i-1].pose.position.y);
              PlanRetry_2[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
          }
          PlanRetry_2.front().pose.orientation = PlanRetry_2[1].pose.orientation;
        }
        else
        {
          ROS_WARN("Pose_A yaw or Pose_B yaw is invalid value");
          return false;
        }
      }
      else
      {
        ROS_WARN("Curve AB is too short, cannot compute plan");
        return false;
      }
    }

    ros::Time plan_time = ros::Time::now();
    if(!PlanRetry_1.empty()&&!PlanRetry_2.empty())
    {
      for(int i = 0; i < (int)PlanRetry_1.size(); i++)
      {
        PlanRetry_1[i].header.stamp = plan_time;
        result_plan.push_back(PlanRetry_1[i]);
      }
      for(int i = 0; i < (int)PlanRetry_2.size(); i++)
      {
        PlanRetry_2[i].header.stamp = plan_time;
        PlanRetry_2[i].header.frame_id = PlanRetry_1.front().header.frame_id;
        result_plan.push_back(PlanRetry_2[i]);
      }
      result = true;
    }
    return result;
  }

  bool CustomPlanner::findCenterOfCurve(geometry_msgs::PoseStamped& pose_A, geometry_msgs::PoseStamped& pose_B, geometry_msgs::PoseStamped& pose_C)
  {
    // nếu hướng của vector AB và hướng của pose_B tạo với nhau một góc ~0 độ hoặc ~180 độ -> điểm C sẽ gần xấp xỉ với trung điểm của đoạn thẳng AB.
    if((computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
        pose_B.pose, pose_A.pose) >= 3.13 && 
        computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
        pose_B.pose, pose_A.pose) <= M_PI) ||
       (computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
        pose_B.pose, pose_A.pose) <= 0.1745 && 
        computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
        pose_B.pose, pose_A.pose) >= 0))
    {
      pose_C.pose.position.x = (pose_A.pose.position.x + pose_B.pose.position.x)/2;
      pose_C.pose.position.y = (pose_A.pose.position.y + pose_B.pose.position.y)/2;
    }
    else
    {
      double x_R = pose_A.pose.position.x;
      double y_R = pose_A.pose.position.y;
      double x_G = pose_B.pose.position.x;
      double y_G = pose_B.pose.position.y;
      double phi_vG = getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w);
      double x_H = (x_R+x_G)/2;
      double y_H = (y_R+y_G)/2;
      double m_vG = tan(phi_vG);
      double m_G_n_vG = -1/m_vG;
      double b_G_n_vG = y_G-m_G_n_vG*x_G;
      double m_RG =(y_G-y_R)/(x_G-x_R);
      double b_RG = y_R-m_RG*x_R;
      double m_H_n_RG = -1/m_RG;
      double b_H_n_RG = y_H-m_H_n_RG*x_H;
      pose_C.pose.position.x = (b_H_n_RG-b_G_n_vG)/(m_G_n_vG-m_H_n_RG);
      pose_C.pose.position.y = (b_H_n_RG*m_G_n_vG-b_G_n_vG*m_H_n_RG)/(m_G_n_vG-m_H_n_RG);
    }
    return true;
  }

  bool CustomPlanner::findCenterOfCurve(Pose& pose_A, Pose& pose_B, Pose& pose_C)
  {
    // nếu hướng của vector AB và hướng của pose_B tạo với nhau một góc ~0 độ hoặc ~180 độ -> điểm C sẽ gần xấp xỉ với trung điểm của đoạn thẳng AB.
    if((computeDeltaAngleEndNode(pose_B.getYaw(),
        pose_B, pose_A) >= 3.13 && 
        computeDeltaAngleEndNode(pose_B.getYaw(),
        pose_B, pose_A) <= M_PI) ||
       (computeDeltaAngleEndNode(pose_B.getYaw(),
        pose_B, pose_A) <= 0.1745 && 
        computeDeltaAngleEndNode(pose_B.getYaw(),
        pose_B, pose_A) >= 0))
    {
      double xC = (pose_A.getX() + pose_B.getX())/2;
      double yC = (pose_A.getY() + pose_B.getY())/2;
      pose_C.setX(xC);
      pose_C.setY(yC);
    }
    else
    {
      double x_R = pose_A.getX();
      double y_R = pose_A.getY();
      double x_G = pose_B.getX();
      double y_G = pose_B.getY();
      double phi_vG = pose_B.getYaw();
      double x_H = (x_R+x_G)/2;
      double y_H = (y_R+y_G)/2;
      double m_vG = tan(phi_vG);
      double m_G_n_vG = -1/m_vG;
      double b_G_n_vG = y_G-m_G_n_vG*x_G;
      double m_RG =(y_G-y_R)/(x_G-x_R);
      double b_RG = y_R-m_RG*x_R;
      double m_H_n_RG = -1/m_RG;
      double b_H_n_RG = y_H-m_H_n_RG*x_H;
      double xC = (b_H_n_RG-b_G_n_vG)/(m_G_n_vG-m_H_n_RG);
      double yC = (b_H_n_RG*m_G_n_vG-b_G_n_vG*m_H_n_RG)/(m_G_n_vG-m_H_n_RG);
      pose_C.setX(xC);
      pose_C.setY(yC);
    }
    return true;
  }

  bool CustomPlanner::isThetaValid(double theta)
  {
    bool result = false;
    if(theta < -M_PI || theta > M_PI) result = false;
    else result = true;
    return result;
  }

  bool CustomPlanner::curveIsValid(int degree, const std::vector<double> &knot_vector,
                  vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& control_points)
  {
    if(degree < 1 || degree > 9)
    {
      ROS_WARN("degree is invalid value");
      return false;
    }
    if(!((knot_vector.size() - degree - 1) == control_points.size()))
    {
      ROS_WARN("relation between degree, number of knots, and number of control points is invalid");
      return false;
    }
    // if(std::is_sorted(knot_vector.begin(), knot_vector.end()))
    // {
    //   ROS_WARN("knot vector is not monotonic");
    //   return false;
    // }
    return true;
  }

  double CustomPlanner::computeDeltaAngleStartNode(double theta, Pose& startPose, Pose& next_Pose)
  {
    double delta_angle = 0;
    if(isThetaValid(theta))
    {
      double xAB = next_Pose.getX() - startPose.getX();
      double yAB = next_Pose.getY() - startPose.getY();
      double d = sqrt(xAB*xAB + yAB*yAB);
      double xC = startPose.getX() + d*cos(theta);
      double yC = startPose.getY() + d*sin(theta);
      double xAC = xC-startPose.getX();
      double yAC = yC-startPose.getY();
      double dAB = sqrt(xAB*xAB + yAB*yAB);
      double cos_a = (xAB*xAC + yAB*yAC)/(dAB*d);
      if(cos_a>1) cos_a = 1;
      else if(cos_a<(-1)) cos_a = -1;
      delta_angle = acos(cos_a);
      // delta_angle = delta_angle*180/M_PI;
      // ROS_WARN("xC: %f, yC: %f", xC, yC);
      // ROS_WARN("dAB: %f", dAB);
      // ROS_WARN("delta_angle: %f", delta_angle);
    }   
    return delta_angle;    
  }

  double CustomPlanner::computeDeltaAngleEndNode(double theta, Pose& endPose, Pose& prev_Pose)
  {
    double delta_angle = 0;
    if(isThetaValid(theta))
    {
      double xAB = endPose.getX()-prev_Pose.getX();
      double yAB = endPose.getY()-prev_Pose.getY();
      double d = sqrt(xAB*xAB + yAB*yAB);
      double xC = endPose.getX() + d*cos(theta);
      double yC = endPose.getY() + d*sin(theta);
      double xBC = xC-endPose.getX();
      double yBC = yC-endPose.getY();
      double dAB = sqrt(xAB*xAB + yAB*yAB);
      double cos_a = (xAB*xBC + yAB*yBC)/(dAB*d);
      if(cos_a>1) cos_a = 1;
      else if(cos_a<(-1)) cos_a = -1;
      delta_angle = acos(cos_a);
      // delta_angle = delta_angle*180/M_PI;
      // ROS_WARN("xC: %f, yC: %f", xC, yC);
      // ROS_WARN("dAB: %f", dAB);
      // ROS_WARN("delta_angle: %f", delta_angle);
    }  
    return delta_angle;
  }

  double CustomPlanner::computeDeltaAngleEndOfPlan(double theta, geometry_msgs::Pose& endPose, geometry_msgs::Pose& prev_Pose)
  {
    double delta_angle = 0;
    if(isThetaValid(theta))
    {
      double xAB =endPose.position.x-prev_Pose.position.x;
      double yAB = endPose.position.y-prev_Pose.position.y;
      double d = sqrt(xAB*xAB + yAB*yAB);
      double xC =endPose.position.x + d*cos(theta);
      double yC = endPose.position.y + d*sin(theta);
      double xBC = xC-endPose.position.x;
      double yBC = yC-endPose.position.y;
      double dAB = sqrt(xAB*xAB + yAB*yAB);
      double cos_a = (xAB*xBC + yAB*yBC)/(dAB*d);
      if(cos_a>1) cos_a = 1;
      else if(cos_a<(-1)) cos_a = -1;
      delta_angle = acos(cos_a);
      // delta_angle = delta_angle*180/M_PI;
      // ROS_WARN("xC: %f, yC: %f", xC, yC);
      // ROS_WARN("dAB: %f", dAB);
      // ROS_WARN("delta_angle: %f", delta_angle);
    }  
    return delta_angle;
  }

  void CustomPlanner::setYawAllPosesOnEdge(vector<Pose>& posesOnEdge, bool reverse)
  {
    if(!reverse)
    {
      if(!posesOnEdge.empty()){
        if(posesOnEdge.size()>2){
          for(int i = 0; i<((int)posesOnEdge.size()-1); i++)
          {
            double theta = calculateAngle(posesOnEdge[i].getX(), posesOnEdge[i].getY(), 
                                          posesOnEdge[i+1].getX(), posesOnEdge[i+1].getY());
            posesOnEdge[i].setYaw(theta);
          }
          posesOnEdge.back().setYaw(posesOnEdge[posesOnEdge.size()-2].getYaw());          
        }
        else if(posesOnEdge.size()==2)
        {
          if(posesOnEdge[0].getX()!=posesOnEdge[1].getX())
          {
            double theta = calculateAngle(posesOnEdge[0].getX(), posesOnEdge[0].getY(), 
                                              posesOnEdge[1].getX(), posesOnEdge[1].getY());            
            posesOnEdge[0].setYaw(theta);
            posesOnEdge[1].setYaw(theta);   
          }                                      
        }
      }
    }
    else
    {
      if(!posesOnEdge.empty()){
        if(posesOnEdge.size()>2){    
          for(int i = (int)posesOnEdge.size() -1; i>0; i--)
          {
            double theta = calculateAngle(posesOnEdge[i].getX(), posesOnEdge[i].getY(), 
                                          posesOnEdge[i-1].getX(), posesOnEdge[i-1].getY());
            posesOnEdge[i].setYaw(theta);
          }
          posesOnEdge.front().setYaw(posesOnEdge[1].getYaw());
        }
        else if(posesOnEdge.size()==2)
        {
          if(posesOnEdge[1].getX()!=posesOnEdge[0].getX())
          {
            double theta = calculateAngle(posesOnEdge[1].getX(), posesOnEdge[1].getY(), 
                                              posesOnEdge[0].getX(), posesOnEdge[0].getY());
            posesOnEdge[1].setYaw(theta);    
            posesOnEdge[0].setYaw(theta);
          }                                        
        }
      }
    }
  }

  double CustomPlanner::computeDeltaAngleStartNode(double thetaEnd, double thetaStart, Pose& Pose)
  {
    double delta_angle = 0;
    if(isThetaValid(thetaEnd)&&isThetaValid(thetaStart))
    {
      double d = 1;
      double xA = Pose.getX();
      double yA = Pose.getY();
      double xB = xA + d*cos(thetaEnd);
      double yB = yA + d*sin(thetaEnd);
      double xAB = xB - xA;
      double yAB = yB - yA;
      double xC = xA + d*cos(thetaStart);
      double yC = yA + d*sin(thetaStart);
      double xAC = xC - xA;
      double yAC = yC - yA;
      double cos_a = (xAB*xAC + yAB*yAC)/(d*d);
      if(cos_a>1) cos_a = 1;
      else if(cos_a<(-1)) cos_a = -1;
      delta_angle = acos(cos_a);
      // delta_angle = delta_angle*180/M_PI;
      // ROS_WARN("delta_angle: %f", delta_angle);
    }   
    return delta_angle;
  }

  double CustomPlanner::computeDeltaAngleStartOfPlan(double theta, geometry_msgs::Pose& startPose, geometry_msgs::Pose& next_Pose)
  {
    double delta_angle = 0;
    if(isThetaValid(theta))
    {
      double xAB = next_Pose.position.x - startPose.position.x;
      double yAB = next_Pose.position.y - startPose.position.y;
      double d = sqrt(xAB*xAB + yAB*yAB);
      double xC = startPose.position.x + d*cos(theta);
      double yC = startPose.position.y + d*sin(theta);
      double xAC = xC-startPose.position.x;
      double yAC = yC-startPose.position.y;
      double dAB = sqrt(xAB*xAB + yAB*yAB);
      double cos_a = (xAB*xAC + yAB*yAC)/(dAB*d);
      if(cos_a>1) cos_a = 1;
      else if(cos_a<(-1)) cos_a = -1;
      delta_angle = acos(cos_a);
      // delta_angle = delta_angle*180/M_PI;
      // ROS_WARN("xC: %f, yC: %f", xC, yC);
      // ROS_WARN("dAB: %f", dAB);
      // ROS_WARN("delta_angle: %f", delta_angle);
    }   
    return delta_angle;    
  }

  double CustomPlanner::computeDeltaAngleStartOfPlan(double thetaEnd, double thetaStart, geometry_msgs::Pose& Pose)
  {
    double delta_angle = 0;
    if(isThetaValid(thetaEnd)&&isThetaValid(thetaStart))
    {
      double d = 1;
      double xA = Pose.position.x;
      double yA = Pose.position.y;
      double xB = xA + d*cos(thetaEnd);
      double yB = yA + d*sin(thetaEnd);
      double xAB = xB - xA;
      double yAB = yB - yA;
      double xC = xA + d*cos(thetaStart);
      double yC = yA + d*sin(thetaStart);
      double xAC = xC - xA;
      double yAC = yC - yA;
      double cos_a = (xAB*xAC + yAB*yAC)/(d*d);
      if(cos_a>1) cos_a = 1;
      else if(cos_a<(-1)) cos_a = -1;
      delta_angle = acos(cos_a);
      // delta_angle = delta_angle*180/M_PI;
      // ROS_WARN("delta_angle: %f", delta_angle);
    }   
    return delta_angle;  
  }

  double CustomPlanner::computeDeltaAngle(Pose& Pose1, Pose& Pose2)
  {
    double delta_angle = 0;
    if(isThetaValid(Pose1.getYaw())&&isThetaValid(Pose2.getYaw()))
    {
      double xA = Pose1.getX();
      double yA = Pose1.getY();
      double xB = Pose2.getX();
      double yB = Pose2.getY();
      double xAB = xB - xA;
      double yAB = yB - yA;
      double d = sqrt(xAB*xAB + yAB*yAB);
      double xC = xB + d*cos(Pose2.getYaw());
      double yC = yB + d*sin(Pose2.getYaw());
      double xBC = xC - xB;
      double yBC = yC - yB;
      double cos_a = (xAB*xBC + yAB*yBC)/(d*d);
      if(cos_a>1) cos_a = 1;
      else if(cos_a<(-1)) cos_a = -1;
      double delta_angle_tmp = acos(cos_a);
      if(delta_angle_tmp>=1.5707963268)
      {
        delta_angle = M_PI - delta_angle_tmp;
      }
      else
      {
        delta_angle = delta_angle_tmp;
      }
      // delta_angle = delta_angle*180/M_PI;
      // ROS_WARN("delta_angle: %f", delta_angle);
    }   
    return delta_angle;
  }

  // Hàm chia đoạn thẳng AB thành các đoạn có độ dài d
  vector<Pose> CustomPlanner::divideSegment(Pose& A, Pose& B, double d) {
      vector<Pose> Poses;
      double xAB = B.getX() - A.getX();
      double yAB = B.getY() - A.getY();
      double length = sqrt(xAB*xAB + yAB*yAB);
      if(length > d)
      {
        Poses.push_back(A); // Thêm điểm A vào vector trước khi chia
        
        int segments = length / d;

        // Tính toán tọa độ của các điểm trên đoạn AB
        double ratio = d / length;
        for (int i = 1; i <= segments; ++i) {
            Pose p;
            double p_x = A.getX() + (B.getX() - A.getX()) * ratio * i;
            double p_y = A.getY() + (B.getY() - A.getY()) * ratio * i;
            p.setX(p_x);
            p.setY(p_y);
            Poses.push_back(p);
        }
        
        if(!Poses.empty()&&(Poses.back().getX()!=B.getX() || Poses.back().getY()!=B.getY()))
        {
            Poses.push_back(B); // Thêm điểm B vào vector sau khi chia
        }    

        // Tính góc cho từng pose trên đoạn AB
        if(//computeDeltaAngleStartNode(A.getYaw(), Poses.front(), Poses[1]) <= 0.872664626 &&
                computeDeltaAngleEndNode(B.getYaw(), Poses.back(), Poses[Poses.size()-2]) <= 1.3962634016) // <= 80 degree
        {
          setYawAllPosesOnEdge(Poses, false);
          Poses.front().setYaw(A.getYaw());
          Poses.back().setYaw(B.getYaw());
        }
        else if(//computeDeltaAngleStartNode(A.getYaw(), Poses.front(), Poses[1]) >= 2.2689280276 &&
                computeDeltaAngleEndNode(B.getYaw(), Poses.back(), Poses[Poses.size()-2]) >= 1.745329252) // >= 100 degree
        {
          setYawAllPosesOnEdge(Poses, true);
          Poses.front().setYaw(A.getYaw());
          Poses.back().setYaw(B.getYaw());
        }
      }
      else
      {
        Poses.push_back(A);
        Poses.push_back(B);
      }
      return Poses;
  }

  // Hàm chia đoạn thẳng AB thành các đoạn có độ dài d
  vector<geometry_msgs::PoseStamped> CustomPlanner::divideSegment(geometry_msgs::PoseStamped& A, geometry_msgs::PoseStamped& B, double d) {
      vector<geometry_msgs::PoseStamped> Poses;
      double xAB = B.pose.position.x - A.pose.position.x;
      double yAB = B.pose.position.y - A.pose.position.y;
      double length = sqrt(xAB*xAB + yAB*yAB);
      if(length > d)
      {
        Poses.push_back(A); // Thêm điểm A vào vector trước khi chia
        
        int segments = length / d;

        // Tính toán tọa độ của các điểm trên đoạn AB
        double ratio = d / length;
        for (int i = 1; i <= segments; ++i) {
            geometry_msgs::PoseStamped p;
            double p_x = A.pose.position.x + (B.pose.position.x - A.pose.position.x) * ratio * i;
            double p_y = A.pose.position.y + (B.pose.position.y - A.pose.position.y) * ratio * i;
            p.pose.position.x = p_x;
            p.pose.position.y = p_y;
            Poses.push_back(p);
        }
        
        if(!Poses.empty()&&(Poses.back().pose.position.x!=B.pose.position.x || Poses.back().pose.position.y!=B.pose.position.y))
        {
            Poses.push_back(B); // Thêm điểm B vào vector sau khi chia
        }    

        // Tính góc cho từng pose trên đoạn AB
        if(//computeDeltaAngleStartOfPlan(getYaw(A.pose.orientation.x, A.pose.orientation.y, A.pose.orientation.z, A.pose.orientation.w),
          //Poses.front().pose, Poses[1].pose) <= 0.872664626 &&  
          computeDeltaAngleEndOfPlan(getYaw(B.pose.orientation.x, B.pose.orientation.y, B.pose.orientation.z, B.pose.orientation.w),
          Poses.back().pose, Poses[Poses.size() - 2].pose) <= 1.3962634016) // <= 80 degree
        {
          for(int i = 0; i<((int)Poses.size()-1); i++)
          {
              double theta = calculateAngle(Poses[i].pose.position.x, Poses[i].pose.position.y, 
                                            Poses[i+1].pose.position.x, Poses[i+1].pose.position.y);
              Poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
          }
          Poses.back().pose.orientation = B.pose.orientation;
        }
        else if(//computeDeltaAngleStartOfPlan(getYaw(A.pose.orientation.x, A.pose.orientation.y, A.pose.orientation.z, A.pose.orientation.w),
                //Poses.front().pose, Poses[1].pose) >= 2.2689280276 &&
                computeDeltaAngleEndOfPlan(getYaw(B.pose.orientation.x, B.pose.orientation.y, B.pose.orientation.z, B.pose.orientation.w),
                Poses.back().pose, Poses[Poses.size() - 2].pose) >= 1.745329252) // >= 100 degree
        {
          for(int i = (int)Poses.size() -1; i>0; i--)
          {
                double theta = calculateAngle(Poses[i].pose.position.x, Poses[i].pose.position.y, 
                                              Poses[i-1].pose.position.x, Poses[i-1].pose.position.y);
                Poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);  
          }
          Poses.front().pose.orientation = A.pose.orientation;
        }
      }
      else
      {
        Poses.push_back(A);
        Poses.push_back(B);
      }
      return Poses;
  }

  bool CustomPlanner::makeCurvePlan(Pose& pose_A, Pose& pose_B, std::vector<Pose>& result_plan)
  {
    bool result = false;
    double xAB = pose_B.getX() - pose_A.getX();
    double yAB = pose_B.getY() - pose_A.getY();
    double d_AB = sqrt(xAB*xAB + yAB*yAB);
    if(d_AB<=0.1)
    {
      ROS_WARN("[custom_planner][makeCurvePlan] Curve AB is too short, it is almost a straight line");
      return false;
    }

    // nếu hướng của vector AB và hướng của pose_B tạo với nhau một góc ~0 độ hoặc ~180 độ -> cung tròn AB sẽ gần như là một đọan thẳng
    if((computeDeltaAngleEndNode(pose_B.getYaw(),
        pose_B, pose_A) >= 3.13 && 
        computeDeltaAngleEndNode(pose_B.getYaw(),
        pose_B, pose_A) <= M_PI) ||
       (computeDeltaAngleEndNode(pose_B.getYaw(),
        pose_B, pose_A) <= 0.1745 && 
        computeDeltaAngleEndNode(pose_B.getYaw(),
        pose_B, pose_A) >= 0))
    {
      vector<Pose> planSegment_AB;
      planSegment_AB = divideSegment(pose_A, pose_B, 0.05);
      result_plan.assign(planSegment_AB.begin(), planSegment_AB.end());
      ROS_WARN("[custom_planner][makeCurvePlan] Curve AB is almost a straight line 1");
    }
    // Tính quỹ đạo từ A->B có dạng một cung tròn
    else
    {
      Pose pose_C;
      if(findCenterOfCurve(pose_A, pose_B, pose_C))
      {
        double xCA = pose_A.getX() - pose_C.getX();
        double yCA = pose_A.getY() - pose_C.getY();
        double xCB = pose_B.getX() - pose_C.getX();
        double yCB = pose_B.getY() - pose_C.getY();
        double rCA = sqrt(xCA*xCA + yCA*yCA);
        double rCB = sqrt(xCB*xCB + yCB*yCB);
        if(fabs(rCA-rCB)>0.008)
        {
          ROS_WARN("[custom_planner][makeCurvePlan] pose_C is not Center of Curve AB");
          return false;
        }            

        double cos_ACB = (xCA*xCB + yCA*yCB)/(rCA*rCB);
        if(cos_ACB>1) cos_ACB = 1;
        else if(cos_ACB<(-1)) cos_ACB = -1;
        double angleACB = acos(cos_ACB);
        double angle_interval = 0.01;
        // tính góc của vector CA:
        double angleCA = atan2(yCA, xCA);

        // check thử xem chiều góc quét từ A -> B thì angleCA + delta_angle hay angleCA - delta_angle
        bool is_increase_angle = false;
        double check_angle = angleCA + 50*angle_interval*angleACB;
        double xA1 = pose_C.getX() + rCA*cos(check_angle);
        double yA1 = pose_C.getY() + rCA*sin(check_angle);
        double xCA1 = xA1 - pose_C.getX();
        double yCA1 = yA1 - pose_C.getY();
        double cos_A1CB = (xCA1*xCB + yCA1*yCB)/(rCA*rCB);
        if(cos_A1CB>1) cos_A1CB = 1;
        else if(cos_A1CB<(-1)) cos_A1CB = -1;
        double angleA1CB = acos(cos_A1CB);
        if(angleA1CB>angleACB)
        {
          is_increase_angle = false;
        }
        else if(angleA1CB<angleACB)
        {
          is_increase_angle = true;
        }
        else
        {
          ROS_WARN("[custom_planner][makeCurvePlan] Curve AB is almost a straight line 2");
          return false;
        }
        if(is_increase_angle)
        {
          for(double i = 0; i<=1; i+= angle_interval)
          {
            double angle_tmp = angleCA + angleACB*i;
            double xP = pose_C.getX() + rCA*cos(angle_tmp);
            double yP = pose_C.getY() + rCA*sin(angle_tmp);
            Pose p;
            p.setX(xP);
            p.setY(yP);
            result_plan.push_back(p);
          }
        }
        else
        {
          for(double i = 0; i<=1; i+= angle_interval)
          {
            double angle_tmp = angleCA - angleACB*i;
            double xP = pose_C.getX() + rCA*cos(angle_tmp);
            double yP = pose_C.getY() + rCA*sin(angle_tmp);
            Pose p;
            p.setX(xP);
            p.setY(yP);
            result_plan.push_back(p);
          }
        }
        if(!result_plan.empty()&&result_plan.size()>2)
        {
          for(int i = 0 ; i < result_plan.size(); i++)
          {
            // ROS_INFO("Pose %d in PlanRetry : %f, %f", i, result_plan[i].getX(), result_plan[i].getY());
          }
          if(//computeDeltaAngleStartNode(pose_A.getYaw(), result_plan.front(), result_plan[1]) < 1.5707963268 &&  
            computeDeltaAngleEndNode(pose_B.getYaw(), result_plan.back(), result_plan[result_plan.size() - 2]) <= 1.3962634016) // <= 80 degree
          {
            for(int i = 0; i<((int)result_plan.size()-1); i++)
            {
                  double theta = calculateAngle(result_plan[i].getX(), result_plan[i].getY(), 
                                                result_plan[i+1].getX(), result_plan[i+1].getY());                  
                  result_plan[i].setYaw(theta);
            }
            result_plan.back().setYaw(pose_B.getYaw());
          }
          else if(//computeDeltaAngleStartNode(pose_A.getYaw(), result_plan.front(), result_plan[1]) >= 1.5707963268 &&  
            computeDeltaAngleEndNode(pose_B.getYaw(), result_plan.back(), result_plan[result_plan.size() - 2]) >= 1.745329252) // >= 100 degree
          {
            for(int i = (int)result_plan.size() -1; i>0; i--)
            {              
                  double theta = calculateAngle(result_plan[i].getX(), result_plan[i].getY(), 
                                                result_plan[i-1].getX(), result_plan[i-1].getY());
                  result_plan[i].setYaw(theta); 
            }
            result_plan.front().setYaw(result_plan[1].getYaw());
          }
          else
          {
            ROS_WARN("[custom_planner][makeCurvePlan] Pose_A yaw or Pose_B yaw is invalid value");
            return false;
          }
        }
        else
        {
          ROS_WARN("[custom_planner][makeCurvePlan] Curve AB is almost a straight line 3");
          return false;
        }
      }
    }
    if(!result_plan.empty())
    {
      if(result_plan.front().getX()!=pose_A.getX() || result_plan.front().getY()!=pose_A.getY())
      {
        result_plan.insert(result_plan.begin(), pose_A);
      }
      if(result_plan.back().getX()!=pose_B.getX() || result_plan.back().getY()!=pose_B.getY())
      {
        result_plan.push_back(pose_B);
      }
      result = true;
    }
    return result;
  }

  void CustomPlanner::test_print_plan_result()
  {
    for (unsigned int i = 0; i < posesOnPathWay.size(); i++)
    {
      if(i==0) std::cerr<<"{ ";
      else if(i>0&&i<(posesOnPathWay.size()-1))
      std::cerr<<"{"<<posesOnPathWay[i].getX()<<", "<<posesOnPathWay[i].getY()<<", "<<posesOnPathWay[i].getYaw()<<"}"<<","<<std::endl;
      else if(i==(posesOnPathWay.size()-1))
      {
        std::cerr<<"{"<<posesOnPathWay[i].getX()<<", "<<posesOnPathWay[i].getY()<<", "<<posesOnPathWay[i].getYaw()<<"} "<<"}"<<std::endl;
      }
    }
    ROS_WARN("done print");
  }

};
