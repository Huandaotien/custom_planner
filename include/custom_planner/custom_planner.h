#ifndef CUSTOM_PLANNER_H
#define CUSTOM_PLANNER_H

#include <iostream>
#include <vector>
#include <cmath>
#include <map>

using namespace std;

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

// sbpl headers
#include <sbpl/headers.h>

// global representation
#include <nav_core/base_global_planner.h>

// tf
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>

// internal lib
#include <custom_planner/pose.h>
#include <custom_planner/pathway.h>
#include "custom_planner/Curve_common.h"
#include "custom_planner/conversion.h"
#include "custom_planner/PlanWithOrder.h"

#include <geometry_msgs/PoseArray.h>

#include <thread>
#include <boost/thread.hpp>

#include "vda5050_msgs/Order.h"
#include "vda5050_msgs/Trajectory.h"
#include "vda5050_msgs/Edge.h"
#include "vda5050_msgs/Node.h"
#include "vda5050_msgs/ControlPoint.h"
#include "vda5050_msgs/NodePosition.h"



using namespace std;

namespace custom_planner{

struct userParams{
    userParams(){
        directory_to_save_paths = "/init/paths";             
        pathway_filename = "pathway.txt";
        current_pose_topic_name = "/amcl_pose";
        map_frame_id = "map";
        base_frame_id = "base_link";
    }
    string directory_to_save_paths;
    string pathway_filename;
    string current_pose_topic_name;
    string map_frame_id;
    string base_frame_id;
};

struct OrderNode{
  string nodeId;
  uint32_t sequenceId;
  double position_x;
  double position_y;
  double theta;
};

class CustomPlanner : public nav_core::BaseGlobalPlanner{
public:
  
  /**
   * @brief  Default constructor for the NavFnROS object
   */
  CustomPlanner();

  
  /**
   * @brief  Constructor for the CustomPlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  CustomPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


  /**
   * @brief  Initialization function for the CustomPlanner object
   * @param  name The name of this planner
   * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use
   */
  virtual void initialize(std::string name, 
                          costmap_2d::Costmap2DROS* costmap_ros);
  
  /**
   * @brief Given a goal pose in the world, compute a plan
   * @param start The start pose 
   * @param goal The goal pose 
   * @param plan The plan... filled by the planner
   * @return True if a valid plan was found, false otherwise
   */
  virtual bool makePlan(const geometry_msgs::PoseStamped& start, 
                        const geometry_msgs::PoseStamped& goal, 
                        std::vector<geometry_msgs::PoseStamped>& plan);

  virtual ~CustomPlanner(){};

private:
  unsigned char costMapCostToSBPLCost(unsigned char newcost);
  void publishStats(int solution_cost, int solution_size, 
                    const geometry_msgs::PoseStamped& start, 
                    const geometry_msgs::PoseStamped& goal);

  unsigned char computeCircumscribedCost();

  static void transformFootprintToEdges(const geometry_msgs::Pose& robot_pose,
                                        const std::vector<geometry_msgs::Point>& footprint,
                                        std::vector<geometry_msgs::Point>& out_footprint);

  void getFootprintList(const std::vector<EnvNAVXYTHETALAT3Dpt_t>& sbpl_path, const std::string& path_frame_id,
                        visualization_msgs::Marker& ma);

 

  inline double getYaw(double x, double y, double z, double w);
  inline double calculateAngle(double xA, double yA, double xB, double yB) {
    double angleRad = atan2(yB - yA, xB - xA);
    // double angleDeg = angleRad * 180.0 / M_PI;
    return angleRad;
  }

  bool loadPathwayData(const string& filename);

  bool findNearestPoseOfPath(vector<Pose>& posesOnPathWay, Pose& PoseToCheck, Pose& PoseResult);

  void order_msg_handle(const vda5050_msgs::Order::ConstPtr& msg);
  bool HandleSetPlanWithOrder(custom_planner::PlanWithOrder::Request& request, custom_planner::PlanWithOrder::Response& response);

  bool makePlanWithOrder(vda5050_msgs::Order msg, uint8_t& status, string& message);

  bool makePlanForRetry(std::vector<geometry_msgs::PoseStamped>& current_plan, 
    geometry_msgs::PoseStamped& pose_A, geometry_msgs::PoseStamped& pose_B, 
    geometry_msgs::PoseStamped& pose_C, std::vector<geometry_msgs::PoseStamped>& result_plan);

  bool findCenterOfCurve(geometry_msgs::PoseStamped& pose_A, geometry_msgs::PoseStamped& pose_B, geometry_msgs::PoseStamped& pose_C);

  bool isThetaValid(double theta);

  bool curveIsValid(int degree, const std::vector<double> &knot_vector,
                  vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& control_points);

  double computeDeltaAngleStartNode(double theta, Pose& startPose, Pose& next_Pose);

  double computeDeltaAngleStartNode(double thetaEnd, double thetaStart, Pose& Pose);

  double computeDeltaAngleStartOfPlan(double theta, geometry_msgs::Pose& startPose, geometry_msgs::Pose& next_Pose);

  double computeDeltaAngleStartOfPlan(double thetaEnd, double thetaStart, geometry_msgs::Pose& Pose);

  double computeDeltaAngleEndNode(double theta, Pose& endPose, Pose& prev_Pose);

  double computeDeltaAngleEndOfPlan(double theta, geometry_msgs::Pose& endPose, geometry_msgs::Pose& prev_Pose);

  void setYawAllPosesOnEdge(vector<Pose>& posesOnEdge, bool reverse);

  double computeDeltaAngle(Pose& Pose1, Pose& Pose2);

  void test_print_plan_result();

  Spline_Inf* input_spline_inf;
  Curve_common* CurveDesign;
  Pathway* pathway;   
  userParams* userParams_; 
  Pose* startPose;
  vector<Pose> posesOnPathWay;      
  Pose start_on_path;  
  std::map<string, OrderNode> orderNodes;  

  vda5050_msgs::Order order_msg_;
  uint16_t start_on_path_index;
  bool initialized_;

  SBPLPlanner* planner_;
  EnvironmentNAVXYTHETALAT* env_;
  
  std::string planner_type_; /**< sbpl method to use for planning.  choices are ARAPlanner and ADPlanner */

  double allocated_time_; /**< amount of time allowed for search */
  double initial_epsilon_; /**< initial epsilon for beginning the anytime search */

  std::string environment_type_; /** what type of environment in which to plan.  choices are 2D and XYThetaLattice. */ 
  std::string cost_map_topic_; /** what topic is being used for the costmap topic */

  bool forward_search_; /** whether to use forward or backward search */
  std::string primitive_filename_; /** where to find the motion primitives for the current robot */
  int force_scratch_limit_; /** the number of cells that have to be changed in the costmap to force the planner to plan from scratch even if its an incremental planner */

  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned char circumscribed_cost_;
  unsigned char sbpl_cost_multiplier_;

  bool publish_footprint_path_;
  int visualizer_skip_poses_;

  bool allow_unknown_;

  std::string name_;
  costmap_2d::Costmap2DROS* costmap_ros_; /**< manages the cost map for us */
  std::vector<geometry_msgs::Point> footprint_;
  unsigned int current_env_width_;
  unsigned int current_env_height_;

  ros::Subscriber order_msg_sub_;
  ros::Publisher plan_pub_;
  ros::Publisher stats_publisher_;

  vector<ros::ServiceServer> service_servers_;
  
  ros::Publisher sbpl_plan_footprint_pub_;  
  boost::mutex mutex_;
};
};

#endif

