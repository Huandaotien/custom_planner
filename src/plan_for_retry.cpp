#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/PoseStamped.h>

using namespace std;

inline double calculateAngle(double xA, double yA, double xB, double yB) {
    double angleRad = atan2(yB - yA, xB - xA);
    // double angleDeg = angleRad * 180.0 / M_PI;
    return angleRad;
}

inline double getYaw(double x, double y, double z, double w){
    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
}

inline double calculateAngle(double xA, double yA, double xB, double yB) {
    double angleRad = atan2(yB - yA, xB - xA);
    // double angleDeg = angleRad * 180.0 / M_PI;
    return angleRad;
}

bool isThetaValid(double theta)
{
    bool result = false;
    if(theta < -M_PI || theta > M_PI) result = false;
    else result = true;
    return result;
}

double computeDeltaAngleStartOfPlan(double theta, geometry_msgs::Pose& startPose, geometry_msgs::Pose& next_Pose)
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
      delta_angle = acos(cos_a);
      // delta_angle = delta_angle*180/M_PI;
      // ROS_WARN("xC: %f, yC: %f", xC, yC);
      // ROS_WARN("dAB: %f", dAB);
      // ROS_WARN("delta_angle: %f", delta_angle);
    }   
    return delta_angle;    
}

double computeDeltaAngleEndOfPlan(double theta, geometry_msgs::Pose& endPose, geometry_msgs::Pose& prev_Pose)
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
        delta_angle = acos(cos_a);
        // delta_angle = delta_angle*180/M_PI;
        // ROS_WARN("xC: %f, yC: %f", xC, yC);
        // ROS_WARN("dAB: %f", dAB);
        // ROS_WARN("delta_angle: %f", delta_angle);
    }  
    return delta_angle;
}

// Hàm chia đoạn thẳng AB thành các đoạn có độ dài d
vector<geometry_msgs::PoseStamped> divideSegment(geometry_msgs::PoseStamped& A, geometry_msgs::PoseStamped& B, double d) {
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
      if(computeDeltaAngleStartOfPlan(getYaw(A.pose.orientation.x, A.pose.orientation.y, A.pose.orientation.z, A.pose.orientation.w),
        Poses.front().pose, Poses[1].pose) <= 0.872664626 &&  
        computeDeltaAngleEndOfPlan(getYaw(B.pose.orientation.x, B.pose.orientation.y, B.pose.orientation.z, B.pose.orientation.w),
        Poses.back().pose, Poses[Poses.size() - 2].pose) <= 0.872664626) // <= 50 degree
      {
        for(int i = 0; i<((int)Poses.size()-1); i++)
        {
            double theta = calculateAngle(Poses[i].pose.position.x, Poses[i].pose.position.y, 
                                          Poses[i+1].pose.position.x, Poses[i+1].pose.position.y);
            Poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);   
        }
        Poses.back().pose.orientation = B.pose.orientation;
      }
      else if(computeDeltaAngleStartOfPlan(getYaw(A.pose.orientation.x, A.pose.orientation.y, A.pose.orientation.z, A.pose.orientation.w),
              Poses.front().pose, Poses[1].pose) >= 2.2689280276 &&
              computeDeltaAngleEndOfPlan(getYaw(B.pose.orientation.x, B.pose.orientation.y, B.pose.orientation.z, B.pose.orientation.w),
              Poses.back().pose, Poses[Poses.size() - 2].pose) >= 2.2689280276) // >= 130 degree
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

// Hàm gọi make plan : tạo tuyến đường đi quay lại điểm retry có trong plan hiện tại và đi tới điểm goal mới theo cung tròn. 
// khi tạo thành công plan thì hàm trả về True, không thành công thì trả về False và có hiện cảnh báo nguyên nhân.
    // current_plan: vector chứa plan hiện tại
    // pose_A: điểm quay lại để retry
    // pose_B: điểm goal mới, có hướng thẳng với palet hiện tại
    // pose_C: tâm của cung tròn AB
    // result_plan: vector chứa plan kết quả

bool makePlanForRetry(std::vector<geometry_msgs::PoseStamped>& current_plan, 
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
      planSegment_AB = divideSegment(pose_A, pose_B, 0.008);
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
      if(abs(rCA-rCB)>0.008)
      {
        ROS_WARN("pose_C is not Center of Curve AB");
        return false;
      }            

      double cos_ACB = (xCA*xCB + yCA*yCB)/(rCA*rCB);
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

// Hàm Tìm tâm C của cung tròn AB khi biết Pose tại điểm B: khi tìm thành công điểm C thì hàm trả về True
    // pose_A: điểm start của cung tròn
    // pose_B: điểm đích trên cung tròn
    // pose_C: tâm của cung tròn AB (kết quả)

bool findCenterOfCurve(geometry_msgs::PoseStamped& pose_A, geometry_msgs::PoseStamped& pose_B, geometry_msgs::PoseStamped& pose_C)
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
