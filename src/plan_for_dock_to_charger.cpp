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
#include <geometry_msgs/Pose2D.h>

using namespace std;

namespace plan_dock_to_charger
{
  void modifyYaw(double& yaw)
  {
    while (yaw < -M_PI)
        yaw += 2.0 * M_PI;
    while (yaw > M_PI)
        yaw -= 2.0 * M_PI;
  }

  inline double calculateAngle(double xA, double yA, double xB, double yB) {
    double deltaX = xB - xA;
    double deltaY = yB - yA;
    double angleRad = 0;
    if(deltaX!=0)
    {
      angleRad = atan2(deltaY, deltaX);
      // double angleDeg = angleRad * 180.0 / M_PI;
    }
    return angleRad;
  }

  inline double getYaw(double x, double y, double z, double w){
      // yaw (z-axis rotation)
      double siny_cosp = 2 * (w * z + x * y);
      double cosy_cosp = 1 - 2 * (y * y + z * z);
      double yaw = std::atan2(siny_cosp, cosy_cosp);
      return yaw;
  }

  bool isThetaValid(double theta)
  {
      bool result = false;
      if(theta < -M_PI || theta > M_PI) result = false;
      else result = true;
      return result;
  }

  double computeDeltaAngleStartOfPlan(double theta, geometry_msgs::Pose2D& startPose, geometry_msgs::Pose2D& next_Pose)
  {
      double delta_angle = 0;
      if(isThetaValid(theta))
      {
        double xAB = next_Pose.x - startPose.x;
        double yAB = next_Pose.y - startPose.y;
        double d = sqrt(xAB*xAB + yAB*yAB);
        double xC = startPose.x + d*cos(theta);
        double yC = startPose.y + d*sin(theta);
        double xAC = xC-startPose.x;
        double yAC = yC-startPose.y;
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

  double computeDeltaAngleEndOfPlan(double theta, geometry_msgs::Pose2D& endPose, geometry_msgs::Pose2D& prev_Pose)
  {
      double delta_angle = 0;
      if(isThetaValid(theta))
      {
          double xAB =endPose.x-prev_Pose.x;
          double yAB = endPose.y-prev_Pose.y;
          double d = sqrt(xAB*xAB + yAB*yAB);
          double xC =endPose.x + d*cos(theta);
          double yC = endPose.y + d*sin(theta);
          double xBC = xC-endPose.x;
          double yBC = yC-endPose.y;
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

  geometry_msgs::Pose2D findPerpendicularIntersection(geometry_msgs::Pose2D& A, geometry_msgs::Pose2D& B, geometry_msgs::Pose2D& C) {
      double x1 = B.x, y1 = B.y;
      double x2 = C.x, y2 = C.y;
      double x3 = A.x, y3 = A.y;

      // Kiểm tra nếu điểm A trùng với B hoặc C
      double tolerance = 1e-3; // Sai số chấp nhận được cho kiểm tra dấu phẩy động
      if (std::abs(x3 - x1) < tolerance && std::abs(y3 - y1) < tolerance) {
          // A trùng với B, trả về A
          return A;
      }
      if (std::abs(x3 - x2) < tolerance && std::abs(y3 - y2) < tolerance) {
          // A trùng với C, trả về A
          return A;
      }

      // Tính toán hệ số của phương trình đường thẳng BC
      double a1 = y2 - y1;
      double b1 = x1 - x2;
      double c1 = a1 * x1 + b1 * y1;

      // Kiểm tra nếu điểm A nằm trên đường thẳng BC
      if (std::abs(a1 * x3 + b1 * y3 - c1) < tolerance) {
          // Điểm A nằm trên đường BC, trả về chính A
          return A;
      }

      // Tính toán hệ số của phương trình đường thẳng vuông góc với BC qua điểm A
      double a2 = b1;
      double b2 = -a1;
      double c2 = a2 * x3 + b2 * y3;

      double determinant = a1 * b2 - a2 * b1;

      geometry_msgs::Pose2D result;

      if (determinant == 0) {
          // Hai đường thẳng song song hoặc trùng nhau, điều này không xảy ra với đường vuông góc
          return result;
      } else {
          double x = (b2 * c1 - b1 * c2) / determinant;
          double y = (a1 * c2 - a2 * c1) / determinant;

          // Gán kết quả vào result
          result.x = x;
          result.y = y;

          result.theta = A.theta; // Giữ nguyên theta của A
          
          return result;
      }
  }

  // Hàm chia đoạn thẳng AB thành các đoạn có độ dài d
  std::vector<geometry_msgs::Pose2D> divideSegment(geometry_msgs::Pose2D& A, geometry_msgs::Pose2D& B, double d) {
      std::vector<geometry_msgs::Pose2D> Poses;
      double xAB = B.x - A.x;
      double yAB = B.y - A.y;
      double length = sqrt(xAB*xAB + yAB*yAB);
      if(length > d)
      {
        Poses.push_back(A); // Thêm điểm A vào vector trước khi chia
        
        int segments = length / d;

        // Tính toán tọa độ của các điểm trên đoạn AB
        double ratio = d / length;
        for (int i = 1; i <= segments; ++i) {
            geometry_msgs::Pose2D p;
            double p_x = A.x + (B.x - A.x) * ratio * i;
            double p_y = A.y + (B.y - A.y) * ratio * i;
            p.x = p_x;
            p.y = p_y;
            Poses.push_back(p);
        }
        
        if(!Poses.empty()&&(Poses.back().x!=B.x || Poses.back().y!=B.y))
        {
            Poses.push_back(B); // Thêm điểm B vào vector sau khi chia
        }    

        // Tính góc cho từng pose trên đoạn AB
        if(computeDeltaAngleEndOfPlan(B.theta,
          Poses.back(), Poses[Poses.size() - 2]) <= 0.2617993878) // <= 15 degree
        {
          for(int i = 0; i<((int)Poses.size()-1); i++)
          {
              double theta = calculateAngle(Poses[i].x, Poses[i].y, 
                                              Poses[i+1].x, Poses[i+1].y);
              Poses[i].theta = (theta); 
          }
          Poses.back().theta = B.theta;
        }
        else if(computeDeltaAngleEndOfPlan(B.theta,
                Poses.back(), Poses[Poses.size() - 2]) >= 2.8797932658) // >= 165 degree
        {       
          for(int i = (int)Poses.size() -1; i>0; i--)
          {
              double theta = calculateAngle(Poses[i].x, Poses[i].y, 
                                              Poses[i-1].x, Poses[i-1].y);
              Poses[i].theta = (theta);     
          }
          Poses.front().theta = A.theta;
        }
      }
      else
      {
        Poses.push_back(A);
        Poses.push_back(B);
      }
      return Poses;
  }

  // Hàm Tìm tâm C của cung tròn AB khi biết Pose tại điểm B: khi tìm thành công điểm C thì hàm trả về True
      // pose_A: điểm start của cung tròn
      // pose_B: điểm đích trên cung tròn
      // pose_C: tâm của cung tròn AB (kết quả)

  bool findCenterOfCurve(geometry_msgs::Pose2D& pose_A, geometry_msgs::Pose2D& pose_B, geometry_msgs::Pose2D& pose_C)
  {
    // nếu hướng của vector AB và hướng của pose_B tạo với nhau một góc ~0 độ hoặc ~180 độ -> điểm C sẽ gần xấp xỉ với trung điểm của đoạn thẳng AB.
    if((computeDeltaAngleEndOfPlan(pose_B.theta,
        pose_B, pose_A) >= 3.13 && 
        computeDeltaAngleEndOfPlan(pose_B.theta,
        pose_B, pose_A) <= M_PI) ||
        (computeDeltaAngleEndOfPlan(pose_B.theta,
        pose_B, pose_A) <= 0.1745 && 
        computeDeltaAngleEndOfPlan(pose_B.theta,
        pose_B, pose_A) >= 0))
    {
      pose_C.x = (pose_A.x + pose_B.x)/2;
      pose_C.y = (pose_A.y + pose_B.y)/2;
    }
    else
    {
      double x_R = pose_A.x;
      double y_R = pose_A.y;
      double x_G = pose_B.x;
      double y_G = pose_B.y;
      double phi_vG = pose_B.theta;
      double x_H = (x_R+x_G)/2;
      double y_H = (y_R+y_G)/2;
      double m_vG = tan(phi_vG);
      double m_G_n_vG = -1/m_vG;
      double b_G_n_vG = y_G-m_G_n_vG*x_G;
      double m_RG =(y_G-y_R)/(x_G-x_R);
      double b_RG = y_R-m_RG*x_R;
      double m_H_n_RG = -1/m_RG;
      double b_H_n_RG = y_H-m_H_n_RG*x_H;
      pose_C.x = (b_H_n_RG-b_G_n_vG)/(m_G_n_vG-m_H_n_RG);
      pose_C.y = (b_H_n_RG*m_G_n_vG-b_G_n_vG*m_H_n_RG)/(m_G_n_vG-m_H_n_RG);
    }
    return true;
  }

  // Hàm tạo tuyến đường có dạng cung tròn AB
      // pose_A: điểm start của cung tròn
      // pose_B: điểm đích trên cung tròn
      // pose_C: tâm của cung tròn AB
      // result_plan: cung tròn AB (kết quả)
  bool makeCurvePlan(geometry_msgs::Pose2D& pose_A, geometry_msgs::Pose2D& pose_B, 
      geometry_msgs::Pose2D& pose_C, std::vector<geometry_msgs::Pose2D>& result_plan)
  {
    std::vector<geometry_msgs::Pose2D> plan1;

    double xCA = pose_A.x - pose_C.x;
    double yCA = pose_A.y - pose_C.y;
    double xCB = pose_B.x - pose_C.x;
    double yCB = pose_B.y - pose_C.y;
    double rCA = sqrt(xCA * xCA + yCA * yCA);
    double rCB = sqrt(xCB * xCB + yCB * yCB);
    if (abs(rCA - rCB) > 0.008)
    {
      ROS_ERROR("pose_C is not Center of Curve AB");
      return false;
    }

    double cos_ACB = (xCA * xCB + yCA * yCB) / (rCA * rCB);
    if (cos_ACB > 1)
      cos_ACB = 1;
    else if (cos_ACB < (-1))
      cos_ACB = -1;
    double angleACB = acos(cos_ACB);
    double angle_interval = 0.01;
    // tính góc của vector CA:
    double angleCA = atan2(yCA, xCA);

    // check thử xem chiều góc quét từ A -> B thì angleCA + delta_angle hay angleCA - delta_angle
    bool is_increase_angle = false;
    double check_angle = angleCA + 50 * angle_interval * angleACB;
    double xA1 = pose_C.x + rCA * cos(check_angle);
    double yA1 = pose_C.y + rCA * sin(check_angle);
    double xCA1 = xA1 - pose_C.x;
    double yCA1 = yA1 - pose_C.y;
    double cos_A1CB = (xCA1 * xCB + yCA1 * yCB) / (rCA * rCB);
    if (cos_A1CB > 1)
      cos_A1CB = 1;
    else if (cos_A1CB < (-1))
      cos_A1CB = -1;
    double angleA1CB = acos(cos_A1CB);
    if (angleA1CB > angleACB)
    {
      is_increase_angle = false;
    }
    else if (angleA1CB < angleACB)
    {
      is_increase_angle = true;
    }
    else
    {
      ROS_ERROR("Curve AB is too short, cannot compute plan");
      return false;
    }
    if (is_increase_angle)
    {
      for (double i = 0; i <= 1; i += angle_interval)
      {
        double angle_tmp = angleCA + angleACB * i;
        double xP = pose_C.x + rCA * cos(angle_tmp);
        double yP = pose_C.y + rCA * sin(angle_tmp);
        geometry_msgs::Pose2D p;
        p.x = xP;
        p.y = yP;
        plan1.push_back(p);
      }
    }
    else
    {
      for (double i = 0; i <= 1; i += angle_interval)
      {
        double angle_tmp = angleCA - angleACB * i;
        double xP = pose_C.x + rCA * cos(angle_tmp);
        double yP = pose_C.y + rCA * sin(angle_tmp);
        geometry_msgs::Pose2D p;
        p.x = xP;
        p.y = yP;
        plan1.push_back(p);
      }
    }
    if (!plan1.empty() && plan1.size() > 2)
    {
      if (computeDeltaAngleEndOfPlan(pose_B.theta,
                                      plan1.back(), plan1[plan1.size() - 2]) <= 1.3962634016) // <= 80 degree
      {
        for (int i = 0; i < ((int)plan1.size() - 1); i++)
        {
          double theta = calculateAngle(plan1[i].x, plan1[i].y,
                                        plan1[i + 1].x, plan1[i + 1].y);
          plan1[i].theta = (theta);
        }
        plan1.back().theta = pose_B.theta;                    
      }
      else if(computeDeltaAngleEndOfPlan(pose_B.theta,
                                plan1.back(), plan1[plan1.size() - 2]) >= 1.745329252) // >= 100 degree
      {
        for (int i = (int)plan1.size() - 1; i > 0; i--)
        {
          double theta = calculateAngle(plan1[i].x, plan1[i].y,
                                        plan1[i - 1].x, plan1[i - 1].y);
          plan1[i].theta = (theta);
        }
        plan1.front().theta = plan1[1].theta;
      }
      else
      {
        ROS_ERROR("Pose_A yaw or Pose_B yaw is invalid value");
        return false;
      }
    }
    else
    {
      ROS_ERROR("Curve AB is too short, cannot compute plan");
      return false;
    }
    result_plan = plan1;
    if(!result_plan.empty())
    return true;
    else
    {
      ROS_ERROR("[makeCurvePlan] failed to make plan");           
      return false;
    }
  }

  // Hàm gọi make plan : tạo tuyến đường robot đi đến vị trí sạc 
  // khi tạo thành công plan thì hàm trả về True, không thành công thì trả về False và có hiện cảnh báo nguyên nhân.
      // current_pose: pose của robot hiện tại trên map
      // charger_pose_on_map: pose của trạm sạc trên map
      // d_offset_min: khoảng cách từ shelf pose đến điểm offset pose tối thiểu để robot có thể đi vào trạm sạc
      // result_plan: vector chứa plan kết quả
  bool makePlanDockToCharger(geometry_msgs::Pose2D& current_pose, 
      geometry_msgs::Pose2D& charger_pose_on_map,
      double d_offset_min, bool robot_move_forward,
      std::vector<geometry_msgs::Pose2D>& result_plan)
  {
    if(d_offset_min<=0)
    {
      ROS_ERROR("[makePlanDockToCharger] d_offset_min is invalid");
      return false;
    }
    bool result = false;    
    std::vector<geometry_msgs::Pose2D> plan1;
    std::vector<geometry_msgs::Pose2D> plan2;
    if(robot_move_forward) // robot move forward
    {
      double shelf_pose_yaw = charger_pose_on_map.theta;
      modifyYaw(shelf_pose_yaw);
      double goal_pose_yaw = shelf_pose_yaw + M_PI;
      modifyYaw(goal_pose_yaw);
      geometry_msgs::Pose2D goal_pose;
      goal_pose = charger_pose_on_map;
      goal_pose.theta = goal_pose_yaw;
      geometry_msgs::Pose2D pose_offset_min;                                            
      pose_offset_min.x = charger_pose_on_map.x + d_offset_min*cos(shelf_pose_yaw);
      pose_offset_min.y = charger_pose_on_map.y + d_offset_min*sin(shelf_pose_yaw);
      pose_offset_min.theta = goal_pose.theta;
      geometry_msgs::Pose2D pose_intersection = findPerpendicularIntersection(current_pose, charger_pose_on_map, pose_offset_min);
      double d_shelfpose_to_intersection = std::sqrt(std::pow(pose_intersection.x - charger_pose_on_map.x, 2) + 
        std::pow(pose_intersection.y - charger_pose_on_map.y, 2));
      double delta_d1 = d_shelfpose_to_intersection - d_offset_min;
      if(delta_d1 <= 0.1 && delta_d1 >= -0.1)
      {
        plan1.clear();
        plan1 = divideSegment(pose_offset_min, goal_pose, 0.02);
        result_plan = plan1;
        if(!result_plan.empty())
        {
          result = true;
          return true;
          ROS_INFO("[makePlanDockToCharger] make plan TH1");
        }
        else
        {
          ROS_ERROR("[makePlanDockToCharger] failed to make plan TH1");
          return false;
        }
      }
      else if(delta_d1 > 0.1)
      {
        geometry_msgs::Pose2D pose_B = pose_offset_min;
        pose_B.theta = goal_pose.theta;
        // nếu hướng của vector AB và hướng của pose_B tạo với nhau một góc ~0 độ hoặc ~180 độ -> cung tròn AB sẽ gần như là một đọan thẳng
        if((computeDeltaAngleEndOfPlan(pose_B.theta,
            pose_B, current_pose) >= 3.13 && 
            computeDeltaAngleEndOfPlan(pose_B.theta,
            pose_B, current_pose) <= M_PI) ||
            (computeDeltaAngleEndOfPlan(pose_B.theta,
            pose_B, current_pose) <= 0.1745 && 
            computeDeltaAngleEndOfPlan(pose_B.theta,
            pose_B, current_pose) >= 0))
        {
          plan1.clear();
          plan2.clear();
          plan1 = divideSegment(current_pose, pose_B, 0.02);
          plan2 = divideSegment(pose_B, goal_pose, 0.02);
          if(!plan1.empty() && !plan2.empty())
          { 
            result_plan.assign(plan1.begin(), plan1.end());
            result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
          }
          if(!result_plan.empty())
          {
            ROS_INFO("[makePlanDockToCharger] make plan TH2");
            result = true;
            return true;
          }
          else
          {
            ROS_ERROR("[makePlanDockToCharger] failed to make plan TH2");           
            return false;
          }
        }
        else
        {
          plan1.clear();
          plan2.clear();
          // Tính toán đoạn đường cong AB
          geometry_msgs::Pose2D pose_C;
          geometry_msgs::Pose2D pose_A = current_pose;
          if(findCenterOfCurve(pose_A, pose_B, pose_C))
          {
            if(makeCurvePlan(pose_A, pose_B, pose_C, plan1))
            {
              plan2 = divideSegment(pose_B, goal_pose, 0.02);
              if(!plan1.empty() && !plan2.empty())
              {
                result_plan.assign(plan1.begin(), plan1.end());
                result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
              }
              if(!result_plan.empty())
              {
                ROS_INFO("[makePlanDockToCharger] make plan TH3");
                result = true;
                return true;
              }
              else
              {
                ROS_ERROR("[makePlanDockToCharger] failed to make plan TH3");           
                return false;
              }
            }
            else
            {
              plan1 = divideSegment(current_pose, pose_B, 0.02);
              plan2 = divideSegment(pose_B, goal_pose, 0.02);
              if(!plan1.empty() && !plan2.empty())
              {
                result_plan.assign(plan1.begin(), plan1.end());
                result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
              }
              if(!result_plan.empty())
              {
                ROS_INFO("[makePlanDockToCharger] make plan TH4");
                result = true;
                return true;
              }
              else
              {
                ROS_ERROR("[makePlanDockToCharger] failed to make plan TH4");           
                return false;
              }
            }
          }
          else
          {          
            plan1 = divideSegment(current_pose, pose_B, 0.02);
            plan2 = divideSegment(pose_B, goal_pose, 0.02);
            if(!plan1.empty() && !plan2.empty())
            {
              result_plan.assign(plan1.begin(), plan1.end());
              result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
            }
            if(!result_plan.empty())
            {
              ROS_INFO("[makePlanDockToCharger] make plan TH5");
              result = true;
              return true;
            }
            else
            {
              ROS_ERROR("[makePlanDockToCharger] failed to make plan TH5");           
              return false;
            }
          }
        }
      }
      else
      {
        double pose_intersection_yaw = calculateAngle(current_pose.x, current_pose.y,
        pose_intersection.x, pose_intersection.y);
        pose_intersection.theta = pose_intersection_yaw;
        plan1.clear();
        plan2.clear();
        plan1 = divideSegment(current_pose, pose_intersection, 0.02);    
        pose_intersection.theta = goal_pose_yaw; 
        plan2 = divideSegment(pose_intersection, goal_pose, 0.02);
        if(!plan1.empty() && !plan2.empty())
        {     
          result_plan.assign(plan1.begin(), plan1.end());
          result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
        }
        if(!result_plan.empty())
        {
          ROS_INFO("[makePlanDockToCharger] make plan TH6");
          result = true;
          return true;
        }
        else
        {
          ROS_ERROR("[makePlanDockToCharger] failed to make plan TH6");            
          return false;
        }
      }
    }
    else // robot move backward
    {
      double shelf_pose_yaw = charger_pose_on_map.theta;
      modifyYaw(shelf_pose_yaw);
      double goal_pose_yaw = shelf_pose_yaw;
      modifyYaw(goal_pose_yaw);
      geometry_msgs::Pose2D goal_pose;
      goal_pose = charger_pose_on_map;
      goal_pose.theta = goal_pose_yaw;
      geometry_msgs::Pose2D pose_offset_min;                                            
      pose_offset_min.x = charger_pose_on_map.x + d_offset_min*cos(shelf_pose_yaw);
      pose_offset_min.y = charger_pose_on_map.y + d_offset_min*sin(shelf_pose_yaw);
      pose_offset_min.theta = goal_pose.theta;
      geometry_msgs::Pose2D pose_intersection = findPerpendicularIntersection(current_pose, charger_pose_on_map, pose_offset_min);
      double d_shelfpose_to_intersection = std::sqrt(std::pow(pose_intersection.x - charger_pose_on_map.x, 2) + 
        std::pow(pose_intersection.y - charger_pose_on_map.y, 2));
      double delta_d1 = d_shelfpose_to_intersection - d_offset_min;
      if(delta_d1 <= 0.1 && delta_d1 >= -0.1)
      {
        plan1.clear();
        plan1 = divideSegment(pose_offset_min, goal_pose, 0.02);
        result_plan = plan1;
        if(!result_plan.empty())
        {
          result = true;
          return true;
          ROS_INFO("[makePlanDockToCharger] make plan TH7");
        }
        else
        {
          ROS_ERROR("[makePlanDockToCharger] failed to make plan TH7");
          return false;
        }
      }
      else if(delta_d1 > 0.1)
      {
        geometry_msgs::Pose2D pose_B = pose_offset_min;
        pose_B.theta = goal_pose.theta;
        // nếu hướng của vector AB và hướng của pose_B tạo với nhau một góc ~0 độ hoặc ~180 độ -> cung tròn AB sẽ gần như là một đọan thẳng
        if((computeDeltaAngleEndOfPlan(pose_B.theta,
            pose_B, current_pose) >= 3.13 && 
            computeDeltaAngleEndOfPlan(pose_B.theta,
            pose_B, current_pose) <= M_PI) ||
            (computeDeltaAngleEndOfPlan(pose_B.theta,
            pose_B, current_pose) <= 0.1745 && 
            computeDeltaAngleEndOfPlan(pose_B.theta,
            pose_B, current_pose) >= 0))
        {
          plan1.clear();
          plan2.clear();
          plan1 = divideSegment(current_pose, pose_B, 0.02);
          plan2 = divideSegment(pose_B, goal_pose, 0.02);
          if(!plan1.empty() && !plan2.empty())
          { 
            result_plan.assign(plan1.begin(), plan1.end());
            result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
          }
          if(!result_plan.empty())
          {
            ROS_INFO("[makePlanDockToCharger] make plan TH8");
            result = true;
            return true;
          }
          else
          {
            ROS_ERROR("[makePlanDockToCharger] failed to make plan TH8");           
            return false;
          }
        }
        else
        {
          plan1.clear();
          plan2.clear();
          // Tính toán đoạn đường cong AB
          geometry_msgs::Pose2D pose_C;
          geometry_msgs::Pose2D pose_A = current_pose;
          geometry_msgs::Pose2D pose_B_opposite;
          pose_B_opposite = pose_B;
          double pose_B_opposite_yaw = pose_B.theta + M_PI;
          modifyYaw(pose_B_opposite_yaw);
          pose_B_opposite.theta = pose_B_opposite_yaw;
          if(findCenterOfCurve(pose_A, pose_B_opposite, pose_C))
          {
            if(makeCurvePlan(pose_A, pose_B, pose_C, plan1))
            {
              plan2 = divideSegment(pose_B, goal_pose, 0.02);
              if(!plan1.empty() && !plan2.empty())
              {
                result_plan.assign(plan1.begin(), plan1.end());
                result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
              }
              if(!result_plan.empty())
              {
                ROS_INFO("[makePlanDockToCharger] make plan TH9");
                result = true;
                return true;
              }
              else
              {
                ROS_ERROR("[makePlanDockToCharger] failed to make plan TH9");           
                return false;
              }
            }
            else
            {
              plan1 = divideSegment(current_pose, pose_B, 0.02);
              plan2 = divideSegment(pose_B, goal_pose, 0.02);
              if(!plan1.empty() && !plan2.empty())
              {
                result_plan.assign(plan1.begin(), plan1.end());
                result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
              }
              if(!result_plan.empty())
              {
                ROS_INFO("[makePlanDockToCharger] make plan TH10");
                result = true;
                return true;
              }
              else
              {
                ROS_ERROR("[makePlanDockToCharger] failed to make plan TH10");           
                return false;
              }
            }
          }
          else
          {          
            plan1 = divideSegment(current_pose, pose_B, 0.02);
            plan2 = divideSegment(pose_B, goal_pose, 0.02);
            if(!plan1.empty() && !plan2.empty())
            {
              result_plan.assign(plan1.begin(), plan1.end());
              result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
            }
            if(!result_plan.empty())
            {
              ROS_INFO("[makePlanDockToCharger] make plan TH11");
              result = true;
              return true;
            }
            else
            {
              ROS_ERROR("[makePlanDockToCharger] failed to make plan TH11");           
              return false;
            }
          }
        }
      }
      else
      {
        double pose_intersection_yaw = calculateAngle(pose_intersection.x, pose_intersection.y,
        current_pose.x, current_pose.y);
        pose_intersection.theta = pose_intersection_yaw;
        plan1.clear();
        plan2.clear();
        plan1 = divideSegment(current_pose, pose_intersection, 0.02);    
        pose_intersection.theta = goal_pose_yaw; 
        plan2 = divideSegment(pose_intersection, goal_pose, 0.02);
        if(!plan1.empty() && !plan2.empty())
        {     
          result_plan.assign(plan1.begin(), plan1.end());
          result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
        }
        if(!result_plan.empty())
        {
          ROS_INFO("[makePlanDockToCharger] make plan TH12");
          result = true;
          return true;
        }
        else
        {
          ROS_ERROR("[makePlanDockToCharger] failed to make plan TH12");            
          return false;
        }
      }
    }
    return result;
  }
}
