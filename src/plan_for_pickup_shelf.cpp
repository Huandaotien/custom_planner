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

geometry_msgs::PoseStamped findPerpendicularIntersection(geometry_msgs::PoseStamped& A, geometry_msgs::PoseStamped& B, geometry_msgs::PoseStamped& C) {
    double x1 = B.pose.position.x, y1 = B.pose.position.y;
    double x2 = C.pose.position.x, y2 = C.pose.position.y;
    double x3 = A.pose.position.x, y3 = A.pose.position.y;

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

    geometry_msgs::PoseStamped result;

    if (determinant == 0) {
        // Hai đường thẳng song song hoặc trùng nhau, điều này không xảy ra với đường vuông góc
        return result;
    } else {
        double x = (b2 * c1 - b1 * c2) / determinant;
        double y = (a1 * c2 - a2 * c1) / determinant;

        // Gán kết quả vào result
        result.pose.position.x = x;
        result.pose.position.y = y;
        result.pose.position.z = A.pose.position.z; // Có thể giữ nguyên giá trị z của A

        result.pose.orientation = A.pose.orientation; // Giữ nguyên orientation của A
        result.header = A.header; // Gán lại header từ A
        
        return result;
    }
}

// Hàm chia đoạn thẳng AB thành các đoạn có độ dài d
std::vector<geometry_msgs::PoseStamped> divideSegment(geometry_msgs::PoseStamped& A, geometry_msgs::PoseStamped& B, double d) {
    std::vector<geometry_msgs::PoseStamped> Poses;
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
              Poses.back().pose, Poses[Poses.size() - 2].pose) >= 1.7453292526) // >= 100 degree
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

// Hàm gọi make plan : tạo tuyến đường robot đi vào lấy xe hàng 
// khi tạo thành công plan thì hàm trả về True, không thành công thì trả về False và có hiện cảnh báo nguyên nhân.
    // current_pose: pose của robot hiện tại trên map
    // shelf_pose_on_map: pose của xe hàng trên map
    // d_instersection: khoảng cách từ điểm intersection đến điểm offset
    // d_offset_min: khoảng cách từ shelf pose đến điểm offset pose tối thiểu để robot có thể vào lấy hàng
    // result_plan: vector chứa plan kết quả
bool makePlanPickupShelf(geometry_msgs::PoseStamped& current_pose, 
    geometry_msgs::PoseStamped& shelf_pose_on_map, double d_intersection, 
    double d_offset_min, bool robot_move_forward, std::vector<geometry_msgs::PoseStamped>& result_plan)
{
    bool result = false;
    std::vector<geometry_msgs::PoseStamped> plan1;
    std::vector<geometry_msgs::PoseStamped> plan2;
    if(robot_move_forward) // robot move forward
    {
      double shelf_pose_yaw = getYaw(shelf_pose_on_map.pose.orientation.x,
                                      shelf_pose_on_map.pose.orientation.y,
                                      shelf_pose_on_map.pose.orientation.z,
                                      shelf_pose_on_map.pose.orientation.w);
      modifyYaw(shelf_pose_yaw);
      double goal_pose_yaw = shelf_pose_yaw + M_PI;
      modifyYaw(goal_pose_yaw);
      geometry_msgs::PoseStamped goal_pose;
      goal_pose.pose.position = shelf_pose_on_map.pose.position;
      goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal_pose_yaw);
      geometry_msgs::PoseStamped pose_offset_min;                                            
      pose_offset_min.pose.position.x = shelf_pose_on_map.pose.position.x + d_offset_min*cos(shelf_pose_yaw);
      pose_offset_min.pose.position.y = shelf_pose_on_map.pose.position.y + d_offset_min*sin(shelf_pose_yaw);
      pose_offset_min.pose.orientation = goal_pose.pose.orientation;
      geometry_msgs::PoseStamped pose_intersection = findPerpendicularIntersection(current_pose, shelf_pose_on_map, pose_offset_min);
      double d_shelfpose_to_intersection = std::sqrt(std::pow(pose_intersection.pose.position.x - shelf_pose_on_map.pose.position.x, 2) + 
        std::pow(pose_intersection.pose.position.y - shelf_pose_on_map.pose.position.y, 2));
      double delta_d1 = d_shelfpose_to_intersection - d_offset_min;
      if(delta_d1 <= 0.1)
      {
        plan1.clear();
        plan1 = divideSegment(pose_offset_min, goal_pose, 0.02);
        result_plan = plan1;
        if(!result_plan.empty())
        return true;
        else
        {
          ROS_ERROR("[makePlanPickupShelf] failed to make plan TH1");
          return false;
        }
      }
      else
      {
        geometry_msgs::PoseStamped pose_B;
        pose_B.pose.position.x = pose_intersection.pose.position.x + d_intersection*cos(goal_pose_yaw);
        pose_B.pose.position.y = pose_intersection.pose.position.y + d_intersection*sin(goal_pose_yaw);
        pose_B.pose.orientation = goal_pose.pose.orientation;
        if(d_intersection <= 0.1)
        {
          double pose_intersection_yaw = calculateAngle(current_pose.pose.position.x, current_pose.pose.position.y,
            pose_intersection.pose.position.x, pose_intersection.pose.position.y);
          pose_intersection.pose.orientation = tf::createQuaternionMsgFromYaw(pose_intersection_yaw);
          plan1.clear();
          plan2.clear();
          plan1 = divideSegment(current_pose, pose_intersection, 0.02);    
          pose_intersection.pose.orientation = tf::createQuaternionMsgFromYaw(goal_pose_yaw); 
          plan2 = divideSegment(pose_intersection, goal_pose, 0.02);     
          result_plan.assign(plan1.begin(), plan1.end());
          result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());   
          if(!result_plan.empty())
          return true;
          else
          {
            ROS_ERROR("[makePlanPickupShelf] failed to make plan TH2");            
            return false;
          }
        }
        else
        {
          double d_shelfpose_to_intersection = std::sqrt(std::pow(pose_B.pose.position.x - shelf_pose_on_map.pose.position.x, 2) + 
            std::pow(pose_B.pose.position.y - shelf_pose_on_map.pose.position.y, 2));
          double delta2 = d_shelfpose_to_intersection - d_offset_min;
          if(delta2 > 0.1 &&
            computeDeltaAngleStartOfPlan(shelf_pose_yaw, shelf_pose_on_map.pose, pose_B.pose) <= 0.5235987756) // <= 30 degree
          {
            // nếu hướng của vector AB và hướng của pose_B tạo với nhau một góc ~0 độ hoặc ~180 độ -> cung tròn AB sẽ gần như là một đọan thẳng
            if((computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) >= 3.13 && 
                computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) <= M_PI) ||
                (computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) <= 0.1745 && 
                computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) >= 0))
            {
              plan1.clear();
              plan2.clear();
              plan1 = divideSegment(current_pose, pose_B, 0.02);
              plan2 = divideSegment(pose_B, goal_pose, 0.02);
              result_plan.assign(plan1.begin(), plan1.end());
              result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
              if(!result_plan.empty())
              return true;
              else
              {
                ROS_ERROR("[makePlanPickupShelf] failed to make plan TH3");           
                return false;
              }
            }
            else
            {
              // Tính toán đoạn đường cong AB
              geometry_msgs::PoseStamped pose_C;
              geometry_msgs::PoseStamped pose_A = current_pose;
              if(findCenterOfCurve(pose_A, pose_B, pose_C))
              {
                double xCA = pose_A.pose.position.x - pose_C.pose.position.x;
                double yCA = pose_A.pose.position.y - pose_C.pose.position.y;
                double xCB = pose_B.pose.position.x - pose_C.pose.position.x;
                double yCB = pose_B.pose.position.y - pose_C.pose.position.y;
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
                double xA1 = pose_C.pose.position.x + rCA * cos(check_angle);
                double yA1 = pose_C.pose.position.y + rCA * sin(check_angle);
                double xCA1 = xA1 - pose_C.pose.position.x;
                double yCA1 = yA1 - pose_C.pose.position.y;
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
                plan1.clear();
                plan2.clear();
                if (is_increase_angle)
                {
                  for (double i = 0; i <= 1; i += angle_interval)
                  {
                    double angle_tmp = angleCA + angleACB * i;
                    double xP = pose_C.pose.position.x + rCA * cos(angle_tmp);
                    double yP = pose_C.pose.position.y + rCA * sin(angle_tmp);
                    geometry_msgs::PoseStamped p;
                    p.pose.position.x = xP;
                    p.pose.position.y = yP;
                    p.pose.position.z = 0;
                    plan1.push_back(p);
                  }
                }
                else
                {
                  for (double i = 0; i <= 1; i += angle_interval)
                  {
                    double angle_tmp = angleCA - angleACB * i;
                    double xP = pose_C.pose.position.x + rCA * cos(angle_tmp);
                    double yP = pose_C.pose.position.y + rCA * sin(angle_tmp);
                    geometry_msgs::PoseStamped p;
                    p.pose.position.x = xP;
                    p.pose.position.y = yP;
                    p.pose.position.z = 0;
                    plan1.push_back(p);
                  }
                }
                if (!plan1.empty() && plan1.size() > 2)
                {
                  if (computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                                                  plan1.back().pose, plan1[plan1.size() - 2].pose) <= 1.3962634016) // <= 80 degree
                  {
                    for (int i = 0; i < ((int)plan1.size() - 1); i++)
                    {
                      double theta = calculateAngle(plan1[i].pose.position.x, plan1[i].pose.position.y,
                                                    plan1[i + 1].pose.position.x, plan1[i + 1].pose.position.y);
                      plan1[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
                    }
                    plan1.back().pose.orientation = pose_B.pose.orientation;                    
                  }
                  else if(computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                                            plan1.back().pose, plan1[plan1.size() - 2].pose) >= 1.745329252) // >= 100 degree
                  {
                    for (int i = (int)plan1.size() - 1; i > 0; i--)
                    {
                      double theta = calculateAngle(plan1[i].pose.position.x, plan1[i].pose.position.y,
                                                    plan1[i - 1].pose.position.x, plan1[i - 1].pose.position.y);
                      plan1[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
                    }
                    plan1.front().pose.orientation = plan1[1].pose.orientation;
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
                plan2 = divideSegment(pose_B, goal_pose, 0.02);
                result_plan.assign(plan1.begin(), plan1.end());
                result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
                if(!result_plan.empty())
                return true;
                else
                {
                  ROS_ERROR("[makePlanPickupShelf] failed to make plan TH4");           
                  return false;
                }
              }
              else
              {
                plan1 = divideSegment(current_pose, pose_B, 0.02);
                plan2 = divideSegment(pose_B, goal_pose, 0.02);
                result_plan.assign(plan1.begin(), plan1.end());
                result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
                if(!result_plan.empty())
                return true;
                else
                {
                  ROS_ERROR("[makePlanPickupShelf] failed to make plan TH5");           
                  return false;
                }
              }
            }
          }
          else
          {
            pose_B = pose_offset_min;
            pose_B.pose.orientation = goal_pose.pose.orientation;
            // nếu hướng của vector AB và hướng của pose_B tạo với nhau một góc ~0 độ hoặc ~180 độ -> cung tròn AB sẽ gần như là một đọan thẳng
            if((computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) >= 3.13 && 
                computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) <= M_PI) ||
                (computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) <= 0.1745 && 
                computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) >= 0))
            {
              plan1 = divideSegment(current_pose, pose_B, 0.02);
              plan2 = divideSegment(pose_B, goal_pose, 0.02);
              result_plan.assign(plan1.begin(), plan1.end());
              result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
              if(!result_plan.empty())
              return true;
              else
              {
                ROS_ERROR("[makePlanPickupShelf] failed to make plan TH6");           
                return false;
              }
            }
            else
            {
              // Tính toán đoạn đường cong AB
              geometry_msgs::PoseStamped pose_C;
              geometry_msgs::PoseStamped pose_A = current_pose;
              if(findCenterOfCurve(pose_A, pose_B, pose_C))
              {
                double xCA = pose_A.pose.position.x - pose_C.pose.position.x;
                double yCA = pose_A.pose.position.y - pose_C.pose.position.y;
                double xCB = pose_B.pose.position.x - pose_C.pose.position.x;
                double yCB = pose_B.pose.position.y - pose_C.pose.position.y;
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
                double xA1 = pose_C.pose.position.x + rCA * cos(check_angle);
                double yA1 = pose_C.pose.position.y + rCA * sin(check_angle);
                double xCA1 = xA1 - pose_C.pose.position.x;
                double yCA1 = yA1 - pose_C.pose.position.y;
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
                plan1.clear();
                plan2.clear();
                if (is_increase_angle)
                {
                  for (double i = 0; i <= 1; i += angle_interval)
                  {
                    double angle_tmp = angleCA + angleACB * i;
                    double xP = pose_C.pose.position.x + rCA * cos(angle_tmp);
                    double yP = pose_C.pose.position.y + rCA * sin(angle_tmp);
                    geometry_msgs::PoseStamped p;
                    p.pose.position.x = xP;
                    p.pose.position.y = yP;
                    p.pose.position.z = 0;
                    plan1.push_back(p);
                  }
                }
                else
                {
                  for (double i = 0; i <= 1; i += angle_interval)
                  {
                    double angle_tmp = angleCA - angleACB * i;
                    double xP = pose_C.pose.position.x + rCA * cos(angle_tmp);
                    double yP = pose_C.pose.position.y + rCA * sin(angle_tmp);
                    geometry_msgs::PoseStamped p;
                    p.pose.position.x = xP;
                    p.pose.position.y = yP;
                    p.pose.position.z = 0;
                    plan1.push_back(p);
                  }
                }
                if (!plan1.empty() && plan1.size() > 2)
                {
                  if (computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                                                  plan1.back().pose, plan1[plan1.size() - 2].pose) <= 1.3962634016) // <= 80 degree
                  {
                    for (int i = 0; i < ((int)plan1.size() - 1); i++)
                    {
                      double theta = calculateAngle(plan1[i].pose.position.x, plan1[i].pose.position.y,
                                                    plan1[i + 1].pose.position.x, plan1[i + 1].pose.position.y);
                      plan1[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
                    }
                    plan1.back().pose.orientation = pose_B.pose.orientation;
                  }
                  else if(computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                                            plan1.back().pose, plan1[plan1.size() - 2].pose) >= 1.745329252) // >= 100 degree
                  {
                    for (int i = (int)plan1.size() - 1; i > 0; i--)
                    {
                      double theta = calculateAngle(plan1[i].pose.position.x, plan1[i].pose.position.y,
                                                    plan1[i - 1].pose.position.x, plan1[i - 1].pose.position.y);
                      plan1[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
                    }
                    plan1.front().pose.orientation = plan1[1].pose.orientation;
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
                plan2 = divideSegment(pose_B, goal_pose, 0.02);
                result_plan.assign(plan1.begin(), plan1.end());
                result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
                if(!result_plan.empty())
                return true;
                else
                {
                  ROS_ERROR("[makePlanPickupShelf] failed to make plan TH7");           
                  return false;
                }
              }
              else
              {
                plan1 = divideSegment(current_pose, pose_B, 0.02);
                plan2 = divideSegment(pose_B, goal_pose, 0.02);
                result_plan.assign(plan1.begin(), plan1.end());
                result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
                if(!result_plan.empty())
                return true;
                else
                {
                  ROS_ERROR("[makePlanPickupShelf] failed to make plan TH8");           
                  return false;
                }
              }
            }
          }
        }
      }
    }
    else // robot move backward
    {
      double shelf_pose_yaw = getYaw(shelf_pose_on_map.pose.orientation.x,
                                      shelf_pose_on_map.pose.orientation.y,
                                      shelf_pose_on_map.pose.orientation.z,
                                      shelf_pose_on_map.pose.orientation.w);
      modifyYaw(shelf_pose_yaw);
      double goal_pose_yaw = shelf_pose_yaw;
      modifyYaw(goal_pose_yaw);
      geometry_msgs::PoseStamped goal_pose;
      goal_pose.pose.position = shelf_pose_on_map.pose.position;
      goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal_pose_yaw);
      geometry_msgs::PoseStamped pose_offset_min;                                            
      pose_offset_min.pose.position.x = shelf_pose_on_map.pose.position.x + d_offset_min*cos(shelf_pose_yaw);
      pose_offset_min.pose.position.y = shelf_pose_on_map.pose.position.y + d_offset_min*sin(shelf_pose_yaw);
      pose_offset_min.pose.orientation = goal_pose.pose.orientation;
      geometry_msgs::PoseStamped pose_intersection = findPerpendicularIntersection(current_pose, shelf_pose_on_map, pose_offset_min);
      double d_shelfpose_to_intersection = std::sqrt(std::pow(pose_intersection.pose.position.x - shelf_pose_on_map.pose.position.x, 2) + 
        std::pow(pose_intersection.pose.position.y - shelf_pose_on_map.pose.position.y, 2));
      double delta_d1 = d_shelfpose_to_intersection - d_offset_min;
      if(delta_d1 <= 0.1)
      {
        plan1.clear();
        plan1 = divideSegment(pose_offset_min, goal_pose, 0.02);
        result_plan = plan1;
        if(!result_plan.empty())
        return true;
        else
        {
          ROS_ERROR("[makePlanPickupShelf] failed to make plan TH9");
          return false;
        }
      }
      else
      {
        geometry_msgs::PoseStamped pose_B;
        pose_B.pose.position.x = pose_intersection.pose.position.x + d_intersection*cos(goal_pose_yaw);
        pose_B.pose.position.y = pose_intersection.pose.position.y + d_intersection*sin(goal_pose_yaw);
        pose_B.pose.orientation = goal_pose.pose.orientation;
        if(d_intersection <= 0.1)
        {
          double pose_intersection_yaw = calculateAngle(pose_intersection.pose.position.x, pose_intersection.pose.position.y,
            current_pose.pose.position.x, current_pose.pose.position.y);
          pose_intersection.pose.orientation = tf::createQuaternionMsgFromYaw(pose_intersection_yaw);
          plan1.clear();
          plan2.clear();
          plan1 = divideSegment(current_pose, pose_intersection, 0.02);   
          pose_intersection.pose.orientation = tf::createQuaternionMsgFromYaw(goal_pose_yaw);
          plan2 = divideSegment(pose_intersection, goal_pose, 0.02);     
          result_plan.assign(plan1.begin(), plan1.end());
          result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());   
          if(!result_plan.empty())
          return true;
          else
          {
            ROS_ERROR("[makePlanPickupShelf] failed to make plan TH10");            
            return false;
          }
        }
        else
        {
          double d_shelfpose_to_intersection = std::sqrt(std::pow(pose_B.pose.position.x - shelf_pose_on_map.pose.position.x, 2) + 
            std::pow(pose_B.pose.position.y - shelf_pose_on_map.pose.position.y, 2));
          double delta2 = d_shelfpose_to_intersection - d_offset_min;
          if(delta2 > 0.1 &&
            computeDeltaAngleStartOfPlan(shelf_pose_yaw, shelf_pose_on_map.pose, pose_B.pose) <= 0.5235987756) // <= 30 degree
          {
            // nếu hướng của vector AB và hướng của pose_B tạo với nhau một góc ~0 độ hoặc ~180 độ -> cung tròn AB sẽ gần như là một đọan thẳng
            if((computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) >= 3.13 && 
                computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) <= M_PI) ||
                (computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) <= 0.1745 && 
                computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) >= 0))
            {
              plan1.clear();
              plan2.clear();
              plan1 = divideSegment(current_pose, pose_B, 0.02);
              plan2 = divideSegment(pose_B, goal_pose, 0.02);
              result_plan.assign(plan1.begin(), plan1.end());
              result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
              if(!result_plan.empty())
              return true;
              else
              {
                ROS_ERROR("[makePlanPickupShelf] failed to make plan TH11");           
                return false;
              }
            }
            else
            {
              // Tính toán đoạn đường cong AB
              geometry_msgs::PoseStamped pose_C;
              geometry_msgs::PoseStamped pose_A = current_pose;
              if(findCenterOfCurve(pose_A, pose_B, pose_C))
              {
                double xCA = pose_A.pose.position.x - pose_C.pose.position.x;
                double yCA = pose_A.pose.position.y - pose_C.pose.position.y;
                double xCB = pose_B.pose.position.x - pose_C.pose.position.x;
                double yCB = pose_B.pose.position.y - pose_C.pose.position.y;
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
                double xA1 = pose_C.pose.position.x + rCA * cos(check_angle);
                double yA1 = pose_C.pose.position.y + rCA * sin(check_angle);
                double xCA1 = xA1 - pose_C.pose.position.x;
                double yCA1 = yA1 - pose_C.pose.position.y;
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
                plan1.clear();
                plan2.clear();
                if (is_increase_angle)
                {
                  for (double i = 0; i <= 1; i += angle_interval)
                  {
                    double angle_tmp = angleCA + angleACB * i;
                    double xP = pose_C.pose.position.x + rCA * cos(angle_tmp);
                    double yP = pose_C.pose.position.y + rCA * sin(angle_tmp);
                    geometry_msgs::PoseStamped p;
                    p.pose.position.x = xP;
                    p.pose.position.y = yP;
                    p.pose.position.z = 0;
                    plan1.push_back(p);
                  }
                }
                else
                {
                  for (double i = 0; i <= 1; i += angle_interval)
                  {
                    double angle_tmp = angleCA - angleACB * i;
                    double xP = pose_C.pose.position.x + rCA * cos(angle_tmp);
                    double yP = pose_C.pose.position.y + rCA * sin(angle_tmp);
                    geometry_msgs::PoseStamped p;
                    p.pose.position.x = xP;
                    p.pose.position.y = yP;
                    p.pose.position.z = 0;
                    plan1.push_back(p);
                  }
                }
                if (!plan1.empty() && plan1.size() > 2)
                {
                  if (computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                                                  plan1.back().pose, plan1[plan1.size() - 2].pose) <= 1.3962634016) // <= 80 degree
                  {
                    for (int i = 0; i < ((int)plan1.size() - 1); i++)
                    {
                      double theta = calculateAngle(plan1[i].pose.position.x, plan1[i].pose.position.y,
                                                    plan1[i + 1].pose.position.x, plan1[i + 1].pose.position.y);
                      plan1[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
                    }
                    plan1.back().pose.orientation = pose_B.pose.orientation;
                  }
                  else if(computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                                            plan1.back().pose, plan1[plan1.size() - 2].pose) >= 1.745329252) // >= 100 degree
                  {
                    for (int i = (int)plan1.size() - 1; i > 0; i--)
                    {
                      double theta = calculateAngle(plan1[i].pose.position.x, plan1[i].pose.position.y,
                                                    plan1[i - 1].pose.position.x, plan1[i - 1].pose.position.y);
                      plan1[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
                    }
                    plan1.front().pose.orientation = plan1[1].pose.orientation;
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
                plan2 = divideSegment(pose_B, goal_pose, 0.02);
                result_plan.assign(plan1.begin(), plan1.end());
                result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
                if(!result_plan.empty())
                return true;
                else
                {
                  ROS_ERROR("[makePlanPickupShelf] failed to make plan TH12");           
                  return false;
                }
              }
              else
              {
                plan1 = divideSegment(current_pose, pose_B, 0.02);
                plan2 = divideSegment(pose_B, goal_pose, 0.02);
                result_plan.assign(plan1.begin(), plan1.end());
                result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
                if(!result_plan.empty())
                return true;
                else
                {
                  ROS_ERROR("[makePlanPickupShelf] failed to make plan TH13");           
                  return false;
                }
              }
            }
          }
          else
          {
            pose_B = pose_offset_min;
            pose_B.pose.orientation = goal_pose.pose.orientation;
            // nếu hướng của vector AB và hướng của pose_B tạo với nhau một góc ~0 độ hoặc ~180 độ -> cung tròn AB sẽ gần như là một đọan thẳng
            if((computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) >= 3.13 && 
                computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) <= M_PI) ||
                (computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) <= 0.1745 && 
                computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                pose_B.pose, current_pose.pose) >= 0))
            {
              plan1 = divideSegment(current_pose, pose_B, 0.02);
              plan2 = divideSegment(pose_B, goal_pose, 0.02);
              result_plan.assign(plan1.begin(), plan1.end());
              result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
              if(!result_plan.empty())
              return true;
              else
              {
                ROS_ERROR("[makePlanPickupShelf] failed to make plan TH14");           
                return false;
              }
            }
            else
            {
              // Tính toán đoạn đường cong AB
              geometry_msgs::PoseStamped pose_C;
              geometry_msgs::PoseStamped pose_A = current_pose;
              if(findCenterOfCurve(pose_A, pose_B, pose_C))
              {
                double xCA = pose_A.pose.position.x - pose_C.pose.position.x;
                double yCA = pose_A.pose.position.y - pose_C.pose.position.y;
                double xCB = pose_B.pose.position.x - pose_C.pose.position.x;
                double yCB = pose_B.pose.position.y - pose_C.pose.position.y;
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
                double xA1 = pose_C.pose.position.x + rCA * cos(check_angle);
                double yA1 = pose_C.pose.position.y + rCA * sin(check_angle);
                double xCA1 = xA1 - pose_C.pose.position.x;
                double yCA1 = yA1 - pose_C.pose.position.y;
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
                plan1.clear();
                plan2.clear();
                if (is_increase_angle)
                {
                  for (double i = 0; i <= 1; i += angle_interval)
                  {
                    double angle_tmp = angleCA + angleACB * i;
                    double xP = pose_C.pose.position.x + rCA * cos(angle_tmp);
                    double yP = pose_C.pose.position.y + rCA * sin(angle_tmp);
                    geometry_msgs::PoseStamped p;
                    p.pose.position.x = xP;
                    p.pose.position.y = yP;
                    p.pose.position.z = 0;
                    plan1.push_back(p);
                  }
                }
                else
                {
                  for (double i = 0; i <= 1; i += angle_interval)
                  {
                    double angle_tmp = angleCA - angleACB * i;
                    double xP = pose_C.pose.position.x + rCA * cos(angle_tmp);
                    double yP = pose_C.pose.position.y + rCA * sin(angle_tmp);
                    geometry_msgs::PoseStamped p;
                    p.pose.position.x = xP;
                    p.pose.position.y = yP;
                    p.pose.position.z = 0;
                    plan1.push_back(p);
                  }
                }
                if (!plan1.empty() && plan1.size() > 2)
                {
                  if (computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                                                  plan1.back().pose, plan1[plan1.size() - 2].pose) <= 1.3962634016) // <= 80 degree
                  {
                    for (int i = 0; i < ((int)plan1.size() - 1); i++)
                    {
                      double theta = calculateAngle(plan1[i].pose.position.x, plan1[i].pose.position.y,
                                                    plan1[i + 1].pose.position.x, plan1[i + 1].pose.position.y);
                      plan1[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
                    }
                    plan1.back().pose.orientation = pose_B.pose.orientation;
                  }
                  else if(computeDeltaAngleEndOfPlan(getYaw(pose_B.pose.orientation.x, pose_B.pose.orientation.y, pose_B.pose.orientation.z, pose_B.pose.orientation.w),
                                            plan1.back().pose, plan1[plan1.size() - 2].pose) >= 1.745329252) // >= 100 degree
                  {
                    for (int i = (int)plan1.size() - 1; i > 0; i--)
                    {
                      double theta = calculateAngle(plan1[i].pose.position.x, plan1[i].pose.position.y,
                                                    plan1[i - 1].pose.position.x, plan1[i - 1].pose.position.y);
                      plan1[i].pose.orientation = tf::createQuaternionMsgFromYaw(theta);
                    }
                    plan1.front().pose.orientation = plan1[1].pose.orientation;
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
                plan2 = divideSegment(pose_B, goal_pose, 0.02);
                result_plan.assign(plan1.begin(), plan1.end());
                result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
                if(!result_plan.empty())
                return true;
                else
                {
                  ROS_ERROR("[makePlanPickupShelf] failed to make plan TH15");           
                  return false;
                }
              }
              else
              {
                plan1 = divideSegment(current_pose, pose_B, 0.02);
                plan2 = divideSegment(pose_B, goal_pose, 0.02);
                result_plan.assign(plan1.begin(), plan1.end());
                result_plan.insert(result_plan.end(), plan2.begin(), plan2.end());
                if(!result_plan.empty())
                return true;
                else
                {
                  ROS_ERROR("[makePlanPickupShelf] failed to make plan TH16");           
                  return false;
                }
              }
            }
          }
        }
      }

    }
    return result;
}
