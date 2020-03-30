#ifndef TRUCK_INFO_H
#define TRUCK_INFO_H

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <iostream>
#include <vector>

class TruckInfo {
 public:
  TruckInfo();

  void UpgrateParam();
  void TruckShow(const std::vector<geometry_msgs::Pose>& path,
                 const ros::Publisher& pub);

  void StartInfoCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_initial);
  void GoalInfoCallback(const geometry_msgs::PoseStampedConstPtr& goal_initial);

  inline bool get_start_goal_state() const { return start_ok_ && goal_ok_; }
  inline geometry_msgs::Pose get_goal_pose() const { return goal_pose_.pose; }
  inline geometry_msgs::Pose get_start_pose() const { return start_pose_.pose; }

  inline double get_truck_width() const { return truck_width_; }
  inline double get_truck_length() const { return truck_length_; }
  inline double get_truck_base2back() const { return truck_base2back_; }

 private:
  ros::NodeHandle private_nh_;

  int dist_limit_;
  bool start_ok_, goal_ok_;
  double start_yaw_, goal_yaw_;
  double dist_limit_coff_, truck_vel_, move_step_;
  geometry_msgs::PoseStamped start_pose_, goal_pose_;
  double truck_length_, truck_width_, truck_base2back_;

  std::vector<geometry_msgs::Point> TruckFrame(
      const geometry_msgs::Pose& central);
};

#endif