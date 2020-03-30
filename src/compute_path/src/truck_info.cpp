#include "compute_path/truck_info.h"

TruckInfo::TruckInfo()
    : start_ok_(false), goal_ok_(false), private_nh_("~TruckInfo") {
  // ros::NodeHandle private_nh("~TruckInfo");
  private_nh_.param<double>("truck_vel", truck_vel_, 2.0);
  private_nh_.param<double>("truck_width", truck_width_, 8.0);
  private_nh_.param<double>("truck_length", truck_length_, 12.0);
  private_nh_.param<double>("truck_base2back", truck_base2back_, 3.0);
  private_nh_.param<double>("move_step", move_step_, 2.4);

  private_nh_.param<double>("goal_yaw", goal_yaw_, 0.0);
  private_nh_.param<double>("start_yaw", start_yaw_, 0.0);
  private_nh_.param<double>("dist_limit_coff", dist_limit_coff_, 10.0);

  private_nh_.param<double>("goal_point_x", goal_pose_.pose.position.x, 50.0);
  private_nh_.param<double>("goal_point_y", goal_pose_.pose.position.y, 0.0);
  private_nh_.param<double>("start_point_x", start_pose_.pose.position.x, 0.0);
  private_nh_.param<double>("start_point_y", start_pose_.pose.position.y, 0.0);

  // 获得障碍物侦测距离
  int dist_limit_temp = static_cast<int>(dist_limit_coff_ * truck_vel_);
  dist_limit_ = (dist_limit_temp > 20) ? dist_limit_temp : 20;

  start_pose_.header.frame_id = goal_pose_.header.frame_id = "global";
  start_pose_.header.stamp = goal_pose_.header.stamp = ros::Time::now();
  start_pose_.pose.position.z = goal_pose_.pose.position.z = 0.0;

  goal_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw_);
  start_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(start_yaw_);
  start_ok_ = goal_ok_ = true;
}

void TruckInfo::TruckShow(const std::vector<geometry_msgs::Pose>& path,
                          const ros::Publisher& pub) {
  static int last_id_num = 0;
  visualization_msgs::MarkerArray show_array;

  // show truck central
  visualization_msgs::Marker truck_central;
  truck_central.header.frame_id = "global";

  truck_central.ns = "truck_central";
  truck_central.id = 0;

  truck_central.type = 2;
  truck_central.action = visualization_msgs::Marker::ADD;

  truck_central.pose = start_pose_.pose;

  truck_central.scale.x = 2.0;
  truck_central.scale.y = 2.0;
  truck_central.scale.z = 0.01;

  truck_central.color.r = 1.0f;
  truck_central.color.g = 0.98f;
  truck_central.color.b = 0.8f;
  truck_central.color.a = 1.0f;
  show_array.markers.push_back(truck_central);

  // show goal pose
  visualization_msgs::Marker goal_point;
  goal_point.header.frame_id = "global";

  goal_point.ns = "goal_point";
  goal_point.id = 1;

  goal_point.type = 2;
  goal_point.action = visualization_msgs::Marker::ADD;

  goal_point.pose = goal_pose_.pose;

  goal_point.scale.x = move_step_;
  goal_point.scale.y = move_step_;
  goal_point.scale.z = 0.01;

  goal_point.color.r = 1.0f;
  goal_point.color.g = 0.0f;
  goal_point.color.b = 0.0f;
  goal_point.color.a = 1.0f;
  show_array.markers.push_back(goal_point);

  // show detective dis
  int discretize = 360;
  double unity = 2.0 * M_PI / discretize;
  visualization_msgs::Marker avoid_limit;
  avoid_limit.header.frame_id = "global";

  avoid_limit.ns = "avoid_limit";
  avoid_limit.id = 2;

  avoid_limit.action = visualization_msgs::Marker::ADD;
  avoid_limit.type = 7;

  avoid_limit.scale.x = 0.2;
  avoid_limit.scale.y = 0.2;
  avoid_limit.scale.z = 0.01;

  avoid_limit.color.r = 0.48f;
  avoid_limit.color.g = 0.55f;
  avoid_limit.color.b = 0.55f;
  avoid_limit.color.a = 1.0;

  geometry_msgs::Point temp;
  temp.z = start_pose_.pose.position.z;
  for (int i = 0; i < discretize; ++i) {
    temp.x = dist_limit_ * cosf(i * unity) + start_pose_.pose.position.x;
    temp.y = dist_limit_ * sinf(i * unity) + start_pose_.pose.position.y;

    avoid_limit.points.emplace_back(temp);
  }
  show_array.markers.push_back(avoid_limit);

  // show truck frame
  visualization_msgs::Marker truck_box;
  truck_box.header.frame_id = "global";

  truck_box.ns = "truck_box";
  int id_init = 3;

  truck_box.type = 4;
  truck_box.action = visualization_msgs::Marker::ADD;

  truck_box.scale.x = 0.2;
  truck_box.scale.y = 0.2;

  truck_box.color.r = 0.18f;
  truck_box.color.g = 0.56f;
  truck_box.color.b = 1.0f;
  truck_box.color.a = 1.0f;

  truck_box.id = id_init++;
  truck_box.points = TruckFrame(start_pose_.pose);
  show_array.markers.push_back(truck_box);

  for (auto& elem : path) {
    truck_box.id = id_init++;
    truck_box.points = TruckFrame(elem);
    show_array.markers.push_back(truck_box);
  }

  int size = show_array.markers.size();
  if (show_array.markers.size() < last_id_num) {
    for (int i = size; i < last_id_num; ++i) {
      truck_box.id = i;
      truck_box.action = visualization_msgs::Marker::DELETE;
      show_array.markers.push_back(truck_box);
    }
  }
  last_id_num = size;

  // push all
  pub.publish(show_array);
}

// 获得truck frame
std::vector<geometry_msgs::Point> TruckInfo::TruckFrame(
    const geometry_msgs::Pose& central) {
  geometry_msgs::Point temp;
  std::vector<geometry_msgs::Point> truck_frame;
  truck_frame.reserve(5);

  temp.x = 0.0 - truck_base2back_;
  temp.y = truck_width_ / 2;
  // temp.y = 7.4 / 2;
  temp.z = central.position.z;
  truck_frame.emplace_back(geometry_msgs::Point(temp));

  temp.x = 0.0 - truck_base2back_;
  temp.y = 0.0 - truck_width_ / 2;
  // temp.y = 0.0 - 7.4 / 2;
  truck_frame.emplace_back(geometry_msgs::Point(temp));

  temp.x = truck_length_ - truck_base2back_;
  temp.y = 0.0 - truck_width_ / 2;
  // temp.y = 0.0 - 7.4 / 2;
  truck_frame.emplace_back(geometry_msgs::Point(temp));

  temp.x = truck_length_ - truck_base2back_;
  temp.y = truck_width_ / 2;
  // temp.y = 7.4 / 2;
  truck_frame.emplace_back(geometry_msgs::Point(temp));

  double yaw = tf::getYaw(central.orientation);

  for (auto& elem : truck_frame) {
    double x = elem.x, y = elem.y;
    elem.x = cosf(yaw) * x - sinf(yaw) * y + central.position.x;
    elem.y = cosf(yaw) * y + sinf(yaw) * x + central.position.y;
  }

  truck_frame.push_back(truck_frame.front());

  return truck_frame;
}

void TruckInfo::UpgrateParam() {
  // ros::NodeHandle private_nh("~TruckInfo");
  private_nh_.param<double>("truck_vel", truck_vel_, 2.0);
  private_nh_.param<double>("truck_width", truck_width_, 8.0);
  private_nh_.param<double>("truck_length", truck_length_, 12.0);
  private_nh_.param<double>("truck_base2back", truck_base2back_, 3.0);

  private_nh_.param<double>("dist_limit_coff", dist_limit_coff_, 10.0);

  // 获得障碍物侦测距离
  int dist_limit_temp = static_cast<int>(dist_limit_coff_ * truck_vel_);
  dist_limit_ = (dist_limit_temp > 20) ? dist_limit_temp : 20;
}

void TruckInfo::StartInfoCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& start_initial) {
  start_ok_ = false;
  start_pose_.header = start_initial->header;
  start_pose_.pose = start_initial->pose.pose;
  start_ok_ = true;
}

void TruckInfo::GoalInfoCallback(
    const geometry_msgs::PoseStampedConstPtr& goal_initial) {
  goal_ok_ = false;
  goal_pose_ = *goal_initial;
  goal_ok_ = true;
}
