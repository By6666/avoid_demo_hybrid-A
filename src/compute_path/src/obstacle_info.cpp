#include "compute_path/obstacle_info.h"

ObsInfo::ObsInfo() : obs_list_ok_(false), hollow_ok_(false) {
  ros::NodeHandle private_nh("~ObsInfo");

  private_nh.param<int>("obstacle_threshold", obstacle_threshold_, 80);
}

// void ObsInfo::GridMapCall(const nav_msgs::OccupancyGridConstPtr& msg) {
//   obs_list_ok_ = false;
//   map = *msg;
//   MapHandel(map.data);
// }

void ObsInfo::GridMapCall(const grid_map::MapInfoConstPtr& msg) {
  obs_list_ok_ = false;
  map = msg->occ_map;
  ref_line = msg->reference_line;
  MapHandel(map.data);
}

void ObsInfo::MapHandel(const std::vector<int8_t>& map_data) {
  obs_list_.clear();
  for (int i = 0; i < map_data.size(); ++i) {
    if (map_data[i] >= obstacle_threshold_) obs_list_.insert(i);
  }
  obs_list_ok_ = hollow_ok_ | 1;
}

void ObsInfo::HollowInfoCall(
    const jsk_recognition_msgs::BoundingBoxArrayConstPtr& msg) {
  hollow_info_ = *msg;
  hollow_ok_ = true;
}
