#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"

#include <iostream>
#include <vector>

#include "grid_map/grid_map.h"
#include "grid_map/MapInfo.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "grid_map");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<grid_map::MapInfo>("map_info", 1, true);
  ros::Publisher pub_map = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  ros::Publisher pub_ref_line = nh.advertise<nav_msgs::Path>("map_ref_line", 1, true);

  ros::Rate loop_rate(10);

  GridMap map;

  bool read_map_flag = map.ReadMap();

  if (!read_map_flag) return 0;

  grid_map::MapInfo map_info;
  // nav_msgs::OccupancyGrid occ_map;
  nav_msgs::Path ref_line;

  map_info.occ_map.header.stamp = ros::Time::now();
  map_info.occ_map.header.frame_id = "global";

  ref_line.header.stamp = ros::Time::now();
  ref_line.header.frame_id = "global";

  while (ros::ok()) {
    map.UpgrateParam();

    bool read_map_flag = map.ReadMap();
    if (!read_map_flag) continue;

    map_info.occ_map.info.height = map.get_raw();
    map_info.occ_map.info.width = map.get_col();
    map_info.occ_map.info.origin.position = map.get_map_origin();
    map_info.occ_map.info.resolution = map.get_map_resolution();
    map_info.occ_map.data = map.get_map_data();

    map_info.reference_line = map.get_ref_line();

    for (const auto& elem : map_info.reference_line) {
      geometry_msgs::PoseStamped temp;
      temp.pose.position.x = elem.x;
      temp.pose.position.y = elem.y;
      ref_line.poses.push_back(temp);
    }

    pub.publish(map_info);
    pub_map.publish(map_info.occ_map);
    pub_ref_line.publish(ref_line);
    loop_rate.sleep();
  }

  return 0;
}
