#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

#include <iostream>

#include "grid_map/grid_map.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "grid_map");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  ros::Rate loop_rate(10);

  GridMap map;

  bool read_map_flag = map.ReadMap();

  if (!read_map_flag) return 0;

  nav_msgs::OccupancyGrid occ_map;

  occ_map.header.stamp = ros::Time::now();
  occ_map.header.frame_id = "global";

  while (ros::ok()) {
    map.UpgrateParam();

    bool read_map_flag = map.ReadMap();
    if (!read_map_flag) continue;

    occ_map.info.height = map.get_raw();
    occ_map.info.width = map.get_col();
    occ_map.info.origin.position = map.get_map_origin();
    occ_map.info.resolution = map.get_map_resolution();
    occ_map.data = map.get_map_data();

    pub.publish(occ_map);
    loop_rate.sleep();
  }

  return 0;
}