#include <iostream>

#include "compute_path/execute.h"
#include "matplotlibcpp.h"
#include "ros/ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "hybrid_A");
  ros::NodeHandle nh;

  ObsInfo obs_info;
  TruckInfo truck_info;
  HybridAstar hybrid_astar;

  // subscribe map topic, get grid map info
  ros::Subscriber map_sub =
      nh.subscribe("/map_info", 1, &ObsInfo::GridMapCall, &obs_info);

  // subscribe hollow topic, get hollow info
  ros::Subscriber hollow_sub = nh.subscribe(
      "/hollow_create/hollow_info", 1, &ObsInfo::HollowInfoCall, &obs_info);

  // subscribe start pose
  ros::Subscriber start_sub = nh.subscribe(
      "/initialpose", 1, &TruckInfo::StartInfoCallback, &truck_info);
  // subscribe goal pose
  ros::Subscriber goal_sub = nh.subscribe(
      "/move_base_simple/goal", 1, &TruckInfo::GoalInfoCallback, &truck_info);

  // pub truck show info
  ros::Publisher truck_show_pub =
      nh.advertise<visualization_msgs::MarkerArray>("truck_show", 1, true);

  ros::Publisher update_show_pub =
      nh.advertise<visualization_msgs::MarkerArray>("update_pose_show", 1, true);

  double time_sum = 0.0;
  int cnt = 0, test_times_cnt;
  bool test_model_flg, curvature_show_flg;
  nh.param<bool>("curvature_show_flg", curvature_show_flg, false);
  nh.param<bool>("test_model_flg", test_model_flg, false);
  nh.param<int>("test_times", test_times_cnt, 20);

  while (ros::ok()) {
    // get all info (1.grid map info, 2.hollow info)
    ros::spinOnce();

    // param update
    truck_info.UpgrateParam();
    hybrid_astar.UpgrateParam();
    if (!(obs_info.get_obs_state() && truck_info.get_start_goal_state())) {
      std::cout << obs_info.get_obs_state() << "  "
                << truck_info.get_start_goal_state() << std::endl;
      ros::Duration(0.01).sleep();
      continue;
    }

    // input start, goal, cost map , obs info and hollow info to hybrid_astar
    ImportInfo(obs_info, truck_info, hybrid_astar);

    // hybrid Astar
    std::cout << "********** Hybrid Astar start !! **********" << std::endl;
    // hybrid_astar.PrintMapSize();
    ros::WallTime start = ros::WallTime::now();
    bool flag = hybrid_astar.ExecuteHybridAstar();
    ros::WallTime end = ros::WallTime::now();
    if (flag) {
      --test_times_cnt;
      std::cout << "hybrid_astar execute successful !!" << std::endl;
    }

    if (test_model_flg && !test_times_cnt) hybrid_astar.PrintPath();
    std::cout << "*********** Hybrid Astar End !! ***********" << std::endl
              << "path_score: " << hybrid_astar.get_path_evaluate_value()
              << "  average_curvature_diff: "
              << hybrid_astar.get_curvature_average_diff()
              << "  max_curvature_diff: "
              << hybrid_astar.get_max_curvature_diff() << std::endl
              << "path_length: " << hybrid_astar.get_path_length()
              << "  extension_nums: " << hybrid_astar.get_extension_pos_nums()
              << "  spend time: " << (end - start).toSec() * 1000 << " ms"
              << std::endl;
    // cost time sum
    time_sum += (end - start).toSec() * 1000;
    std::cout << "average spend time : " << time_sum / (++cnt) << " ms"
              << std::endl
              << std::endl
              << std::endl;

    // show in rviz
    truck_info.TruckShow(hybrid_astar.get_final_path(), truck_show_pub);
    hybrid_astar.UpdatePoseShow(update_show_pub);

    if (test_model_flg && !test_times_cnt) {
      // while (ros::ok()) {
      truck_info.TruckShow(hybrid_astar.get_final_path(), truck_show_pub);
      ros::Duration(0.1).sleep();
      hybrid_astar.UpdatePoseShow(update_show_pub);
      ros::Duration(0.1).sleep();
      // }
      hybrid_astar.PrintMapSize();
      std::cout << std::endl << std::endl;
      break;
    }

    ros::Duration(0.1).sleep();
  }
  
  if (curvature_show_flg) {
    ros::shutdown();
    matplotlibcpp::plot(hybrid_astar.get_curvature_data());
    matplotlibcpp::grid(true);
    matplotlibcpp::show();
  }

  return 0;
}
