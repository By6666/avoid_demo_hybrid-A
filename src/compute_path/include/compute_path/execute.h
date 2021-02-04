#ifndef EXECUTE_H
#define EXECUTE_H

#include "compute_path/compute_path.h"
#include "compute_path/obstacle_info.h"
#include "compute_path/truck_info.h"

inline void ImportInfo(const ObsInfo& obs_info, const TruckInfo& truck_info,
                       HybridAstar& hybrid_astar) {
  hybrid_astar.set_obs_list() = obs_info.get_obs_list();
  hybrid_astar.set_map_data() = obs_info.get_map_cost();
  hybrid_astar.set_map_heigh() = obs_info.get_map_heigh();
  hybrid_astar.set_map_width() = obs_info.get_map_width();
  hybrid_astar.set_hollow_list() = obs_info.get_hollow_info();
  hybrid_astar.set_map_resolution() = obs_info.get_map_resolution();
  hybrid_astar.set_map_origin() = obs_info.get_map_origin().position;
  hybrid_astar.set_ref_line() = obs_info.get_ref_line();

  hybrid_astar.set_goal_pose() = truck_info.get_goal_pose();
  hybrid_astar.set_start_pose() = truck_info.get_start_pose();
  hybrid_astar.set_truck_width() = truck_info.get_truck_width();
  hybrid_astar.set_truck_length() = truck_info.get_truck_length();
  hybrid_astar.set_truck_base2back() = truck_info.get_truck_base2back();
}

#endif
