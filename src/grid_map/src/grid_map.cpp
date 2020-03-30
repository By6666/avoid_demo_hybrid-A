#include "grid_map/grid_map.h"

GridMap::GridMap() {
  ros::NodeHandle private_nh("~");

  private_nh.param<int>("file_num", file_num_, 1);
  private_nh.param<std::string>("file_path", file_path_, "");

  private_nh.param<double>("map_origin_x", origin_point.x, -10.0);
  private_nh.param<double>("map_origin_y", origin_point.y, -10.0);
  private_nh.param<double>("map_origin_z", origin_point.z, -0.0);

  private_nh.param<float>("map_resolutin", map_resolution_, 1.0f);
};

bool GridMap::ReadMap() {
  // file_name_ = file_path_ + std::to_string(file_num_) + ".txt";

  // std::ifstream read_grid;
  // read_grid.open(file_name_, std::ios_base::in);
  // if (!read_grid.is_open()) {
  //   std::cout << "** file open fail !! **" << std::endl;
  //   return false;
  // }

  // std::string grid_buff = "";
  // for (int16_t i = 0; i < raw_; ++i) {
  //   getline(read_grid, grid_buff);
  //   for (int16_t j = 0; j < grid_buff.length(); ++j) {
  //     if (grid_buff[j] == 'x')
  //       map_data.push_back(85);
  //     else if (grid_buff[j] != ' ')
  //       map_data.push_back(0);
  //   }
  // }
  // read_grid.close();

  map_data.clear();
  col_ = raw_ = 0;
  file_name_ = file_path_ + std::to_string(file_num_) + ".txt";

  std::ifstream read_grid;
  read_grid.open(file_name_, std::ios_base::in);
  if (!read_grid.is_open()) {
    std::cout << "** file open fail !! **" << std::endl;
    return false;
  }

  std::string grid_buff = "";
  while (true) {
    getline(read_grid, grid_buff);
    if (grid_buff.size() == 0) break;
    for (int16_t j = 0; j < grid_buff.length(); ++j) {
      if (grid_buff[j] == 'x')
        map_data.push_back(85);
      else if (grid_buff[j] != ' ')
        map_data.push_back(0);
    }
    ++raw_;
  }
  col_ = map_data.size() / raw_;
  read_grid.close();

  return true;
}

void GridMap::UpgrateParam() {
  ros::NodeHandle private_nh("~");

  private_nh.param<int>("file_num", file_num_, 1);
  private_nh.param<std::string>("file_path", file_path_, "");

  private_nh.param<double>("map_origin_x", origin_point.x, -10.0);
  private_nh.param<double>("map_origin_y", origin_point.y, -10.0);
  private_nh.param<double>("map_origin_z", origin_point.z, -0.0);

  private_nh.param<float>("map_resolutin", map_resolution_, 1.0f);

  // std::vector<int8_t> empty;
  // std::swap(map_data, empty);
  // map_data.reserve(col_ * raw_);
};