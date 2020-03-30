#ifndef GRID_MAP_H
#define GRID_MAP_H

#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "stdint.h"

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class GridMap {
 public:
  GridMap();

  bool ReadMap();
  void UpgrateParam();

  inline int get_raw() const { return raw_; }
  inline int get_col() const { return col_; }
  inline std::string get_file_pwd() const { return file_name_; }
  inline float get_map_resolution() const { return map_resolution_; }
  inline std::vector<int8_t> get_map_data() const { return map_data; }
  inline geometry_msgs::Point get_map_origin() const { return origin_point; }

 private:
  int file_num_;
  int raw_, col_;
  float map_resolution_;
  std::string file_name_;
  std::string file_path_;
  std::vector<int8_t> map_data;
  geometry_msgs::Point origin_point;

  inline int16_t CalculateID(int16_t raw, int16_t col) {
    return raw * col_ + col;
  }
};

#endif