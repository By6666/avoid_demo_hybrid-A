#include "compute_path/compute_path.h"

HybridAstar::HybridAstar() {
  ros::NodeHandle private_nh("~compute_path");

  //** 通过gridmap进行统一,不再通过其launch文件获得
  // private_nh.param<int>("width", width_, 70);
  // private_nh.param<int>("heigh", heigh_, 20);

  private_nh.param<double>("node2goal_r", node2goal_r_, 1.0);
  private_nh.param<double>("heur_dis_cof", heur_dis_cof_, 20.0);
  private_nh.param<double>("node2goal_yaw", node2goal_yaw_, 0.05);  // rad

  private_nh.param<int>("path_seg_num", path_seg_num_, 6);
  private_nh.param<int>("path_fit_degree", path_fit_degree_, 3);
  private_nh.param<int>("path_resolution", path_resolution_, 20);
  private_nh.param<bool>("path_optimize_flg", path_optimize_flg_, false);

  //** 2020.03.04 modify
  private_nh.param<bool>("use_goal_flg", use_goal_flg_, false);
  private_nh.param<double>("move_step", move_step_, 2.4);
  private_nh.param<double>("segment_dis", segment_dis_, 0.2);
  private_nh.param<int>("extension_point_num", extension_point_num_, 5);
  private_nh.param<double>("min_turning_radius", min_turning_radius_, 22.0);
  private_nh.param<double>("heur_yaw_cof_goal", heur_yaw_cof_goal_, 20.0);
  private_nh.param<double>("heur_yaw_cof_last", heur_yaw_cof_last_, 20.0);
  private_nh.param<double>("heur_curvature_cof", heur_curvature_cof_, 10.0);
  private_nh.param<double>("heur_reference_line_cof", reference_line_cof_, 10.0);

  //** 待删除
  discrete_angle_ = 2.0 * M_PI / static_cast<double>(angle_size_);
  private_nh.param<int>("path_discrete_num", path_discrete_num_, 10);
  private_nh.param<int>("angle_size", angle_size_, 96);

  path_op_client_ =
      private_nh.serviceClient<optimize_path::PathFitInfo>("optimize_service");

  // CreateUpdateSet(path_discrete_num_);
  CreateUpdateSet();
}

// create update pose set prime, start pose is origin
void HybridAstar::CreateUpdateSet(int num) {
  update_set_.clear();
  update_pass_set_.clear();

  move_step_ = min_turning_radius_ * discrete_angle_;
  update_pass_set_.resize(3);

  geometry_msgs::Pose temp;
  temp.position.x = 0.0;
  temp.position.y = 0.0;
  temp.position.z = start_pose_.position.z;
  temp.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // ego 不含起始点, 包含终点
  double temp_discrete = move_step_ / static_cast<double>(num);
  for (int i = 0; i < num; ++i) {
    temp.position.x = (i + 1) * temp_discrete;
    update_pass_set_[0].push_back(temp);
  }

  // left
  temp.position.x = 0.0;
  temp.position.y = 0.0;
  temp.position.z = start_pose_.position.z;
  temp.orientation = tf::createQuaternionMsgFromYaw(0.0);
  temp_discrete = discrete_angle_ / num;
  for (int i = 0; i < num; ++i) {
    temp.position.x = min_turning_radius_ * sin(temp_discrete * (i + 1));
    temp.position.y = min_turning_radius_ * (1 - cos(temp_discrete * (i + 1)));
    temp.orientation = tf::createQuaternionMsgFromYaw(temp_discrete * (i + 1));
    update_pass_set_[1].push_back(temp);
  }

  // right
  temp.position.x = 0.0;
  temp.position.y = 0.0;
  temp.position.z = start_pose_.position.z;
  temp.orientation = tf::createQuaternionMsgFromYaw(0.0);
  temp_discrete = discrete_angle_ / num;
  for (int i = 0; i < num; ++i) {
    temp.position.x = min_turning_radius_ * sin(temp_discrete * (i + 1));
    temp.position.y =
        -1.0 * min_turning_radius_ * (1 - cos(temp_discrete * (i + 1)));
    temp.orientation = tf::createQuaternionMsgFromYaw(-temp_discrete * (i + 1));
    update_pass_set_[2].push_back(temp);
  }

  for (auto& elem : update_pass_set_) {
    update_set_.push_back(elem.back());
  }
}

//** 2020.03.04 modify
void HybridAstar::CreateUpdateSet() {
  update_set_.clear();
  update_pass_set_.clear();
  update_points_orientation_stg_.clear();

  const int extension_point_half_num = (extension_point_num_ - 1) >> 1;
  // const double discrete_angle = 2.0 * M_PI /
  // static_cast<double>(angle_size_);

  //** 扩展一步的距离
  const double move_step = move_step_;

  geometry_msgs::Pose temp;
  temp.position.x = 0.0;
  temp.position.y = 0.0;
  temp.position.z = 0.0;
  temp.orientation = tf::createQuaternionMsgFromYaw(0.0);

  update_pass_set_.resize(extension_point_half_num * 2 + 1);
  update_points_orientation_stg_.resize(extension_point_half_num + 1);

  // ego 不含起始点, 包含终点
  int num = std::ceil(move_step / segment_dis_);
  for (int i = 0; i < num; ++i) {
    temp.position.x = (i + 1) * segment_dis_;
    update_pass_set_[0].push_back(temp);
  }

  //** 每份转弯半径
  const double ave_turning_radius =
      1.0 / min_turning_radius_ / extension_point_half_num;

  //** 动态的angle_size
  angle_size_ = std::ceil(2 * M_PI / (move_step_ * ave_turning_radius));

  for (int k = 1; k <= extension_point_half_num; ++k) {
    double current_turning_radius = 1.0 / (ave_turning_radius * k);
    double temp_discrete = move_step / current_turning_radius / num;

    // left
    temp.position.x = 0.0;
    temp.position.y = 0.0;
    temp.position.z = 0.0;
    temp.orientation = tf::createQuaternionMsgFromYaw(0.0);

    for (int i = 0; i < num; ++i) {
      temp.position.x = current_turning_radius * sin(temp_discrete * (i + 1));
      temp.position.y =
          current_turning_radius * (1 - cos(temp_discrete * (i + 1)));
      temp.orientation =
          tf::createQuaternionMsgFromYaw(temp_discrete * (i + 1));
      update_pass_set_[k * 2 - 1].push_back(temp);
    }

    // rigth
    temp.position.x = 0.0;
    temp.position.y = 0.0;
    temp.position.z = start_pose_.position.z;
    temp.orientation = tf::createQuaternionMsgFromYaw(0.0);

    for (int i = 0; i < num; ++i) {
      temp.position.x = current_turning_radius * sin(temp_discrete * (i + 1));
      temp.position.y =
          current_turning_radius * (1 - cos(temp_discrete * (i + 1))) * (-1.0);
      temp.orientation =
          tf::createQuaternionMsgFromYaw(-temp_discrete * (i + 1));
      update_pass_set_[k * 2].push_back(temp);
    }
  }

  for (int i = 0; i < update_pass_set_.size(); ++i) {
    update_set_.push_back(update_pass_set_[i].back());
    if (i % 2 != 0)
      update_points_orientation_stg_[(i + 1) >> 1] =
          tf::getYaw(update_pass_set_[i].back().orientation);
  }
}

void HybridAstar::UpdatePoseShow(const ros::Publisher& pub) {
  visualization_msgs::MarkerArray show_array;

  // show start pose
  visualization_msgs::Marker start_pose;
  start_pose.header.frame_id = "global";

  start_pose.ns = "start_pose";
  start_pose.id = 0;

  start_pose.type = 2;
  start_pose.action = visualization_msgs::Marker::ADD;

  start_pose.pose = start_pose_;

  start_pose.scale.x = 0.2;
  start_pose.scale.y = 0.2;
  start_pose.scale.z = 0.01;

  start_pose.color.r = 0.76f;
  start_pose.color.g = 1.0f;
  start_pose.color.b = 0.24f;
  start_pose.color.a = 1.0f;

  show_array.markers.push_back(start_pose);

  // show update_pass set
  UPDATE_SET update_pass_set_tf = UpdateSetTransform(start_pose_);
  visualization_msgs::Marker update_pass_pose;
  update_pass_pose.header.frame_id = "global";

  update_pass_pose.ns = "update_pass_pose";
  update_pass_pose.id = 1;

  update_pass_pose.type = 7;
  update_pass_pose.action = visualization_msgs::Marker::ADD;

  update_pass_pose.scale.x = 0.1;
  update_pass_pose.scale.y = 0.1;
  update_pass_pose.scale.z = 0.01;

  update_pass_pose.color.r = 0.36f;
  update_pass_pose.color.g = 0.28f;
  update_pass_pose.color.b = 0.55f;
  update_pass_pose.color.a = 1.0f;
  for (auto& elem : update_pass_set_tf) {
    for (auto& item : elem) {
      update_pass_pose.points.push_back(item.position);
    }
  }
  show_array.markers.push_back(update_pass_pose);

  // show prime path
  visualization_msgs::Marker prime_path;
  prime_path.header.frame_id = "global";

  prime_path.ns = "prime_path";
  prime_path.id = 2;

  prime_path.type = 7;
  prime_path.action = visualization_msgs::Marker::ADD;

  prime_path.scale.x = 0.4;
  prime_path.scale.y = 0.4;
  prime_path.scale.z = 0.01;

  prime_path.color.r = 0.54f;
  prime_path.color.g = 0.17f;
  prime_path.color.b = 0.88f;
  prime_path.color.a = 1.0f;
  for (auto& elem : prime_path_temp_) {
    prime_path.points.push_back(elem.position);
  }
  show_array.markers.push_back(prime_path);

  // show final path
  visualization_msgs::Marker final_path;
  final_path.header.frame_id = "global";

  final_path.ns = "final_path";
  final_path.id = 3;

  final_path.type = 7;
  final_path.action = visualization_msgs::Marker::ADD;

  final_path.scale.x = 0.25;
  final_path.scale.y = 0.25;
  final_path.scale.z = 0.01;

  final_path.color.r = 0.4f;
  final_path.color.g = 1.0f;
  final_path.color.b = 0.4f;
  final_path.color.a = 1.0f;
  for (auto& elem : final_path_) {
    final_path.points.push_back(elem.position);
  }
  show_array.markers.push_back(final_path);

  pub.publish(show_array);
}

// all point on expand path transform
UPDATE_SET HybridAstar::UpdateSetTransform(const geometry_msgs::Pose& pose) {
  UPDATE_SET update_pass_set_tf;
  update_pass_set_tf.resize(update_pass_set_.size());

  int iter = 0;
  for (auto& elem : update_pass_set_) {
    for (auto& item : elem) {
      update_pass_set_tf[iter].emplace_back(PoseTransform(pose, item));
    }
    ++iter;
  }
  return update_pass_set_tf;
}

// expand point transform
UPDATE_POS HybridAstar::UpdateNodeTransform(const geometry_msgs::Pose& pose) {
  UPDATE_POS update_set_tf;

  for (auto& elem : update_set_) {
    update_set_tf.emplace_back(PoseTransform(pose, elem));
  }
  return update_set_tf;
}

// pose transform
geometry_msgs::Pose HybridAstar::PoseTransform(
    const geometry_msgs::Pose& central, const geometry_msgs::Pose& pose) {
  double yaw = tf::getYaw(central.orientation);

  geometry_msgs::Pose temp_pose;
  temp_pose.position.x = pose.position.x * cos(yaw) -
                         pose.position.y * sin(yaw) + central.position.x;
  temp_pose.position.y = pose.position.x * sin(yaw) +
                         pose.position.y * cos(yaw) + central.position.y;
  temp_pose.position.z = central.position.z;

  temp_pose.orientation =
      tf::createQuaternionMsgFromYaw(yaw + tf::getYaw(pose.orientation));

  return temp_pose;
}
geometry_msgs::Pose HybridAstar::PoseTransformPath(
    const AstarNode* const central, const geometry_msgs::Pose& pose) {
  double yaw = central->yaw;

  geometry_msgs::Pose temp_pose;
  temp_pose.position.x =
      pose.position.x * cos(yaw) - pose.position.y * sin(yaw) + central->x;
  temp_pose.position.y =
      pose.position.x * sin(yaw) + pose.position.y * cos(yaw) + central->y;
  temp_pose.orientation =
      tf::createQuaternionMsgFromYaw(yaw + tf::getYaw(pose.orientation));

  return temp_pose;
}
AstarNode HybridAstar::PoseTransform(const AstarNode* const central,
                                     const geometry_msgs::Pose& pose) {
  AstarNode temp_node;
  double yaw = central->yaw;

  temp_node.x =
      pose.position.x * cos(yaw) - pose.position.y * sin(yaw) + central->x;
  temp_node.y =
      pose.position.x * sin(yaw) + pose.position.y * cos(yaw) + central->y;

  temp_node.yaw = TranformYawRange(central->yaw + tf::getYaw(pose.orientation));

  return temp_node;
}

void HybridAstar::UpgrateParam() {
  ros::NodeHandle private_nh("~compute_path");

  // private_nh.param<int>("path_discrete_num", path_discrete_num_, 10);

  private_nh.param<double>("node2goal_r", node2goal_r_, 1.0);
  private_nh.param<double>("heur_dis_cof", heur_dis_cof_, 20.0);
  private_nh.param<double>("node2goal_yaw", node2goal_yaw_, 0.05);

  private_nh.param<int>("path_seg_num", path_seg_num_, 6);
  private_nh.param<int>("path_fit_degree", path_fit_degree_, 3);
  private_nh.param<int>("path_resolution", path_resolution_, 20);
  private_nh.param<bool>("path_optimize_flg", path_optimize_flg_, false);

  //** 2020.03.04 modify
  private_nh.param<int>("angle_size", angle_size_, 96);
  private_nh.param<bool>("use_goal_flg", use_goal_flg_, false);
  private_nh.param<double>("move_step", move_step_, 2.4);
  private_nh.param<double>("segment_dis", segment_dis_, 0.2);
  private_nh.param<int>("extension_point_num", extension_point_num_, 5);
  private_nh.param<double>("min_turning_radius", min_turning_radius_, 22.0);
  private_nh.param<double>("heur_yaw_cof_goal", heur_yaw_cof_goal_, 20.0);
  private_nh.param<double>("heur_yaw_cof_last", heur_yaw_cof_last_, 20.0);
  private_nh.param<double>("heur_curvature_cof", heur_curvature_cof_, 10.0);
  private_nh.param<double>("heur_reference_line_cof", reference_line_cof_, 10.0);

  // discrete_angle_ = 2.0 * M_PI / static_cast<double>(angle_size_);

  CreateUpdateSet();
  // CreateUpdateSet(path_discrete_num_);
}

bool HybridAstar::ComputePath() {
  int cnt = 0;
  while (!openlist_.empty()) {
    AstarNode* cur_node = &node_stg_[openlist_.top().index];
    double f_min = openlist_.top().key.f;
    int open_list_size = openlist_.size();
    openlist_.pop();

    if (fabs(cur_node->g + cur_node->h - f_min) > DBL_EPSILON) {
      // std::cout << cnt++ << ": curent expend pose : "
      //           << "[" << cur_node->x << ", " << cur_node->y << ", "
      //           << cur_node->yaw << "]"
      //           << "  f_min: " << f_min << " != " << cur_node->g +
      //           cur_node->h
      //           << "  continue" << open_list_size << std::endl;
      // openlist_.emplace(CalculateID(cur_node), CalculateKey(cur_node));
      continue;
    }

    if (IsGoal(cur_node)) {
      // std::cout << "find goal : "
      //           << "[" << cur_node->x << ", " << cur_node->y << ", "
      //           << cur_node->yaw << "]" << std::endl;
      find_goal_info_ = cur_node;
      if (CalculateID(cur_node) != goal_pose_id_) {
        goal_info_->best_p = cur_node->best_p;
      }

      return true;
    }

    // std::cout << cnt++ << ": curent expend pose : "
    //           << "[" << cur_node->x << ", " << cur_node->y << ", "
    //           << cur_node->yaw << "  s_id:" << CalculateID(cur_node) << "]"
    //           << "  f_min: " << f_min << " open_list_size: " <<
    //           open_list_size
    //           << std::endl;

    if (cur_node->state != STATE::CLOSE) {
      cur_node->state = STATE::CLOSE;
      std::vector<AstarNode> neighbors = GetNeighbors(cur_node);
      for (auto& elem : neighbors) {
        PushNode(elem, cur_node);
      }
    }
  }
  std::cout << "open list empty !!" << std::endl;
  return false;
}

void HybridAstar::Init() {
  // stg container clear
  OPENLIST open_empty;
  std::swap(openlist_, open_empty);
  NODE_TYPE node_empty;
  std::swap(node_stg_, node_empty);
  final_path_.clear();
  prime_path_.clear();
  expend_pos_stg_.clear();
  prime_path_temp_.clear();
  prime_path_optimize_.clear();

  CreateHollowList();

  goal_pose_id_ = CalculateID(goal_pose_.position.x, goal_pose_.position.y,
                              tf::getYaw(goal_pose_.orientation));
  start_pose_id_ = CalculateID(start_pose_.position.x, start_pose_.position.y,
                               tf::getYaw(start_pose_.orientation));

  double start_h =
      CalculateDisTwoPoint(start_pose_.position, goal_pose_.position);
  node_stg_[goal_pose_id_] = AstarNode(goal_pose_, DBL_MAX, 0.0);
  node_stg_[start_pose_id_] = AstarNode(start_pose_, 0.0, start_h);

  goal_info_ = &node_stg_[goal_pose_id_];
  start_info_ = &node_stg_[start_pose_id_];
  // std::cout << "goal yaw = " << goal_info_->yaw << std::endl;

  // start point push openlist
  openlist_.emplace(start_pose_id_, KeyValue(start_info_->h, 0.0));
  start_info_->state = STATE::OPEN;
}

// get neighbors
std::vector<AstarNode> HybridAstar::GetNeighbors(const AstarNode* const node) {
  std::vector<AstarNode> neighbors;
  int iter = 0;
  for (auto& elem : update_set_) {
    AstarNode temp_node = PoseTransform(node, elem);

    if (!IsInMap(temp_node.x, temp_node.y) ||
        // IsObstacle(CalculateXYIndex(temp_node.x, temp_node.y)))
        detectCollision(temp_node)) {
      continue;
    } else {
      //** 2020.03.04 modify
      // 当前node与下一个点之间的yaw差值
      // double dis_yaw_to_last = fabs(tf::getYaw(elem.orientation));
      temp_node.g = node->g + move_step_;
      temp_node.h = CalculateHeuristic(temp_node, node);
      if (iter) temp_node.h += move_step_ * 0.0;
      neighbors.push_back(temp_node);
      // std::cout << "neighbor : "
      //           << "[" << temp_node.x << ", " << temp_node.y << ", "
      //           << temp_node.yaw << ", " << temp_node.g << ", " <<
      //           temp_node.h
      //           << "]" << std::endl;
    }
    ++iter;
  }
  return neighbors;
}

// push node
bool HybridAstar::PushNode(const AstarNode& node, AstarNode* const p) {
  static int push_cnt = 0;

  bool res = false;
  auto node_id = CalculateID(node);

  auto iter = node_stg_.find(node_id);

  if (iter == node_stg_.end()) {
    node_stg_[node_id] =
        AstarNode(node.x, node.y, node.yaw, node.g, node.h, p, STATE::OPEN);
    KeyValue key_temp = CalculateKey(node);
    // std::cout << "push num: " << push_cnt++ << "  [" << key_temp.f << ","
    //           << key_temp.g << "  s_id:" << node_id
    //           << "  P_id:" << CalculateID(p) << "]" << std::endl;
    openlist_.emplace(node_id, CalculateKey(node));
    expend_pos_stg_.insert(node_id);

    res = true;
  } else {
    // if (iter->second.state != STATE::CLOSE) {
    if ((node.g + node.h) < (iter->second.g + iter->second.h)) {
      // iter->second.g = node.g;
      // iter->second.h = node.h;
      // iter->second.best_p = p;

      // iter->second.x = node.x;
      // iter->second.
      iter->second = node;
      iter->second.best_p = p;
      KeyValue key_temp = CalculateKey(iter->second);
      // std::cout << "push num: " << push_cnt++ << "  [" << key_temp.f << ","
      //           << key_temp.g << "  s_id:" << node_id
      //           << "  P_id:" << CalculateID(p) << "]******" << std::endl;
      openlist_.emplace(node_id, CalculateKey(iter->second));
      iter->second.state = STATE::OPEN;
      expend_pos_stg_.insert(node_id);
      res = true;
    }
    // }
  }
  return res;
}

// path recur
void HybridAstar::RecurPath() {
  // std::cout << "** path recur ***" << std::endl;
  AstarNode* node_ptr = use_goal_flg_ ? goal_info_ : find_goal_info_;
  while (node_ptr != NULL) {
    // std::cout << "[" << node_ptr->x << ", " << node_ptr->y << ", "
    //           << node_ptr->yaw << "]" << std::endl;
    prime_path_temp_.emplace_back(node_ptr->AstarNodeTransformGeometryPose());
    prime_path_.push_back(node_ptr);

    node_ptr = node_ptr->best_p;
  }

  std::reverse(prime_path_.begin(), prime_path_.end());

  // 记录path评价值
  path_length_ = (prime_path_.size() - 1) * move_step_;  // path length
  max_curvature_diff_ = 0.0;
  path_curvature_value_ = 0.0;
  for (int i = 2; i < prime_path_.size(); ++i) {
    double temp = fabs(
        TranformYawRange(prime_path_[i]->yaw - prime_path_[i - 1]->yaw) -
        TranformYawRange(prime_path_[i - 1]->yaw - prime_path_[i - 2]->yaw));
    path_curvature_value_ += temp;

    max_curvature_diff_ = std::max(max_curvature_diff_, temp);
  }
  // 方向盘平均旋转圈数
  curvature_average_diff_ = path_curvature_value_ / move_step_ *
                            min_turning_radius_ * 2.0 /
                            (prime_path_.size() - 2);

  // 总的曲率变化值
  path_curvature_value_ /= move_step_ / 100.0;

  // 方向盘最大旋转圈数
  max_curvature_diff_ =
      max_curvature_diff_ / move_step_ * min_turning_radius_ * 2.0;
}

// get final path
bool HybridAstar::FinalPath() {
  if (!path_optimize_flg_ || prime_path_.size() < path_seg_num_ ||
      !path_op_client_.exists()) {
    final_path_.emplace_back(
        prime_path_.front()->AstarNodeTransformGeometryPose());
    for (int i = 0; i < prime_path_.size() - 1; ++i) {
      double delta_yaw = prime_path_[i + 1]->yaw - prime_path_[i]->yaw;
      delta_yaw = TranformYawRange(delta_yaw);
      for (auto& elem : update_pass_set_[JudgeOrientation(delta_yaw)]) {
        final_path_.emplace_back(PoseTransformPath(prime_path_[i], elem));
      }
    }
    return true;
  } else {
    optimize_path::PathFitInfo cli;
    cli.request.fit_degree = path_fit_degree_;
    cli.request.path_seg_num = path_seg_num_;
    //** 修改为随move_step变化
    cli.request.path_resolution =
        std::ceil(static_cast<double>(path_seg_num_ - 1) * move_step_ * 10.0);
    cli.request.prime_path = GetXYYAWvector();

    // while (!path_op_client_.waitForExistence(ros::Duration(1))) {
    // };
    // ros::WallTime start = ros::WallTime::now();

    int call_cnt = 0;
    bool call_successful_flg = false;
    while (call_cnt < 5 && path_op_client_.exists()) {
      if (path_op_client_.call(cli)) {
        prime_path_optimize_ = cli.response.final_path;
        // std::cout << "final_path size : " << cli.response.final_path.size()
        //           << std::endl;
        // for (auto& elem : cli.response.final_path) std::cout << elem << "  ";
        // std::cout << std::endl;
        call_successful_flg = true;
        break;
      } /*else {*/
      ++call_cnt;
      // }
    }

    // ros::WallTime end = ros::WallTime::now();

    // std::cout << "spend time: " << (end - start).toSec() * 1000 << " ms"
    //           << std::endl;

    int x_size = prime_path_optimize_.size() / 3;
    for (int i = 0; i < x_size; ++i) {
      final_path_.emplace_back(XYYAWTramsformGeometryPose(
          prime_path_optimize_[i], prime_path_optimize_[i + x_size],
          prime_path_optimize_[i + 2 * x_size]));
    }
    return call_successful_flg;
  }
}

// execute hybrid-Astar
bool HybridAstar::ExecuteHybridAstar() {
  // init
  // std::cout << "123" << std::endl;
  Init();
  // std::cout << "456" << std::endl;

  // compute path
  bool flag = ComputePath();
  // std::cout << "789  compute path result --> " << flag << std::endl;
  if (!flag) {
    HintShow("Planning Failed !!");
    return false;
  }
  HintShow("Planning Successful !!");

  RecurPath();

  bool path_flg = FinalPath();
  if (!flag) {
    HintShow("Path Optimize Failed !!");
    return false;
  }

  return true;
}

// print path
void HybridAstar::PrintPath() {
  //   std::cout << "** final result **" << std::endl;
  //   std::cout << "expend pos nums : " << expend_pos_stg_.size() << std::endl
  //             << "path:" << std::endl;
  //   std::cout << "------------prime_path_-------------" << std::endl;
  //   for (auto& elem : prime_path_) {
  //     std::cout /*<< "[" */ << elem->x << ", " << elem->y << ", "
  //                           << elem->yaw /*<<
  // "]"*/
  //                           << std::endl;
  //   }
  // std::cout << "[";
  std::cout << "--------------final_path_-----------" << std::endl;

  for (auto& elem : final_path_) {
    std::cout << elem.position.x << "  " << elem.position.y << "  "
              << tf::getYaw(elem.orientation) << std::endl;
  }

  std::cout << "*************curvature*************" << std::endl;
  // std::cout << "]" << std::endl;
  // std::cout << "[";
  curvature_data_stg_.resize(final_path_.size()-1);
  for (int i = 1; i < final_path_.size(); ++i) {
    double temp_curve =
        TranformYawRange(tf::getYaw(final_path_[i].orientation) -
                         tf::getYaw(final_path_[i - 1].orientation)) /
        std::hypot(final_path_[i].position.x - final_path_[i - 1].position.x,
                   final_path_[i].position.y - final_path_[i - 1].position.y);
    // std::cout << std::round(temp_curve *
    //                         (update_points_orientation_stg_.size() - 1) *
    //                         min_turning_radius_)
    //           << std::endl;
    std::cout << temp_curve << std::endl;
    curvature_data_stg_[i-1] = temp_curve;
  }
  // std::cout << "]" << std::endl;
}

// /* collision checking new
//  */
// bool HybridAstar::detectCollision(const AstarNode& node) {
//   Collision collision_checker;

//   PointSet_type truck_frame = GetTruckFrame(node);
//   collision_checker.obj_1_ = truck_frame;

//   for (auto& elem : hollow_list_) {
//     collision_checker.obj_2_ = elem;
//     if (collision_checker.IsCollision(true)) return true;
//   }
//   return false;
// }

// 检测碰撞 old by xc
// 做法:将truck投影的平面上,然后检测每一个点是否是障碍物
bool HybridAstar::detectCollision(const AstarNode& node) {
  // define the truck as rectangle
  double left = -1.0 * truck_base2back_;
  double right = truck_length_ - truck_base2back_;
  double top = truck_width_ / 2;
  double bottom = -1.0 * truck_width_ / 2;
  double resolution = 1.0;

  // std::cout << "truck_width_:" << truck_width_ << std::endl;

  // Coordinate of base_link in ogm frame
  double one_angle_range = 2.0 * M_PI / angle_size_;
  double base_x = node.x;
  double base_y = node.y;
  double base_theta = node.yaw;
  // std::cout<< "base_theta: " << base_theta << std::endl;

  // Calculate cos and sin in advance
  double cos_theta = std::cos(base_theta);
  double sin_theta = std::sin(base_theta);

  // Convert each point to index and check if the node is Obstacle
  for (double x = left; x < right; x += resolution) {
    for (double y = top + 1.0; y >= bottom; y -= resolution) {
      // 2D coordinate rotate
      double index_x = (x * cos_theta - y * sin_theta + base_x) / resolution;
      double index_y = (x * sin_theta + y * cos_theta + base_y) / resolution;

      if (!IsInMap(index_x, index_y)) {
        return true;
      }
      if (IsObstacle(calculateXYIndexMap(index_x, index_y))) {
        return true;
      }
    }
  }
  return false;
}
