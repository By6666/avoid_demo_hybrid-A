#include "compute_path/collision_check.h"

/* 判断点在线的位置
 */
int Collision::order(const Line& line, const Point_type& pt) const {
  // 通过向量的外积判断
  int intOrientation = (line.p_end.y - line.p_start.y) * (pt.x - line.p_end.x) -
                       (line.p_end.x - line.p_start.x) * (pt.y - line.p_end.y);
  if (intOrientation == 0) return 0;  // colinear
  if (intOrientation > 0) return 1;   // right side
  if (intOrientation < 0) return -1;  // left side
};

/* 判断点是否在直线上
   true:在直线上  false:不在
 */
bool Collision::onSegment(const Line& line, const Point_type& pt) const {
  if ((pt.x <= std::max(line.p_start.x, line.p_end.x)) &&
      (pt.x >= std::min(line.p_start.x, line.p_end.x)) &&
      (pt.y <= std::max(line.p_start.y, line.p_end.y)) &&
      (pt.y >= std::min(line.p_start.y, line.p_end.y))) {
    return true;
  } else {
    return false;
  }
}

/* 判断两条直线是否相交
   true:相交  false:不相交
 */
bool Collision::IsIntersection(const Line& line1, const Line& line2) const {
  int order1 = order(line1, line2.p_start);
  int order2 = order(line1, line2.p_end);
  int order3 = order(line2, line1.p_start);
  int order4 = order(line2, line1.p_end);
  if (order1 != order2 && order3 != order4) {
    return true;
  }
  if (order1 == 0 && onSegment(line1, line2.p_start)) {
    return true;
  }
  if (order2 == 0 && onSegment(line1, line2.p_end)) {
    return true;
  }
  if (order3 == 0 && onSegment(line2, line1.p_start)) {
    return true;
  }
  if (order4 == 0 && onSegment(line2, line1.p_end)) {
    return true;
  }
  return false;
}

/* 判断两个obj是否collision
   true: 碰撞  false: 不碰撞

truck ego exp:
      pt4   pt1
         ego   -->
      pt3   pt2
*/
bool Collision::IsCollision(bool is_hollow) {
  std::vector<Line> bA;  // bA is truck
  if (is_hollow) {
    bA.emplace_back(obj_1_.front(), obj_1_.back());
    bA.emplace_back(obj_1_[1], obj_1_[2]);
  } else {
    for (int16_t i = 0; i < obj_1_.size(); ++i) {
      bA.emplace_back(obj_1_[i], obj_1_[(i + 1) % obj_1_.size()]);
    }
  }

  std::vector<Line> bB;  // bB is obs or hollow
  for (int16_t i = 0; i < obj_2_.size(); ++i) {
    bB.emplace_back(obj_2_[i], obj_2_[(i + 1) % obj_2_.size()]);
  }

  for (int i = 0; i < bA.size(); i++) {
    for (int j = 0; j < bB.size(); j++) {
      if (IsIntersection(bA[i], bB[j])) {
        return true;
      }
    }
  }
  return false;
}
