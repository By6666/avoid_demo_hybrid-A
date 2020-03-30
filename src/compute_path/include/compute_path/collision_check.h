/* collision checking
 */

#ifndef COLLISION_CHECK_H
#define COLLISION_CHECK_H

#include <geometry_msgs/Point.h>

#include <iostream>
#include <vector>

typedef geometry_msgs::Point Point_type;
typedef std::vector<Point_type> PointSet_type;

class Line {
 public:
  Line() = default;
  Line(const Point_type& start, const Point_type& end)
      : p_start(start), p_end(end) {}

  Point_type p_end;
  Point_type p_start;
};

class Collision {
 public:
  Collision() = default;

  // first input is truck, second input is obstacle or hollow
  Collision(const PointSet_type& object_1, const PointSet_type& object_2)
      : obj_1_(object_1), obj_2_(object_2) {}

  bool IsCollision(bool is_hollow = false);

  PointSet_type obj_1_;
  PointSet_type obj_2_;

 private:
  int order(const Line& line, const Point_type& pt) const;
  bool onSegment(const Line& line, const Point_type& pt) const;
  bool IsIntersection(const Line& line1, const Line& line2) const;
};

#endif