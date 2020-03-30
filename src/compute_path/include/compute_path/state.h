#ifndef STATE_H
#define STATE_H

#include "geometry_msgs/PoseStamped.h"
#include "jsk_recognition_msgs/BoundingBoxArray.h"
#include "tf/tf.h"

typedef uint64_t _Type_ID;
typedef jsk_recognition_msgs::BoundingBoxArray _Type_Hollow;

enum STATE { NONE = 0, CLOSE = 1, OPEN = 2 };

class AstarNode {
 public:
  AstarNode() = default;
  AstarNode(const geometry_msgs::Pose& arg_pose, double arg_g, double arg_h,
            AstarNode* arg_best_p = NULL, STATE arg_state = STATE::NONE)
      : x(arg_pose.position.x),
        y(arg_pose.position.y),
        yaw(tf::getYaw(arg_pose.orientation)),
        g(arg_g),
        h(arg_h),
        best_p(arg_best_p),
        state(arg_state){};

  AstarNode(double arg_x, double arg_y, double arg_yaw, double arg_g,
            double arg_h, AstarNode* arg_best_p = NULL,
            STATE arg_state = STATE::NONE)
      : x(arg_x),
        y(arg_y),
        yaw(arg_yaw),
        g(arg_g),
        h(arg_h),
        best_p(arg_best_p),
        state(arg_state){};

  double x, y, yaw;
  double g;
  double h;
  AstarNode* best_p;
  STATE state;

  double get_all_cost() { return g + h; }

  inline geometry_msgs::Pose AstarNodeTransformGeometryPose() {
    geometry_msgs::Pose temp;
    temp.position.x = this->x;
    temp.position.y = this->y;
    temp.orientation = tf::createQuaternionMsgFromYaw(this->yaw);
    return temp;
  }
};

class KeyValue {
 public:
  KeyValue() = default;
  KeyValue(double arg_f, double arg_g) : f(arg_f), g(arg_g) {}

  double f;
  double g;
};

class ComNode {
 public:
  ComNode() = default;
  ComNode(_Type_ID index, const KeyValue& arg_key) : index(index), key(arg_key){};

  KeyValue key;
  _Type_ID index;

  bool operator>(const ComNode& right) const {
    if (fabs(key.f - right.key.f) < DBL_EPSILON)
      return key.g > right.key.g;
    else
      return key.f > right.key.f;
  }
};

#endif