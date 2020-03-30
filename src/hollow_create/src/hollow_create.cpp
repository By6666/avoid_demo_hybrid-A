#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <ros/ros.h>
#include <tf/tf.h>

#include <iostream>

int main(int argc, char** argv) {
  ros::init(argc, argv, "hollow_create");
  ros::NodeHandle nh("~");
  ros::Publisher pub =
      nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("hollow_info", 1);

  double size_x, size_y, place_x, place_y, yaw;

  nh.param<double>("yaw", yaw, 0.0);
  nh.param<double>("size_x", size_x, 10.0);
  nh.param<double>("size_y", size_y, 5.0);
  nh.param<double>("place_y", place_y, 0.0);
  nh.param<double>("place_x", place_x, 12.0);

  jsk_recognition_msgs::BoundingBoxArray boxes_array;
  jsk_recognition_msgs::BoundingBox box_temp;

  boxes_array.header.frame_id = "global";
  boxes_array.header.stamp = ros::Time::now();

  box_temp.header.frame_id = "global";
  box_temp.header.stamp = ros::Time::now();

  box_temp.label = 0;
  box_temp.dimensions.x = size_x;
  box_temp.dimensions.y = size_y;
  box_temp.dimensions.z = 1.0;

  box_temp.pose.position.x = place_x;
  box_temp.pose.position.y = place_y;
  box_temp.pose.position.z = 0.0;

  box_temp.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

  boxes_array.boxes.push_back(box_temp);

  while (ros::ok()) {
    nh.param<double>("yaw", yaw, 0.0);
    nh.param<double>("size_x", size_x, 10.0);
    nh.param<double>("size_y", size_y, 10.0);
    nh.param<double>("place_y", place_y, 0.0);
    nh.param<double>("place_x", place_x, 12.0);

    boxes_array.boxes.back().dimensions.x = size_x;
    boxes_array.boxes.back().dimensions.y = size_y;

    boxes_array.boxes.back().pose.position.x = place_x;
    boxes_array.boxes.back().pose.position.y = place_y;

    boxes_array.boxes.back().pose.orientation =
        tf::createQuaternionMsgFromYaw(yaw);

    pub.publish(boxes_array);
    ros::Duration(0.1).sleep();
  }

  return 0;
}
