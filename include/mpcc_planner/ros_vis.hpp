#ifndef ROS_VIS_HPP
#define ROS_VIS_HPP

#include "mpcc_planner/arc_spline.hpp"
#include <Eigen/Core>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace ros_vis {
class RosVis {
 private:
  double trackWidth_;
  // ros publishers
  ros::Publisher track_center_pub_, track_left_pub_, track_right_pub_;
  ros::Publisher traj_pub_, car_pub_, ref_pub_;
  // ros msgs
  nav_msgs::Path track_center_msg_, track_left_msg_, track_right_msg_, traj_msg_, ref_msg_;
  visualization_msgs::Marker car_msg_;
 public:
  RosVis(ros::NodeHandle n);
  ~RosVis() {};
  void publish_track(arc_spline::ArcSpline &arc);
  void publish_traj(const Eigen::MatrixXd &state, arc_spline::ArcSpline &arc);
  typedef std::shared_ptr<RosVis> Ptr;
};

RosVis::RosVis(ros::NodeHandle n) {
  n.getParam("trackWidth", trackWidth_);
  track_center_pub_ = n.advertise<nav_msgs::Path>("track_center", 1);
  track_left_pub_ = n.advertise<nav_msgs::Path>("track_left", 1);
  track_right_pub_ = n.advertise<nav_msgs::Path>("track_right", 1);
  traj_pub_ = n.advertise<nav_msgs::Path>("traj", 1);
  ref_pub_ = n.advertise<nav_msgs::Path>("ref", 1);
  car_pub_ = n.advertise<visualization_msgs::Marker>("car", 1);
  // msgs
  track_center_msg_.header.frame_id = "world";
  track_left_msg_.header.frame_id = "world";
  track_right_msg_.header.frame_id = "world";
  ref_msg_.header.frame_id = "world";
  traj_msg_.header.frame_id = "world";
  car_msg_.header.frame_id = "world";
  car_msg_.type = visualization_msgs::Marker::ARROW;
  car_msg_.action = visualization_msgs::Marker::ADD;
  car_msg_.scale.x = 0.02;
  car_msg_.scale.y = 0.02;
  car_msg_.scale.z = 0;
  car_msg_.color.a = 1;
  car_msg_.color.r = 1;
  car_msg_.color.g = 0;
  car_msg_.color.b = 0;
}

void RosVis::publish_track(arc_spline::ArcSpline &arc) {
  track_center_msg_.poses.resize(0);
  geometry_msgs::PoseStamped tmpPoint;
  tmpPoint.header.frame_id = "world";
  tmpPoint.pose.position.z = 0;
  for (double s=0; s<arc.arcL(); s+=0.1) {
    tmpPoint.pose.position.x = arc(s)(0);
    tmpPoint.pose.position.y = arc(s)(1);
    track_center_msg_.poses.push_back(tmpPoint);
    tmpPoint.pose.position.x = arc(s)(0) - arc(s,1)(1)*trackWidth_/2;
    tmpPoint.pose.position.y = arc(s)(1) + arc(s,1)(0)*trackWidth_/2;
    track_left_msg_.poses.push_back(tmpPoint);
    tmpPoint.pose.position.x = arc(s)(0) + arc(s,1)(1)*trackWidth_/2;
    tmpPoint.pose.position.y = arc(s)(1) - arc(s,1)(0)*trackWidth_/2;
    track_right_msg_.poses.push_back(tmpPoint);
  }
  track_center_pub_.publish(track_center_msg_);
  // track_left_pub_.publish(track_left_msg_);
  // track_right_pub_.publish(track_right_msg_);
}

void RosVis::publish_traj(const Eigen::MatrixXd &state, arc_spline::ArcSpline &arc) {
  // trajetory
  traj_msg_.poses.resize(0);
  geometry_msgs::PoseStamped tmpPoint;
  tmpPoint.header.frame_id = "world";
  for (int i=0; i<state.cols(); ++i) {
    tmpPoint.pose.position.x = state.coeff(0,i);
    tmpPoint.pose.position.y = state.coeff(1,i);
    tmpPoint.pose.position.z = 0;
    traj_msg_.poses.push_back(tmpPoint);
  }
  traj_pub_.publish(traj_msg_);
  // car
  car_msg_.points.resize(2);
  geometry_msgs::Point pt;
  double x = state.coeffRef(0,0);
  double y = state.coeffRef(1,0);
  double phi = state.coeffRef(2,0);
  double v = state.coeffRef(3,0);
  pt.x = x;
  pt.y = y;
  pt.z = 0;
  car_msg_.points[0] = pt;
  pt.x += 0.1*v*std::cos(phi);
  pt.y += 0.1*v*std::sin(phi);
  car_msg_.points[1] = pt;
  car_pub_.publish(car_msg_);
  // ref
  ref_msg_.poses.resize(0);
  for (int i=0; i<state.cols(); ++i) {
    double s = state.coeffRef(4,i);
    s = std::fmod(s+arc.arcL(), arc.arcL());
    Eigen::Vector2d ps = arc(s);
    tmpPoint.pose.position.x = ps(0);
    tmpPoint.pose.position.y = ps(1);
    tmpPoint.pose.position.z = 0;
    ref_msg_.poses.push_back(tmpPoint);
  }
  ref_pub_.publish(ref_msg_);
}

} // namespace ros_vis

#endif
