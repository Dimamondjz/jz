#ifndef MAV_ROS_INTERFACE_HPP
#define MAV_ROS_INTERFACE_HPP

namespace mav_ros_interface {

class MavRosInterface {
 private:
  ros::NodeHandle n_;
  ros::Publisher cmd_pub_;

 public:
  MavRosInterface(ros::NodeHandle n);
  ~MavRosInterface(){};
  bool initMavRos();
  void pubCmd(double a, double beta);
  typedef std::shared_ptr<MavRosInterface> Ptr;
};  // class MavRosInterface

MavRosInterface::MavRosInterface(ros::NodeHandle n) {
  n_ = n;
  // TODO
  // cmd_pub_ = ...
}

bool MavRosInterface::initMavRos() {
  // TODO
  return true;
}

void pubCmd(double a, double beta) {
  // TODO
}

}  // namespace mav_ros_interface

#endif
