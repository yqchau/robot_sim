#include "cleaning_sequence/set_gripper_state.hpp"

namespace behavior_manager {

void setGripperState(ros::Publisher &publisher, bool gripper_open) {
  std_msgs::Float64 msg;
  if (gripper_open) // open
  {
    msg.data = 0.0;
    publisher.publish(msg);
  } else {
    msg.data = 1.05;
    publisher.publish(msg);
  }

  // Sleep to allow time for gripper to reach the desired state
  ros::Duration(5.0).sleep();
}

} // namespace behavior_manager
