#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace behavior_manager {

/**
 * @brief Set the gripper state (open or closed) using the specified publisher.
 * @param publisher The ROS publisher for sending gripper state messages.
 * @param gripper_open A boolean indicating whether to open or close the
 * gripper.
 */
void setGripperState(ros::Publisher &publisher, bool gripper_open);

} // namespace behavior_manager