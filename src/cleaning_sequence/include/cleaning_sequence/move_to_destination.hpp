#pragma once

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace behavior_manager {

/**
 * @brief Move the robot to a specified goal using the MoveBase action.
 * @param action_client The SimpleActionClient for MoveBase action.
 * @param goal The MoveBase goal to be achieved.
 */
void moveTo(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
                &action_client,
            const move_base_msgs::MoveBaseGoal &goal);

/**
 * @brief Update the MoveBase goal with specified position and orientation.
 * @param goal The MoveBase goal to be updated.
 * @param x The x-coordinate of the goal in the "odom" frame.
 * @param y The y-coordinate of the goal in the "odom" frame.
 * @param yaw The yaw angle (orientation) of the goal in radians.
 */
void updateGoal(move_base_msgs::MoveBaseGoal &goal, float x, float y,
                float yaw);
} // namespace behavior_manager
