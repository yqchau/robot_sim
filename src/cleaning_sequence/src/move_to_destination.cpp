#include "cleaning_sequence/move_to_destination.hpp"

namespace behavior_manager {

void moveTo(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
                &action_client,
            const move_base_msgs::MoveBaseGoal &goal) {
  // Callback when the goal is completed
  auto doneCb = [](const actionlib::SimpleClientGoalState &state,
                   const move_base_msgs::MoveBaseResultConstPtr &result) {
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
  };
  // Callback when the goal becomes active
  auto activeCb = []() { ROS_INFO("Goal just went active"); };
  // Callback for feedback during the execution of the goal
  auto feedbackCb =
        [](const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) {};

  // Wait for the action server to come up
  ROS_INFO("Waiting for the action server to come up");
  action_client.waitForServer();

  ROS_INFO("Sending goal");
  action_client.sendGoal(goal, doneCb, activeCb, feedbackCb);

  // Wait for the result (optional)
  action_client.waitForResult(ros::Duration(30.0));
}

void updateGoal(move_base_msgs::MoveBaseGoal &goal, float x, float y,
                float yaw) {
  // Convert yaw to quaternion
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw); // Roll, pitch, yaw

  // Print the quaternion components
  ROS_INFO("Quaternion: x=%.3f, y=%.3f, z=%.3f, w=%.3f", quaternion.x(),
           quaternion.y(), quaternion.z(), quaternion.w());

  // Create and send the goal
  goal.target_pose.header.frame_id = "odom";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.x = quaternion.x();
  goal.target_pose.pose.orientation.y = quaternion.y();
  goal.target_pose.pose.orientation.z = quaternion.z();
  goal.target_pose.pose.orientation.w = quaternion.w();
}
} // namespace behavior_manager
