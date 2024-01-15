#pragma once

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include "cleaning_sequence/move_to_destination.hpp"
#include "cleaning_sequence/set_gripper_state.hpp"

namespace behavior_manager {

/**
 * @brief The BehaviorManager class manages robot behavior during a cleaning
 * sequence.
 */
class BehaviorManager {

public:
  /**
   * @brief Default constructor for the BehaviorManager class.
   */
  BehaviorManager();

private:
  /**
   * @brief Reads parameters from the ROS parameter server.
   */
  void getYamlParameters();

#ifdef DEBUG_MODE
  /**
   * @brief Callback function for handling interactive marker feedback (debug
   * mode only).
   * @param msg The InteractiveMarkerFeedback message received from the
   * interactive marker.
   */
  void callbackInteractiveMarker(
      const visualization_msgs::InteractiveMarkerFeedback::ConstPtr &msg);
#endif

  ros::NodeHandle m_nh,
      m_nh_private; ///< ROS node handles for public and private namespaces.
  ros::Subscriber m_subscriber_viz_feedback; ///< ROS subscriber for interactive
                                             ///< marker feedback.
  ros::Publisher m_publisher_marker; ///< ROS publisher for general markers.
  ros::Publisher
      m_publisher_marker_ee; ///< ROS publisher for end-effector markers.
  ros::Publisher
      m_publisher_gripper_state; ///< ROS publisher for gripper state.
  tf2_ros::Buffer tfBuffer;      ///< Buffer for managing transformation data.
  tf2_ros::TransformListener
      tfListener; ///< Listener for obtaining transformations.
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      m_move_base_action_client; ///< Action client for MoveBase action.

  move_base_msgs::MoveBaseGoal m_move_base_goal; ///< Goal for MoveBase action.
  geometry_msgs::Pose m_ee_pose; ///< End-effector pose information.
};
} // namespace behavior_manager
