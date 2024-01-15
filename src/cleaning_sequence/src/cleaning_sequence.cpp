#include "cleaning_sequence/cleaning_sequence.hpp"

namespace behavior_manager {

BehaviorManager::BehaviorManager()
    : m_nh_private("~"), m_move_base_action_client("/move_base", true),
      tfListener(tfBuffer) {
  getYamlParameters();

  // Initialize ROS Interface
  m_publisher_gripper_state =
      m_nh.advertise<std_msgs::Float64>("/rh_p12_rn_position/command", 1, true);

#ifdef DEBUG_MODE
  m_subscriber_viz_feedback =
      m_nh.subscribe("/rviz_moveit_motion_planning_display/"
                     "robot_interaction_interactive_marker_topic/feedback",
                     1, &BehaviorManager::callbackInteractiveMarker, this);
  m_publisher_marker =
      m_nh.advertise<geometry_msgs::PoseStamped>("/destination_pose", 1);
  m_publisher_marker_ee =
      m_nh.advertise<geometry_msgs::PoseStamped>("/destination_pose_ee", 1);
#endif

  // P2P Navigation
  moveTo(m_move_base_action_client, m_move_base_goal);

  // Set Gripper State
  setGripperState(m_publisher_gripper_state, false);

  ros::spin();
}

void BehaviorManager::getYamlParameters() {
  float x, y, yaw;
  m_nh_private.getParam("move_base_goal/x", x);
  m_nh_private.getParam("move_base_goal/y", y);
  m_nh_private.getParam("move_base_goal/yaw", yaw);

  static constexpr float degrees_to_radians = 3.14159 / 180.;
  updateGoal(m_move_base_goal, x, y, yaw * degrees_to_radians);
}

#ifdef DEBUG_MODE
void BehaviorManager::callbackInteractiveMarker(
    const visualization_msgs::InteractiveMarkerFeedback::ConstPtr &msg) {
  ROS_INFO("Inside Callback..");
  geometry_msgs::PoseStamped msg_pose, msg_pose_final;
  msg_pose.pose = msg->pose;
  msg_pose.header = msg->header;
  // msg_pose.header.frame_id =
  m_publisher_marker.publish(msg_pose);

  try {
    geometry_msgs::TransformStamped transformStamped =
        tfBuffer.lookupTransform("rh_p12_rn_base", "base_link", ros::Time(0));

    tf2::doTransform(msg_pose, msg_pose_final, transformStamped);
    // done = true;
    std::cout << "Pose: " << msg_pose_final << "\n";
    // break;
    msg_pose_final.header.frame_id = "rh_p12_rn_base";
    m_ee_pose = msg_pose_final.pose;
    m_publisher_marker_ee.publish(msg_pose_final);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }
}
#endif

} // namespace behavior_manager

int main(int argc, char **argv) {
  ros::init(argc, argv, "cleaning_sequence");
  behavior_manager::BehaviorManager behavior_manager;
  return 0;
}