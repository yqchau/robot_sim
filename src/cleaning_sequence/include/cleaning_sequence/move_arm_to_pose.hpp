// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>
// #include <moveit_msgs/DisplayRobotState.h>
// #include <moveit_msgs/DisplayTrajectory.h>

// Set Gripper State
// setGripperState(m_publisher_gripper_state, false);

// Pose:
//   header:
//   seq: 0
//   stamp: 338.300000000
//   frame_id: rh_p12_rn_base
// pose:
//   position:
//     x: 0.328257
//     y: -0.237357
//     z: -0.0784945
//   orientation:
//     x: 9.20337e-05
//     y: -0.706826
//     z: -9.21896e-05
//     w: 0.707388

//   ros::AsyncSpinner spinner(1);
//   spinner.start();

//   sleep(1.0);

//   static const std::string PLANNING_GROUP = "ur3_manipulator";
//   moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
//   const robot_state::JointModelGroup *joint_model_group =
//     move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

//   ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());
//   ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
//   geometry_msgs::Pose target_pose1;
//   target_pose1.orientation.w = 1.0;
//   target_pose1.position.x = 0.28;
//   target_pose1.position.y = -0.7;
//   target_pose1.position.z = 1.0;
//   move_group.setPoseTarget(target_pose1);
//   move_group.move();

//   // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

//   // bool success = (move_group.plan(my_plan) ==
//   moveit::core::MoveItErrorCode::SUCCESS);
//   // ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//   /* Sleep to give Rviz time to visualize the plan. */
//   sleep(5.0);

//   ros::waitForShutdown();