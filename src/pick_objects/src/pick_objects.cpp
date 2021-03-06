#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
// Define a client to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 3.5;
  goal.target_pose.pose.position.y = 2.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending the pickup goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base moved to the pickup location!");
    ROS_INFO("Waiting for 5 seconds");
    ros::Duration(5.0).sleep();
    goal.target_pose.pose.position.x = -3.8;
    goal.target_pose.pose.position.y = 4.7;
    goal.target_pose.pose.orientation.w = -1.0;
    goal.target_pose.header.stamp = ros::Time::now();
    ROS_INFO("Sending the dropoff goal");
    ac.sendGoal(goal);
      

    // Wait an infinite time for the results
    ac.waitForResult();
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, the base moved to the dropoff location!");
    } else {
      ROS_WARN("The base failed to move to the second goal for some reason");
    }
  } else {
    ROS_WARN("The base failed to move to the second goal for some reason");
  }
  ros::spin();
  return 0;
}
