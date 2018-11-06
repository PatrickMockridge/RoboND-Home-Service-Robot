#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h> //Goal position
#include <actionlib/client/simple_action_client.h> //Commanding to naviagate to goal
#include <visualization_msgs/Marker.h> //Goal Marker

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;
ros::Subscriber marker_subscriber;

//set time limit for pursuing the goal pose
int goalTimeout = 200;

move_base_msgs::MoveBaseGoal createGoal(const visualization_msgs::Marker &m){
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = m.pose;
  return goal;
}
  
bool pursueGoal(move_base_msgs::MoveBaseGoal &g){
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer()){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  ac.sendGoal(g);
  
  ac.waitForResult(ros::Duration(goalTimeout));

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){ 
    return true;
  }else{
  	return false;
  }
  
}

void markerCallback(const visualization_msgs::Marker &m){
  ROS_INFO("Received new goal location. Plotting path...");
  move_base_msgs::MoveBaseGoal goal = createGoal(m);
  bool success = pursueGoal(goal);
  
  if(success){
    ROS_INFO("Goal pose reached. Waiting 5 seconds...");
    ros::Duration(5.0).sleep();
  }else{
    ROS_INFO("Failed to reach goal pose for some reason.  ¯\\_(ツ)_/¯");
  }
}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  
  // Create a ROS NodeHandle object
  ros::NodeHandle n;
  
  //create marker subscriber
  marker_subscriber = n.subscribe("/visualization_marker", 1, markerCallback);
  
  // Enter an infinite loop where the marker_callback function will be called when new marker messages arrive
  ros::Duration time_between_ros_wakeups(0.5);
  while (ros::ok()) {
    ros::spinOnce();
    time_between_ros_wakeups.sleep();
  }

}