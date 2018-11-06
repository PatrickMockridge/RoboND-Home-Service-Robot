#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
//#include <tf.h> //to calculate difference in yaw between goal and actual pose
#include <geometry_msgs/PoseWithCovariance.h> //Pose information
#include <nav_msgs/Odometry.h>
#include <math.h> //to calculate euclidean distance
#include <string>
#include <iostream>

using namespace std;

//set allowable heading error (in radians) to goal pose
const float headingTolerance = 0.3;
//set allowable position error to goal pose (meters)
const float positionTolerance = .5;

//flag to detect if goal pose has been reached
bool poseAchieved = false;

geometry_msgs::Pose goalPose;

void testPoseCallback(const nav_msgs::Odometry::ConstPtr &msg){
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ros::Duration(1.0).sleep();
}

const float positionError(const geometry_msgs::Pose &bot, const geometry_msgs::Pose &goal){
  const float distance = sqrt(pow(bot.position.x-goal.position.x, 2) + pow(bot.position.y-goal.position.y,2));

  //const float distance = p1.position.distance(p2.position);
  return distance;
}

const float headingError(const geometry_msgs::Pose &bot, const geometry_msgs::Pose &goal){
  return 0.0;
}

void poseCallback(const nav_msgs::Odometry::ConstPtr &msg){
  const geometry_msgs::Pose p = msg->pose.pose;
  const float headingErr = headingError(p, goalPose);
  const float positionErr = positionError(p, goalPose);

  if((headingErr < headingTolerance && positionErr < positionTolerance)){
    poseAchieved = true;
  }else{
    ROS_INFO("Current pose error exceeds pose tolerance of %f radians (heading) and %f meters (position)", headingTolerance, positionTolerance);
    ROS_INFO("Heading Error: Ignored", headingErr);
    ROS_INFO("Position Error: %f\n", positionErr);
  }
}

visualization_msgs::Marker createMarker(float (&pose)[7]){
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_marker";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pose[0];
  marker.pose.position.y = pose[1];
  marker.pose.position.z = pose[2];
  marker.pose.orientation.x = pose[3];
  marker.pose.orientation.y = pose[4];
  marker.pose.orientation.z = pose[5];
  marker.pose.orientation.w = pose[6];

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  return marker;
}

void publishPose(ros::NodeHandle &n, float (&pose)[7], ros::Duration &timeout){
  poseAchieved = false;

  //convert the pose array to a marker message
  visualization_msgs::Marker marker = createMarker(pose);

  //set the current goal pose to the marker pose
  goalPose = marker.pose;

  //create a new marker publisher
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  //check if anything is subscribing to the marker publisher
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      break;
    }
    ROS_WARN_ONCE("Warning: Create a subscriber to the marker");
    ros::Duration(1).sleep();
  }

  //listen to the bot odometry to determine if goal pose is reached
  //compare against published marker pose as goal pose (need boost bind to pass this into callback)
  ROS_INFO("Creating odometry subscriber.");
  ros::Subscriber bot_pose_subscriber = n.subscribe("odom", 1, poseCallback);

  ROS_INFO("Publishing marker.");
  marker_pub.publish(marker);

  //mark start time to track time since marker published
  ros::Time start = ros::Time::now();
  ros::Time now = ros::Time::now();


  ros::Rate loop_frequency(1); //check pose at 1Hz
  while((now - start)<timeout && !poseAchieved && ros::ok()){
    ros::spinOnce();
    now = ros::Time::now();
    marker_pub.publish(marker); //publish again in case it was missed
    loop_frequency.sleep();
  }
}

bool checkIfTesting(){
  //determine if running in testing mode
  ROS_INFO("Run in test mode? Enter 0 for No or 1 for Yes");

  int testing;
  while(true){
    if(std::cin >> testing){
      return (bool) testing;
    }else{
      ROS_INFO("Please enter either 0 or 1");
    }
  }
}

int main( int argc, char** argv ){
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;

  //determine if running in testing mode
  //if so, just immediately move on to next marker
  ros::Duration poseTimeout;
  bool testing = checkIfTesting();
  if(testing){
    poseTimeout = ros::Duration(0.0);
  }else{
    poseTimeout = ros::Duration(150.0);
  }

  //send pickup pose with testing status flag
  float pickupPose[7] = {-4.16, 1.64, 0.03, 0, 0, 1.0, 0.55};
  ROS_INFO("Publishing pick up pose marker.");
  publishPose(n, pickupPose, poseTimeout);

  if(poseAchieved){
    ROS_INFO("Reached pick up pose! Waiting 3 seconds before sending drop off pose...");
  }else{
    ROS_INFO("Failed to reach the pick up pose. Trying again. Waiting 3 seconds...");
  }
  ros::Duration(3).sleep();

  //send drop off pose with testing status flag
  float dropoffPose[7] = {3.47, 4.28, 0.0, 0, 0, 1.0, 2.0};
  ROS_INFO("Publishing drop off pose marker.");
  publishPose(n, dropoffPose, poseTimeout);

  if(poseAchieved){
    ROS_INFO("Drop off pose reached.");
  }else{
    ROS_INFO("Failed to reach the drop off pose.");
  }
  ros::Duration(3).sleep();

  return 0;

}
