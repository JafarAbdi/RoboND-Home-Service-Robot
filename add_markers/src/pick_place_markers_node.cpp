#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

static geometry_msgs::Pose pose_msg;

double distance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {
  double dx = p2.position.x - p1.position.x;
  double dy = p2.position.y - p1.position.y;
  return dx * dx + dy * dy;
}

void pose_cb(const nav_msgs::Odometry& msg) { pose_msg = msg.pose.pose; }

int main(int argc, char** argv) {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber pose_sub = n.subscribe("/odom", 10, pose_cb);

  geometry_msgs::Pose pick_pose, drop_pose;
  pick_pose.position.x = 4.5;
  pick_pose.position.y = -3.0;
  pick_pose.position.z = 0.0;
  pick_pose.orientation.x = 0.0;
  pick_pose.orientation.y = 0.0;
  pick_pose.orientation.z = 0.0;
  pick_pose.orientation.w = 1.0;
  drop_pose.position.x = -4.5;
  drop_pose.position.y = -3.0;
  drop_pose.orientation.x = 0.0;
  drop_pose.orientation.y = 0.0;
  drop_pose.orientation.z = 0.707;
  drop_pose.orientation.w = 0.707;
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on
  // these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique
  // ID Any marker sent with the same namespace and id will overwrite the old
  // one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and
  // SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3
  // (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the
  // frame/time specified in the header
  marker.pose = pick_pose;
  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    ros::Duration(1.0).sleep();
  }

  ROS_INFO("Publishing to add the pick place position");
  marker_pub.publish(marker);
  while (distance(pick_pose, pose_msg) > 0.1) ros::spinOnce();

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3
  // (DELETEALL)
  marker.action = visualization_msgs::Marker::DELETE;
  // Set the pose of the marker.  This is a full 6DOF pose relative to the
  // frame/time specified in the header
  marker.pose = pick_pose;
  ROS_INFO("Publishing to delete the pick place position");
  marker_pub.publish(marker);
  ROS_INFO("Waiting for 5 seconds");
  ros::Duration(5.0).sleep();

  while (distance(drop_pose, pose_msg) > 0.35) ros::spinOnce();

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3
  // (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;
  // Set the pose of the marker.  This is a full 6DOF pose relative to the
  // frame/time specified in the header
  marker.pose = drop_pose;
  ROS_INFO("Publishing to add the drop place position");
  marker_pub.publish(marker);
  return 0;
}
