#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

using namespace std;

double x;
double y;
double th;

double vx;
double vy;
double vz;
double va;
double vb;
double vth;

void handle_pos(const geometry_msgs::Twist& pos_msg){
  x = pos_msg.linear.x;
  th = pos_msg.angular.z;
}

void handle_vel(const geometry_msgs::Twist& vel_msg){
  vx = vel_msg.linear.x;
  vth = vel_msg.angular.z;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Subscriber sub_pos = n.subscribe("current_pos_xz", 100, handle_pos);
  ros::Subscriber sub_vel = n.subscribe("current_vel_xyz", 100, handle_vel);
  tf::TransformBroadcaster odom_broadcaster;

  x = 0.0;
  y = 0.0;
  th = 0.0;

  vx = 0.0;
  vy = 0.0;
  vth = 0.0;

  //ros::Rate r(1000.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    //cout << "VX: " << vx << endl;
    //cout << "VTH: " << vth << endl;
    //r.sleep();
  }
}
