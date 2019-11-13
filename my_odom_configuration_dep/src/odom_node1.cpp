#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <iostream>

using namespace std;

double x = 0.0;
double y = 0.0;
double th = 0.0;

double vx = 0.0;
double vy = 0.0;
double vth = 0.0;

void handle_odom(const geometry_msgs::Vector3Stamped& odom_msg){
  x  = odom_msg.vector.x;
  y  = odom_msg.vector.y;
  th  = odom_msg.vector.z;
}

void handle_vel(const geometry_msgs::Vector3Stamped& vel_msg){
  vx = vel_msg.vector.x;
  vth = vel_msg.vector.z;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  ros::Subscriber sub_odom = n.subscribe("odom_xy", 100, handle_vel);
  ros::Subscriber sub_vel = n.subscribe("current_vel_xyz", 100, handle_vel);
  ros::Publisher chatter1 = n.advertise<std_msgs::Float32>("v_x", 100);
  ros::Publisher chatter2 = n.advertise<std_msgs::Float32>("v_th", 100);
  
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0);

  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
 
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
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    cout << "VX: " << vx << endl;
    cout << "VTH: " << vth << endl;
  
    last_time = current_time;
    r.sleep();
  }
}
