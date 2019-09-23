#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include<visualization_msgs/Marker.h>
#include<opencv2/opencv.hpp>
#include<send_msg/rt.h>

double x;
double y;
double z;
double x0;
double y00;
double z0;


void Callback(const send_msg::rt & msg)
{
  static ros::NodeHandle n;
  static ros::Publisher armor_rel = n.advertise<visualization_msgs::Marker>("armor_topic3",1);
  static visualization_msgs::Marker cube0;

  x = msg.rx;
  y = msg.ry;
  z = msg.rz;

  x0 = msg.x / 10;
  y00 = msg.z / 10;
  z0 = msg.y / 10;

  cube0.header.frame_id = "/my_frame";
  cube0.header.stamp = ros::Time::now();

  cube0.ns = "cube0";
  cube0.id = 0;

  cube0.type = visualization_msgs::Marker::CUBE;
  cube0.action = visualization_msgs::Marker::ADD;

  cube0.pose.position.x = x0 / 2;
  cube0.pose.position.y = y00 / 2;
  cube0.pose.position.z = z0 / 2;

  cube0.scale.x = 1.35;
  cube0.scale.y = 0.15;
  cube0.scale.z = 0.55;

  cube0.pose.orientation.x = msg.rx;
  cube0.pose.orientation.y = msg.rz;
  cube0.pose.orientation.z = 0;
  cube0.pose.orientation.w = 1;

  cube0.color.r = 0.0f;
  cube0.color.g = 1.0f;
  cube0.color.b = 0.0f;
  cube0.color.a = 1.0;
  cube0.lifetime = ros::Duration();

  armor_rel.publish(cube0);
  cv::waitKey(30);

}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("joker", 1000, &Callback);
  ros::spin();
  return 0;
};