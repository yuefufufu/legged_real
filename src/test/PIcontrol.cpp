#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <algorithm> 
#include <geometry_msgs/Twist.h>

using namespace std;

ros::Publisher PI_pub;
geometry_msgs::Twist pi_pub;
double prop, der;
double Kp = 2;
double Kd = 1;
double p_bef = 0;

void PI_callback(const geometry_msgs::Twist& point_sub)
{
    prop=point_sub.linear.x-0.4;
    der=p_bef-point_sub.linear.x;
    pi_pub.linear.y=Kp*prop+Kd*der;
    p_bef=point_sub.linear.x;

  cout << point_sub.linear.x << endl;
  cout << pi_pub.linear.y << endl;
  
  PI_pub.publish(pi_pub);
  ros::Rate loop_rate(100);
  loop_rate.sleep();
}
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "PIcontrol");
  ros::NodeHandle nh;
  PI_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber rs_sub = nh.subscribe("point_vel", 1000, PI_callback);
  ros::spin();
  return 0;
}