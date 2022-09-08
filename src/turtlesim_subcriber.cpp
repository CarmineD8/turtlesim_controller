#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "turtlesim/Kill.h"
#include "turtlesim/TeleportAbsolute.h"
#include "turtlesim_controller/Vel.h"
#include "turtlesim_controller/Harmonic.h"

float pos_x;
float pos_y;

void turtleCallback(const turtlesim::Pose::ConstPtr& msg)
	{
	ROS_INFO("Turtle subscriber@[%f, %f, %f]",  
	msg->x, msg->y, msg->theta);
	
	pos_x = msg->x;
	pos_y = msg->y;

}


int main (int argc, char **argv)
{
// Initialize the node, setup the NodeHandle for handling the communication with the ROS //system  
	ros::init(argc, argv, "turtlebot_subscriber");  
	ros::NodeHandle nh;
	// Define the publisher and the subscriber to turtle's position 
	ros::ServiceClient client2 = nh.serviceClient<turtlesim::Kill>("/kill");
	turtlesim::Kill srv2;
	srv2.request.name = "turtle1";
	client2.waitForExistence();
	client2.call(srv2);
	
	ros::ServiceClient client1 = nh.serviceClient<turtlesim::Spawn>("/spawn");
	turtlesim::Spawn srv1;
	srv1.request.x = 2.0;
	srv1.request.y = 1.0;
 	srv1.request.theta = 0.0;
 	srv1.request.name = "rt1_turtle";
 	client1.waitForExistence();
	client1.call(srv1);
	
	
	ros::Publisher pub = nh.advertise<geometry_msgs::Twist> ("rt1_turtle/cmd_vel", 1); 
	ros::Subscriber sub = nh.subscribe("rt1_turtle/pose", 1,turtleCallback);
	
	ros::Publisher pub2 = nh.advertise<turtlesim_controller::Vel>("rt1_turtle/my_vel", 1);
	
	ros::ServiceClient client3 = nh.serviceClient<turtlesim_controller::Harmonic>("/harmonic");
	
	ros::Rate loop_rate(1000);
	while (ros::ok()){
	
	geometry_msgs::Twist my_vel;
	
	turtlesim_controller::Harmonic harm_vel;
	harm_vel.request.x = pos_x;
	client3.call(harm_vel);
	
	if (pos_x > 9.0){
	my_vel.linear.x = 1.0;
	my_vel.angular.z = 1.0;
	}
	else if (pos_x < 2.0){
	my_vel.linear.x = 1.0;
	my_vel.angular.z = -1.0;
	}
	else{
	my_vel.linear.x = harm_vel.response.vel;
	my_vel.angular.z = 0.0;
	}
	if(pos_y > 9.0)
	{
	my_vel.linear.x = 0.0;
	my_vel.angular.z = 0.0;
	}
	pub.publish(my_vel);
	
	turtlesim_controller::Vel new_vel;
	new_vel.name = "linear_velocity";
	new_vel.vel = my_vel.linear.x;
	pub2.publish(new_vel);
	
	
	ros::spinOnce();
     	loop_rate.sleep();
  	}
	
	return 0;
}


