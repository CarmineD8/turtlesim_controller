// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <chrono>
#include <cinttypes>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim_controller/srv/harmonic.hpp"

using std::placeholders::_1;

namespace turtlesim_controller{

class MyController : public rclcpp::Node
{
public:
  MyController(const rclcpp::NodeOptions & options)
  : Node("my_controller", options)
  { 
    //initialize the publisher, the subscriber, client1, client2
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/my_turtle/cmd_vel", 1);
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/my_turtle/pose", 10, std::bind(&MyController::topic_callback, this, _1));
      
    client1_ = this->create_client<turtlesim::srv::Kill>("/kill");
    while (!client1_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    
    client2_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
    while (!client2_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    
    client3_ = this->create_client<turtlesim_controller::srv::Harmonic>("/harmonic");
    while (!client3_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
    }
    
    this->call_client1();
    this->call_client2();
    
  }
  
private:

  void call_client1(){
  auto request = std::make_shared<turtlesim::srv::Kill::Request>();
  request->name = "turtle1";
  auto result_future = client1_->async_send_request(request);
  }
  
  void call_client2(){
  auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
  request->name = "my_turtle";
  request->x=2.5;
  request->y=1.0;
  request->theta=0.0;
  auto result_future = client2_->async_send_request(request);
  }

  void topic_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "The position is: x: %f y: %f z: %f", msg->x, msg->y, msg->theta);
    auto message = geometry_msgs::msg::Twist();
    if(msg->x>9.0){
	    message.linear.x=1.0;
	    message.angular.z=1.0;
	    RCLCPP_INFO(this->get_logger(), "Publishing: vel_x %f, vel_z %f", message.linear.x, message.linear.z);
	    publisher_->publish(message);
    }
    else if (msg->x<2.0){
	    message.linear.x=1.0;
	    message.angular.z=-1.0;
	    RCLCPP_INFO(this->get_logger(), "Publishing: vel_x %f, vel_z %f", message.linear.x, message.linear.z);
	    publisher_->publish(message);
    }
    else {
	    auto request = std::make_shared<turtlesim_controller::srv::Harmonic::Request>();
	    request->pos = msg->x;
	    
	    using ServiceResponseFuture =
	    rclcpp::Client<turtlesim_controller::srv::Harmonic>::SharedFuture;
	    auto response_received_callback = [this](ServiceResponseFuture future) {
		      auto message = geometry_msgs::msg::Twist();
		      message.linear.x=future.get()->vel;
		      RCLCPP_INFO(this->get_logger(), "Publishing: vel_x %f, vel_z %f", message.linear.x, message.linear.z);
		      publisher_->publish(message);
    	};
    auto future_result = client3_->async_send_request(request, response_received_callback);
    }
    
  }
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client1_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client2_;
  rclcpp::Client<turtlesim_controller::srv::Harmonic>::SharedPtr client3_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(turtlesim_controller::MyController) 

