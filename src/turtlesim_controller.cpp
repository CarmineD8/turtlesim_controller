#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class TurtlesimController: public rclcpp::Node
{
public:
TurtlesimController(): Node("turtlesim_controller")
 {
 subscription_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&TurtlesimController::topic_callback, this, _1));
 publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
 timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&TurtlesimController::timer_callback, this));
 }

private:

void timer_callback()
 {
    if(x_<9.0 and x_>2.0){
    message.linear.x = 1.0;
    message.angular.z = 0.0;
    }
    else if (x_>=9.0){
    message.linear.x = 1.0;
    message.angular.z = 1.0;
    }
    else if (x_<=2.0){
    message.linear.x = 1.0;
    message.angular.z = -1.0;
    }
    publisher_->publish(message);
 }

 void topic_callback(const turtlesim::msg::Pose::SharedPtr msg)
 {
 RCLCPP_INFO(this->get_logger(), "The position of the turtle is (x, y): '%f, %f'", msg->x, msg->y);
 x_ = msg->x;
 }

 rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
 rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
 rclcpp::TimerBase::SharedPtr timer_;
 geometry_msgs::msg::Twist message;
 float x_;
};

int main(int argc, char * argv[])
{
 rclcpp::init(argc, argv);
 auto node = std::make_shared<TurtlesimController>();
 rclcpp::spin(node);
 rclcpp::shutdown();
 return 0;
}