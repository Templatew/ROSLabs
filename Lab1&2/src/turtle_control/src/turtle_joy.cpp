#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

class TurtleJoy : public rclcpp::Node {
public:
    TurtleJoy() : Node("turtle_joy") {

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&TurtleJoy::joy_callback, this, std::placeholders::_1));
        
        this->declare_parameter("linear_gain_x", 1.0);
        this->declare_parameter("linear_gain_y", 1.0);
        this->declare_parameter("angular_gain", 1.0);
        
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        auto twist = geometry_msgs::msg::Twist();
        auto linear_x = get_parameter("linear_gain_x").as_double();
        auto linear_y = get_parameter("linear_gain_y").as_double();
        auto angular_z = get_parameter("angular_gain").as_double();


        twist.linear.x = msg->axes[1] * linear_x;
        twist.linear.y = msg->axes[0] * linear_y;
        twist.angular.z = msg->axes[3] * angular_z;
        

        RCLCPP_INFO(this->get_logger(), "Gains - linear_x: %f, linear_y: %f, angular_z: %f", linear_x, linear_y, angular_z);
        RCLCPP_INFO(this->get_logger(), "Values - v_x: %f, v_y: %f, a_z: %f", msg->axes[1], msg->axes[0], msg->axes[3]);

        publisher_->publish(twist);

    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleJoy>());
    rclcpp::shutdown();
    return 0;
}
