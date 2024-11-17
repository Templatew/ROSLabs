#include <chrono>
#include <functional>
#include <memory>



#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std::chrono_literals;


class TurtlePublisher : public rclcpp::Node
{
public:
    TurtlePublisher() : Node("turtle_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&TurtlePublisher::publish_message, this));
    }

private:
    void publish_message()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 1.0;  // Vitesse linéaire
        message.angular.z = 0.5; // Vitesse angulaire
        publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtlePublisher>());
    rclcpp::shutdown();
    return 0;
}
