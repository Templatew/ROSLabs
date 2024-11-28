#include "rclcpp/rclcpp.hpp"
#include "interbotix_xs_msgs/msg/joint_group_command.hpp"
#include "interbotix_xs_msgs/msg/joint_single_command.hpp"
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

class ArmController : public rclcpp::Node {
public:
ArmController() : Node("arm_controller") {
    // Publisher to the /px100/commands/joint_group topic
    arm_publisher_ = this->create_publisher<interbotix_xs_msgs::msg::JointGroupCommand>(
        "/px100/commands/joint_group", 10);

    // Publisher to the /px100/commands/joint_single topic for the gripper
    gripper_publisher_ = this->create_publisher<interbotix_xs_msgs::msg::JointSingleCommand>(
        "/px100/commands/joint_single", 10);

    // Timer to send commands
    timer_ = this->create_wall_timer(
        3s, std::bind(&ArmController::publish_commands, this));

    // Initialize joint positions
    command1_.name = "arm";
    command1_.cmd = {0.0, 0.0, 0.0, 0.0}; // First Position

    command2_.name = "arm";
    command2_.cmd = {1.0, 0.0, -1.0, 0.0}; // Second Position

    // Initialize gripper commands
    gripper_close_.name = "gripper";
    gripper_close_.cmd = -300.0; // Close the gripper

    gripper_open_.name = "gripper";
    gripper_open_.cmd = 300.0; // Open the gripper

    // Start with the first command
    use_command1_ = true;
}

private:
void publish_commands() {
    if (use_command1_) {
        RCLCPP_INFO(this->get_logger(), "Publishing Command 1 and Closing Gripper");
        arm_publisher_->publish(command1_);
        gripper_publisher_->publish(gripper_close_);
    } else {
        RCLCPP_INFO(this->get_logger(), "Publishing Command 2 and Opening Gripper");
        arm_publisher_->publish(command2_);
        gripper_publisher_->publish(gripper_open_);
    }
    use_command1_ = !use_command1_; // Toggle between commands
}

rclcpp::Publisher<interbotix_xs_msgs::msg::JointGroupCommand>::SharedPtr arm_publisher_;
rclcpp::Publisher<interbotix_xs_msgs::msg::JointSingleCommand>::SharedPtr gripper_publisher_;
rclcpp::TimerBase::SharedPtr timer_;
interbotix_xs_msgs::msg::JointGroupCommand command1_;
interbotix_xs_msgs::msg::JointGroupCommand command2_;
interbotix_xs_msgs::msg::JointSingleCommand gripper_close_;
interbotix_xs_msgs::msg::JointSingleCommand gripper_open_;
bool use_command1_;
};

int main(int argc, char **argv) {
rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<ArmController>());
rclcpp::shutdown();
return 0;
}
