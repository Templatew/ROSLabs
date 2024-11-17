

# Report: ROS Lab Sessions 1 & 2 CAZAUBON Lorenz - DESUE Léo

## Table of Contents
- [Report: ROS Lab Sessions 1 \& 2 CAZAUBON Lorenz - DESUE Léo](#report-ros-lab-sessions-1--2-cazaubon-lorenz---desue-léo)
  - [Table of Contents](#table-of-contents)
  - [Lab 1](#lab-1)
    - [First Hello World package](#first-hello-world-package)
    - [Turtlesim Publisher](#turtlesim-publisher)

## Lab 1


### First Hello World package

*For our very first package we decided to follow [this](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) tutorial from start to finish.*

- Once all files were correctly created, we built the package with this command:

    ```bash
    lorenz@Legion:~/Documents/ros2_ws$ colcon build --packages-select cpp_pubsub
    Starting >>> cpp_pubsub
    Finished <<< cpp_pubsub [0.13s]                  
    Summary: 1 package finished [0.30s]
    ```

- The build was succesful, our package is ready to be used. To do so we simply had to run the talker and listener in two distinct terminals.

    ```bash
    #First terminal
    lorenz@Legion:~/Documents/ros2_ws$ . install/setup.bash
    lorenz@Legion:~/Documents/ros2_ws$ ros2 run cpp_pubsub talker
    [INFO] [1731851398.103893880] [minimal_publisher]: Publishing: 'Hello, world! 0'
    [INFO] [1731851398.603926237] [minimal_publisher]: Publishing: 'Hello, world! 1'
    [INFO] [1731851399.104009651] [minimal_publisher]: Publishing: 'Hello, world! 2'
    [INFO] [1731851399.604020636] [minimal_publisher]: Publishing: 'Hello, world! 3'
    [INFO] [1731851400.104067241] [minimal_publisher]: Publishing: 'Hello, world! 4'
    [INFO] [1731851400.604100785] [minimal_publisher]: Publishing: 'Hello, world! 5'
    ```
    ```bash
    #Second terminal
    lorenz@Legion:~/Documents/ros2_ws$ . install/setup.bash
    lorenz@Legion:~/Documents/ros2_ws$ ros2 run cpp_pubsub listener
    [INFO] [1731851398.104100483] [minimal_subscriber]: I heard: 'Hello, world! 0'
    [INFO] [1731851398.604107905] [minimal_subscriber]: I heard: 'Hello, world! 1'
    [INFO] [1731851399.104287497] [minimal_subscriber]: I heard: 'Hello, world! 2'
    [INFO] [1731851399.604303230] [minimal_subscriber]: I heard: 'Hello, world! 3'
    [INFO] [1731851400.104361709] [minimal_subscriber]: I heard: 'Hello, world! 4'
    [INFO] [1731851400.604389317] [minimal_subscriber]: I heard: 'Hello, world! 5'
    ```

### Turtlesim Publisher

- We launched both the turtlesim_node and turtle_teleop_key nodes in two terminals :

    ```bash
    #First terminal
    lorenz@Legion:~/Documents/ros2_ws$ ros2 run turtlesim turtlesim_node
    qt.qpa.plugin: Could not find the Qt platform plugin "wayland" in ""
    [INFO] [1731852837.026445475] [turtlesim]: Starting turtlesim with node name /turtlesim
    [INFO] [1731852837.029203595] [turtlesim]: Spawning turtle [turtle1] at x=[5,544445], y=[5,544445], theta=[0,000000]
    ```

    ```bash
    #Second terminal
    lorenz@Legion:~/Documents/ros2_ws$ ros2 run turtlesim turtle_teleop_key
    Reading from keyboard
    ---------------------------
    Use arrow keys to move the turtle.
    Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.
    'Q' to quit.
    ```

-  Then we used the following commands to find on which topic and what type of data we need to send to move the turtle.

    ```bash
    lorenz@Legion:~/Documents/ros2_ws$ ros2 topic list
    /parameter_events
    /rosout
    /turtle1/cmd_vel
    /turtle1/color_sensor
    /turtle1/pose

    lorenz@Legion:~/Documents/ros2_ws$ ros2 topic info /turtle1/cmd_vel
    Type: geometry_msgs/msg/Twist
    Publisher count: 1
    Subscription count: 1
    ```

    **Topic:** `/turtle1/cmd_vel` 
    
    **Data:** `geometry_msgs/msg/Twist`

- Now that we know on wich topic and what data we need to publish we built a new package named `turtle_control` that used most of the code from our very first package `cpp_pubsub`.

    We only had a few lines to modify for this to work :

    **turtle_publisher.cpp**

    ```cpp
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
    ```

    **CMakeLists.txt**

    ```cpp
    //add the following
    find_package(geometry_msgs REQUIRED)

    add_executable(turtle_publisher src/turtle_publisher.cpp)  
    ament_target_dependencies(turtle_publisher rclcpp geometry_msgs)

    install(TARGETS turtle_publisher
            DESTINATION lib/${PROJECT_NAME})
    ```

    **package.xml**

    ```xml
    <!--add the following-->
    <depend>geometry_msgs</depend>
    ```

- Perfect ! Now we simply need to build our package, run it and turtlesim to enjoy a turtle make circles. xD


    ```bash
    lorenz@Legion:~/Documents/ros2_ws$ ros2 run turtle_control turtle_publisher
    ```
    
    ![Turtle Making Circles](/Lab1&2/img/04.png)

    
