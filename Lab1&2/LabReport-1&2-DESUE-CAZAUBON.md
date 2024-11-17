

# Report: ROS Lab Sessions 1 & 2 CAZAUBON Lorenz - DESUE Léo

## Table of Contents
- [Report: ROS Lab Sessions 1 \& 2](#report-ros-lab-sessions-1--2)
  - [Table of Contents](#table-of-contents)
  - [Lab 1](#lab-1)
    - [First Hello World package](#first-hello-world-package)
    - [Turtlesim](#turtlesim)


<div style="page-break-after: always;"></div>



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


<div style="page-break-after: always;"></div>



### Turtlesim

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

-  Then we used the following command to find what type of data we need to send to move the turtle.

    ```bash
    lorenz@Legion:~/Documents/ros2_ws$ ros2 topic list
    /parameter_events
    /rosout
    /turtle1/cmd_vel
    /turtle1/color_sensor
    /turtle1/pose
    ```

    The answer is `/turtle1/cmd_vel`

- Now that we know wich type of data we need to send, we created a new package called `turtle_control`, in this package we added a **.cpp** 