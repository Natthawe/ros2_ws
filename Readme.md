Day 1
Create Packages C++
-------

-ros2 pkg create my_custom_pkg --build-type ament_cmake

-create node c++

-create node python

-config package.xml

-config CMakeLists.txt

Day 2
Create a Launch File with Python
-------

-create file name -> test.launch.py

    - add node
    
        - package
        
        - executable
        
        - name
        
        - remappings
        
        - parameters

-config package.xml

-config CMakeLists.txt


Day 3
Creating a YAML Parameters
-------

-ros2 pkg create --build-type ament_python my_python_pkg

-create params_node.py

-create params.yaml

-create launch file


Day 4
Create Packages Differential robot
-------

-ros2 pkg create --build-type ament_cmake differential_robot

-create differential_robot.launch.py

-create differential_robot.urdf

-add robot_base.stl

-add rviz2_settings.rviz

Day 5
C++ Nodes Development in Depth
-------

-ros2 pkg create --build-type ament_cmake nodes

#### Check msg
-ls /opt/ros/humble/include/std_msgs/std_msgs/msg/

#### Check laserscan_msg
-ls /opt/ros/humble/include/sensor_msgs/sensor_msgs/msg/laser_scan.hpp
-ros2 interface show sensor_msgs/msg/LaserScan

Run
------
    ros2 run nodes laser

Day 6
-------
[PCL](https://pointclouds.org/downloads/)

    sudo apt install libpcl-dev    

[rtabmap_ros](https://github.com/introlab/rtabmap_ros/tree/humble-devel)

[turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations/tree/humble-devel)

####rtabmap
    sudo apt install ros-humble-rtabmap

####rtabmap-ros
    sudo apt install ros-humble-rtabmap-ros

####pcl-msgs
    sudo apt install ros-humble-pcl-msgs    

####pcl-ros
    sudo apt install ros-humble-pcl-ros

####pcl-conversions
    sudo apt install ros-humble-pcl-conversions