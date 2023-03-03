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