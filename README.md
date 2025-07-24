Using ROS2 Jazzy and Gazebo Harmonic for autonomous navigation simulation of Scout_mini.

You need to do this:

1.Run the following command to clone the scout_1 repository to your local machine:

    git clone https://github.com/2024828/scout_1.git

2.The file uses some Gazebo models, which you can download from:https://drive.google.com/file/d/1tcfoLFReEW1XNHPUAeLpIz2iZXqQBvo_/view.

Make sure to let Gazebo know about their location by running:

    export GZ_SIM_RESOURCE_PATH=~/gazebo_models

3.Build the package and run:
    
    colcon build
    . install/setup.bash
    ros2 launch scout_1 spawn_robot.launch.py

Open a new terminal and run:
    
    . install/setup.bash
    ros2 launch scout_1 navigation_with_slam.launch.py

You can use the `2D goal pose` in the RViz2 toolbar to control the robot's movement and mapping. The robot will automatically plan the path.

4.More References:

  https://github.com/agilexrobotics/scout_ros2

  
