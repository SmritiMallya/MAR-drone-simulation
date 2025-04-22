# Improved Drone Model

This repository contains the Unified Robot Description Format (URDF) file for an improved drone model designed for use in robotics simulations, particularly with ROS (Robot Operating System) and Gazebo.

## Overview

The `ImprovedDrone.urdf.xacro` file defines a quadcopter robot with the following features:

* **Improved Visuals:** Defined materials with more distinct and appealing colors for different parts of the drone.
* **Detailed Base Link:** The main body includes both visual and collision geometries, along with realistic inertia properties.
* **Onboard Camera:** A camera sensor is attached to the front of the drone, providing simulated visual data. It also includes a Gazebo camera plugin for generating image and camera info topics.
* **LED Indicators:** A front-facing LED for visual indication.
* **Robust Arms:** Defined arms with collision and inertial properties.
* **Landing Stand:** Includes four landing legs with foot pads for stable ground contact. The foot pads have higher friction in Gazebo.
* **Detailed Rotors:** Each rotor assembly consists of a rotor hub and two blades. They are defined with their own inertia and connected to the base link via continuous joints for rotation.
* **Motor Simulation:** Gazebo motor plugins are included for each rotor to simulate motor dynamics and receive motor speed commands.
* **IMU Sensor:** An Inertial Measurement Unit (IMU) is included for sensing the drone's orientation and angular velocities. A Gazebo IMU plugin publishes IMU data.
* **Physics and Control Plugins:** Includes Gazebo plugins for multirotor physics and ROS 2 control interface.

## File Structure

* `ImprovedDrone.urdf.xacro`: The main URDF file in Xacro format, defining the robot model.

## Dependencies

This model is primarily designed for use with:

* **ROS (Robot Operating System):** For robot control, data processing, and communication.
* **Gazebo Simulator:** For simulating the robot in a realistic 3D environment, including physics and sensor data.
* **`xacro`:** A macro language for XML, used to create concise and reusable robot descriptions.
* **`gazebo_ros` and `gazebo_ros2_control`:** ROS packages providing interfaces between Gazebo and ROS/ROS 2.
* **`libgazebo_motor_model.so`:** A Gazebo plugin for simulating rotor motor dynamics (typically part of the `gazebo_ros_pkgs`).
* **`libgazebo_ros_camera.so`:** A Gazebo plugin for simulating camera sensors and publishing data to ROS topics (typically part of the `gazebo_ros_pkgs`).
* **`libgazebo_ros_imu_sensor.so`:** A Gazebo plugin for simulating IMU sensors and publishing data to ROS topics (typically part of the `gazebo_ros_pkgs`).
* **`libgazebo_multirotor_base_plugin.so`:** A Gazebo plugin providing basic multirotor physics and control (typically part of the `rotors_gazebo_plugins` or similar).

## How to Use

1.  **Ensure Dependencies are Installed:** Make sure you have ROS, Gazebo, and the necessary ROS-Gazebo packages installed on your system.
2.  **Save the File:** Save the provided XML code as `ImprovedDrone.urdf.xacro` in a suitable location within your ROS workspace (e.g., in a `urdf` folder within your robot's package).
3.  **Create a ROS Package (if you don't have one):**
    ```bash
    cd ~/your_ros2_workspace/src
    ros2 pkg create improved_drone_description --build-type ament_cmake --dependencies urdf xacro gazebo_ros
    cd improved_drone_description
    mkdir urdf
    mv path/to/ImprovedDrone.urdf.xacro urdf/
    mkdir launch
    ```
4.  **Create a Launch File (for ROS 2):** Create a launch file (e.g., `display.launch.py` in the `launch` folder) to load the URDF into the ROS parameter server and spawn it in Gazebo.

    ```python
    import launch
    from launch.substitutions import Command, LaunchConfiguration
    from launch_ros.actions import LifecycleNode, Node
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch.conditions import IfCondition
    from ament_cmake_core.packages import get_package_share_directory
    import os

    def launch_setup(context, *args, **kwargs):
        pkg_share = get_package_share_directory('improved_drone_description')
        urdf_path = os.path.join(pkg_share, 'urdf', 'ImprovedDrone.urdf.xacro')

        robot_name = LaunchConfiguration('robot_name').perform(context)
        use_sim_time = LaunchConfiguration('use_sim_time')
        start_gazebo = LaunchConfiguration('start_gazebo')

        robot_description_content = Command(
            [
                'xacro',
                ' ',
                urdf_path,
                ' ',
                'name:=',
                robot_name,
            ]
        )
        robot_description = {'robot_description': robot_description_content}

        # Gazebo node
        gazebo = IfCondition(start_gazebo).then(
            launch.actions.ExecuteProcess(
                cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
                output='screen'
            )
        )

        # Spawn the robot
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', '/robot_description',
                       '-entity', robot_name],
            output='screen',
            condition=IfCondition(start_gazebo)
        )

        # Robot state publisher
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': use_sim_time}]
        )

        return [
            DeclareLaunchArgument('robot_name', default_value='improved_drone',
                                description='Name of the robot to spawn'),
            DeclareLaunchArgument('use_sim_time', default_value='true',
                                description='Use simulation (Gazebo) clock if true'),
            DeclareLaunchArgument('start_gazebo', default_value='false',
                                description='Launch Gazebo simulator'),
            gazebo,
            spawn_entity,
            robot_state_publisher,
        ]

    def generate_launch_description():
        return launch.LaunchDescription([
            OpaqueFunction(function=launch_setup)
        ])
    ```

5.  **Run the Simulation (ROS 2):**
    ```bash
    cd ~/your_ros2_workspace
    colcon build
    source install/setup.bash
    ros2 launch urdf_tutorial display.launch.py model:=<file_path>/drone.urdf
    ```

    Replace `improved_drone_description` with the actual name of your ROS package.

## Features and Improvements

* **Enhanced Visuals:** The drone now has more visually appealing and distinguishable materials for different components.
* **Collision and Inertia:** Collision models are defined for the base link and arms, and realistic inertia properties are set for better simulation accuracy.
* **Camera Integration:** An onboard camera is included with a Gazebo plugin to simulate vision capabilities.
* **Landing Gear:** The addition of landing legs provides a stable base for the drone when it's on the ground.
* **Detailed Rotors:** The rotor assemblies are modeled with blades, and Gazebo motor plugins are configured for realistic rotor dynamics.
* **IMU Sensor:** An IMU sensor is integrated for orientation and angular velocity feedback, crucial for control algorithms.
* **ROS 2 Control Ready:** The inclusion of the `gazebo_ros2_control` plugin allows for advanced control of the drone using ROS 2 control frameworks.

## Notes

* This URDF file is designed for simulation purposes and may not perfectly represent a real-world drone's physical properties.
* The Gazebo plugins used here require the corresponding packages to be installed in your ROS environment.
* You can further extend this model by adding more sensors, actuators, or modifying its physical properties as needed for your specific simulation scenarios.

## Author

Smriti Mallya
