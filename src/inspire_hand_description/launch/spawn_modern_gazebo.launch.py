import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Define package and file paths
    pkg_name = 'inspire_hand_description'
    urdf_file_name = 'handright9253.urdf' # Use the corrected URDF file
    
    pkg_share = get_package_share_directory(pkg_name)
    urdf_file_path = os.path.join(pkg_share, 'urdf', urdf_file_name)

    # 1. Read the URDF file
    with open(urdf_file_path, 'r') as f:
        robot_description = f.read()

    # 2. Launch Gazebo (Ignition)
    # This launch file starts the Gazebo server and GUI
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # You can specify a world file here. '-r' means run the world on startup.
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # 3. Publish the robot description so the spawner can find it
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
    )
    
    # 4. Spawn the robot in Gazebo
    # The 'create' node subscribes to the /robot_description topic,
    # converts the URDF to SDF, and spawns it.
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description', # The topic to listen for the URDF
            '-name', 'inspire_hand',      # The name of the model in Gazebo
            '-allow_renaming', 'true',
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher_node,
        spawn_node
    ])