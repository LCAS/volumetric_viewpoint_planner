from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config_lattice_publisher = PathJoinSubstitution(
        [FindPackageShare("viewpoint_generator"), "config", "params.yaml"]
    )

    # Configure lattice points for robot arm tool frame
    lattice_publisher = Node(
        package='viewpoint_generator',
        executable='sampling_surface',
        name='lattice_publisher',
        parameters=[config_lattice_publisher],
        output='screen',
    )

    static_transform_publisher_1 = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        name="static_tf_publisher_plant_to_world",
        output="log",
        arguments = ["1.0", "-0.3", "0", "3.1415", "0", "0", "plant", "panda_link0"] )
    
    static_transform_publisher_2 = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        name="static_tf_publisher_plant_to_arm",
        output="log",
        arguments = ["0.7", "-0.1", "0", "3.1415", "0", "0", "plant", "panda_link0"] )
    
    static_transform_publisher_3 = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        name="static_tf_publisher_arm_to_plant",
        output="log",
        arguments = ["1.0", "0.0", "0", "3.1415", "0", "0", "panda_link0", "plant"] )

    ld = LaunchDescription()
    ld.add_action(lattice_publisher)
    ld.add_action(static_transform_publisher_3)

    return ld
