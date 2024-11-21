from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    config_file = PathJoinSubstitution(
        [FindPackageShare("moveit2_commander_recorder"), "config", "parameters.yaml"]
    )

    # Configure lattice points for the robot arm tool frame
    moveit_commander = Node(
        package='moveit2_commander_recorder',
        executable='pose_commander',
        name='moveit_commander_node',
        parameters=[config_file],
        output='screen',
    )

    # Configure lattice points for the robot arm tool frame
    image_view_saver = Node(
        package='image_view',
        executable='image_saver',
        name='image_saver',
        parameters=[config_file],
        remappings=[
                ('/save', '/image_saver/save'),  # Remap the image save service
                ('/image', '/flir_camera/image_raw'),    # Remap image topic for capturing Ex: '/tip_camera_rgb/image_raw' '/image_color'
                ('/camera_info', '/flir_camera/camera_info') # Remap image_info topic for capturing Ex: '/tip_camera_rgb/camera_info'
            ],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(moveit_commander)
    ld.add_action(image_view_saver)

    return ld
