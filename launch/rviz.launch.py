import os,xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from  launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_name =get_package_share_directory('kuka_robot')

    xacro_file = os.path.join(pkg_name, 'urdf', 'kr210.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output ='screen',
        parameters=[{'robot_description':robot_description}]
    )
    rviz_node =Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
    )
    joint_state_gui=Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output= 'screen'
    )

    ld=LaunchDescription()

    ld.add_action(robot_state_publisher)
    ld.add_action(rviz_node)
    ld.add_action(joint_state_gui)

    return ld