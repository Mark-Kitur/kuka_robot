import os,xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from  launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
    robot_1=Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','robot_description',
                   '-entity','robot',
                   '-x','0.0',
                   '-y','0.0',
                   '-z','0.2'],
        output='screen'
    )

    robot_2=Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','robot_description',
                   '-entity','robot2',
                   '-x','5.0',
                   '-y','5.0',
                   '-z','0.2'],
        output='screen'
    )



    gazebo_node =IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'
        )]),
        launch_arguments={
            'verbose':'true'
        }.items()
    )

        # load controllers
    # Spawn joint state broadcaster first
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawn arm controller
    arm_controller_s = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawn gripper controller
    gripper_controller_s = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # timer

    jiont_t =TimerAction(
        period=0.0, actions=[joint_state_broadcaster]
    )
    arm_t =TimerAction(
        period=6.0, actions=[arm_controller_s]
    )
    gripper_t =TimerAction(
        period=4.0, actions=[gripper_controller_s]
    )


    ld=LaunchDescription()

    ld.add_action(robot_state_publisher)
    ld.add_action(gazebo_node)
    ld.add_action(robot_1)
    # ld.add_action(robot_2)
    ld.add_action(jiont_t)
    ld.add_action(arm_t)
    ld.add_action(gripper_t)
    # ld.add_action(rviz_node)
    # ld.add_action(joint_state_gui)

    return ld