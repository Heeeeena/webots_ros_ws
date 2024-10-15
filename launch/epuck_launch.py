import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection


def generate_launch_description():
    package_dir = get_package_share_directory('webots_demo')
    world = LaunchConfiguration('world',default='epuck_world.wbt')
    use_rviz = LaunchConfiguration('rviz', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_slam_cartographer = LaunchConfiguration('cartographer', default=True)
    random_walk = LaunchConfiguration('walk', default=False)


    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )

    # ROS control spawners
    controller_manager_timeout = ['--controller-manager-timeout', '500']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

    robot_description_path = os.path.join(package_dir, 'resource', 'epuck_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    use_twist_stamped = 'ROS_DISTRO' in os.environ and os.environ['ROS_DISTRO'] == 'rolling'
    if use_twist_stamped:
        mappings = [('/diffdrive_controller/cmd_vel', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    else:
        mappings = [('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel'), ('/diffdrive_controller/odom', '/odom')]
    epuck_driver = WebotsController(
        robot_name='e-puck',
        parameters=[
            {'robot_description': robot_description_path,
             'use_sim_time': use_sim_time,
             'set_robot_state_publisher': True},
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )

    # RViz
    rviz_config = os.path.join(get_package_share_directory('webots_demo'), 'rviz', 'all.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    # SLAM
    slam_nodes = []
    # SLAM:cartographer
    cartographer_config_dir = os.path.join(package_dir, 'config')
    cartographer_config_basename = 'carto_config.lua'
    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', cartographer_config_basename],
        condition=launch.conditions.IfCondition(use_slam_cartographer))
    slam_nodes.append(cartographer)

    grid_executable = 'cartographer_occupancy_grid_node'
    cartographer_grid = Node(
        package='cartographer_ros',
        executable=grid_executable,
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', '0.05'],
        condition=launch.conditions.IfCondition(use_slam_cartographer))
    slam_nodes.append(cartographer_grid)

    # SLAM:GMapping
    # gmapping = Node(
    #     package='slam_gmapping',
    #     executable='slam_gmapping',
    #     name='slam_gmapping',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}, {'delta_':'0.02'}],
    #     arguments=['-delta', '0.02'],
    #     condition=launch.conditions.IfCondition(use_slam_gmapping))
    # navigation_nodes.append(gmapping)

    # 墙壁跟随节点创建
    obstacle_avoider = Node(
        package='webots_demo',
        executable='random_walk',
        output='screen',
        condition=launch.conditions.IfCondition(random_walk))
    
    #扫描节点的创建
    epuck_scan = Node(
        package='webots_demo',
        executable='epuck_scan',
        output='screen')

    
    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=epuck_driver,
        nodes_to_start=[rviz] + slam_nodes + ros_control_spawners + [obstacle_avoider]
    )

    return LaunchDescription([
        webots,
        webots._supervisor,

        robot_state_publisher,
        footprint_publisher,

        epuck_driver,
        
        epuck_scan,
        waiting_nodes,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])
