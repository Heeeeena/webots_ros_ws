import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction

def generate_launch_description():
    package_dir = get_package_share_directory('webots_demo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = LaunchConfiguration('world',default='gazebo_simple_world.world')
    use_rviz = LaunchConfiguration('rviz', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    use_slam_cartographer = LaunchConfiguration('cartographer', default=True)
    random_walk = LaunchConfiguration('walk', default=False)
    # explore = LaunchConfiguration('explore', default=False)


    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            package_dir,
            'worlds',
            'gazebo_simple_world.world'
        ])}.items(),
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
    # Robot controller node (replacing WebotsController)
    epuck_driver = Node(
        package='controller_manager',
        executable='spawner',
        name='e_puck_driver',
        parameters=[{
            'robot_description': robot_description_path,
            'use_sim_time': use_sim_time,
            'set_robot_state_publisher': True, 
        },
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
        arguments=['-resolution', '0.02'],
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

    # 探索节点创建
    # explore_node = Node(
    #     package='webots_demo',
    #     executable='explore',
    #     output='screen',
    #     condition=launch.conditions.IfCondition(explore))
    
    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = [rviz] + slam_nodes + ros_control_spawners
    return LaunchDescription([
        gz_sim,

        robot_state_publisher,
        footprint_publisher,

        epuck_driver,
        
        epuck_scan,
        rviz,
        slam_nodes[0],
        slam_nodes[1],
        ros_control_spawners[0],
        ros_control_spawners[1],

        TimerAction(
            period=5.0,  # 延迟时间，单位为秒
            actions=[
                obstacle_avoider
            ]
        ),

        # This action will kill all nodes once the Webots simulation has exited
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=gz_sim,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown())
        #         ],
        #     )
        # )
    ])
