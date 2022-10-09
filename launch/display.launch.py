from tkinter.tix import Tree
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_rick_sim').find('rover_rick_sim')
    default_model_path = os.path.join(pkg_share, 'src/description/rover_rick.urdf.xacro')
    parameters_file_path = os.path.join(pkg_share, 'config/dual_ekf.yaml')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_path=os.path.join(pkg_share, 'world/my_world.sdf'),
    agriculture_path=os.path.join(pkg_share, 'world/actually_empty_world.world'),
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}, {"use_sim_time": True}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{"use_sim_time": True}]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{"use_sim_time": True}]
    )
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rover_rick', '-topic', 'robot_description', '-z', '0.166', '-x', '-0.5', '-y', '32.0', '-R', '0.0', '-P', '0.0', '-Y', '0.0'],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )
    gps_imu_covariance_node = launch_ros.actions.Node(
        package='rover_rick_sim',
        executable='gps_imu_covariance',
        name='gps_imu_covariance',
        output='screen',
        parameters=[{"use_sim_time": True,
                     "imu_covariance_x": 0.01,
                     "imu_covariance_y": 0.01,
                     "imu_covariance_z": 0.01,
                     "imu_covariance_roll": 2e-3,
                     "imu_covariance_pitch": 2e-3,
                     "imu_covariance_yaw": 2e-3,
                     "imu_covariance_linear_velocity_x": 1.7e-1,
                     "imu_covariance_linear_velocity_y": 1.7e-1,
                     "imu_covariance_linear_velocity_z": 1.7e-1,
                     "gps_covariance_x": 0.001,
                     "gps_covariance_y": 0.001,
                     "gps_covariance_z": 0.001,}]
    )
    static_publisher_node = launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_publisher',
            output='screen',
            arguments=["0", "0", "0", "0", "0", "0", "odom", "map"],
            parameters=[{"use_sim_time": True}]
        )
    
    efk_odom = launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_odom',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', 'odometry/local')]           
           )
    efk_map = launch_ros.actions.Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node_map',
	        output='screen',
            parameters=[parameters_file_path],
            remappings=[('odometry/filtered', 'odometry/global')]
           )       
    efk_nav = launch_ros.actions.Node(
            package='robot_localization', 
            executable='navsat_transform_node', 
            name='navsat_transform',
	        output='screen',
            parameters=[parameters_file_path],
            # from -> to
            remappings=[('/imu/data', '/sim/imu'),
                        ('/gps/fix', '/sim/gps'), 
                        ('/odometry/filtered', '/sim/odom_to_map')]           

           )

    nav2_costmap2d_node = launch_ros.actions.Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d_markers",
        name="nav2_costmap2d_node",
        parameters=[{"user_sim_time": True}],
        output="screen"
    )

    joycon_node = launch_ros.actions.Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )

    joy_converter_node = launch_ros.actions.Node(
        package='remote_control2',
        executable='cmd_vel_conversion',
        name='cmd_vel_conversion',
        parameters=[
            {'cmd_vel_topic' : '/sim/cmd_vel'},
        ]
    )
    
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path, description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', agriculture_path], output='screen'),

        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        # gps_imu_covariance_node,
        static_publisher_node,
        # efk_nav,
        rviz_node,
        # joycon_node,
        # joy_converter_node
    ])