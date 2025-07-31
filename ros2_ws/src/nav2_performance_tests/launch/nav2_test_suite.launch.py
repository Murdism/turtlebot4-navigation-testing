from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Individual coordinate arguments
    declare_start_x = DeclareLaunchArgument('start_x', default_value='0.0')
    declare_start_y = DeclareLaunchArgument('start_y', default_value='0.0')
    declare_start_z = DeclareLaunchArgument('start_z', default_value='0.0')
    declare_start_yaw = DeclareLaunchArgument('start_yaw', default_value='0.0')
    declare_goal_x = DeclareLaunchArgument('goal_x', default_value='2.0')
    declare_goal_y = DeclareLaunchArgument('goal_y', default_value='2.0')
    declare_goal_z = DeclareLaunchArgument('goal_z', default_value='0.0')
    declare_goal_yaw = DeclareLaunchArgument('goal_yaw', default_value='1.0')
    declare_test_file = DeclareLaunchArgument('test_file', default_value='')
    
    # Combined coordinate arguments (alternative)
    declare_start = DeclareLaunchArgument('start', default_value='')  # Format: "x,y" or "x,y,z,yaw"
    declare_goal = DeclareLaunchArgument('goal', default_value='')   # Format: "x,y" or "x,y,z,yaw"
    
    # Other arguments
    declare_dist_thres = DeclareLaunchArgument('dist_thres', default_value='0.30')
    declare_test_name = DeclareLaunchArgument('test_name', default_value='test_01')
    declare_repetitions = DeclareLaunchArgument('repetitions', default_value='1')
    declare_world_name = DeclareLaunchArgument('world_name', default_value='depot')
    declare_entity_name = DeclareLaunchArgument('entity_name', default_value='turtlebot4')
    

    def parse_and_launch(context):
        """Parse combined parameters or use individual ones"""
        # Get parameter values
        start_combined = LaunchConfiguration('start').perform(context)
        goal_combined = LaunchConfiguration('goal').perform(context)
        
        def parse_coordinate_string(coord_str, defaults):
            """Parse coordinate string like '1.0,2.0' or '1.0,2.0,0.0,0.5'"""
            if not coord_str:
                return defaults
            
            parts = [float(x.strip()) for x in coord_str.split(',')]
            
            # Fill missing values with defaults
            result = list(defaults)
            for i, part in enumerate(parts):
                if i < len(result):
                    result[i] = part
            return tuple(result)
        
        # Parse start coordinates into tuple
        if start_combined:
            start_coords = parse_coordinate_string(start_combined, (0.0, 0.0, 0.0, 0.0))
        else:
            start_coords = (
                float(LaunchConfiguration('start_x').perform(context)),
                float(LaunchConfiguration('start_y').perform(context)),
                float(LaunchConfiguration('start_z').perform(context)),
                float(LaunchConfiguration('start_yaw').perform(context))
            )
        
        # Parse goal coordinates into tuple
        if goal_combined:
            goal_coords = parse_coordinate_string(goal_combined, (2.0, 2.0, 0.0, 1.0))
        else:
            goal_coords = (
                float(LaunchConfiguration('goal_x').perform(context)),
                float(LaunchConfiguration('goal_y').perform(context)),
                float(LaunchConfiguration('goal_z').perform(context)),
                float(LaunchConfiguration('goal_yaw').perform(context))
            )

        world_name = LaunchConfiguration('world_name').perform(context)
        test_file = LaunchConfiguration('test_file').perform(context)
        
        # Dynamic map path based on world name
        map_path = f'/opt/ros/jazzy/share/turtlebot4_navigation/maps/{world_name}.yaml'
        
        tb4_gz_launch_path = os.path.join(
            FindPackageShare('turtlebot4_gz_bringup').find('turtlebot4_gz_bringup'),
            'launch', 'turtlebot4_gz.launch.py'
        )

        tb4_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tb4_gz_launch_path),
            launch_arguments={
                'use_sim_time': 'true',
                'nav2': 'true',
                'slam': 'false',
                'localization': 'true',
                'rviz': 'true',
                'world': world_name,
                'map': map_path,
                'x': str(start_coords[0]),
                'y': str(start_coords[1]),
                'z': str(start_coords[2]),
                'yaw': str(start_coords[3]),
            }.items()
        )

        nav2_test_action_server_node = Node(
            package='nav2_performance_tests',
            executable='Nav2TestNode',
            name='Nav2TestNode',
            output='screen',
            parameters=[{
                'start_x': start_coords[0],
                'start_y': start_coords[1],
                'start_z': start_coords[2],
                'start_yaw': start_coords[3],
                'goal_x': goal_coords[0],
                'goal_y': goal_coords[1],
                'goal_z': goal_coords[2],
                'goal_yaw': goal_coords[3],
                'dist_thres': float(LaunchConfiguration('dist_thres').perform(context)),
                'test_name': LaunchConfiguration('test_name').perform(context),
                'repetitions': int(LaunchConfiguration('repetitions').perform(context)),
                'world_name': world_name,
                'test_file': test_file,
                'entity_name': LaunchConfiguration('entity_name').perform(context),
                'use_sim_time': True 
            }]
        )
        
        return [tb4_launch, nav2_test_action_server_node]

    return LaunchDescription([
        declare_start_x, declare_start_y, declare_start_z, declare_start_yaw,
        declare_goal_x, declare_goal_y, declare_goal_z, declare_goal_yaw,
        declare_start, declare_goal,
        declare_dist_thres, declare_test_name, declare_repetitions,
        declare_world_name, declare_entity_name,declare_test_file,
        OpaqueFunction(function=parse_and_launch)
    ])