import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable,RegisterEventHandler,DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import (
    Command, 
    PathJoinSubstitution, 
    LaunchConfiguration
)

def generate_launch_description():

    ld = LaunchDescription()
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # --- 1. PANDA ROBOT ARM ---

    # MODIFIED: Renamed variable for clarity
    panda_xacro_path_str = 'urdf/panda.urdf.xacro' 
    
    # MODIFIED: Renamed variable for clarity
    panda_robot_description_path = PathJoinSubstitution([
        get_package_share_directory('panda_description'),   
        panda_xacro_path_str
    ])
    
    # MODIFIED: Renamed variable for clarity
    panda_robot_description = Command(['xacro ', panda_robot_description_path])

    set_prime_offload = SetEnvironmentVariable(
    name='__NV_PRIME_RENDER_OFFLOAD',
    value='1'
)

    set_glx_vendor = SetEnvironmentVariable(
    name='__GLX_VENDOR_LIBRARY_NAME',
    value='nvidia'
)

    # MODIFIED: Renamed node, added use_sim_time, and added remapping
    panda_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='panda_robot_state_publisher', # Unique name
        output='screen',        
        parameters=[{
            'robot_description': panda_robot_description,
            'use_sim_time': use_sim_time # Added this
        }],
        remappings=[ # Added this block
            ('/robot_description', '/panda_robot_description')
        ]
    )

    workspace_path = os.environ.get('COLCON_PREFIX_PATH') or os.environ.get('AMENT_PREFIX_PATH')

    # panda description share dir
    pkg_panda_description = os.path.join(workspace_path, "panda_description", "share")

    # default gazebo models location
    gz_models_path = "/home/akhil/.ignition/models"

    # set search path for resources
    gz_resource_path = SetEnvironmentVariable(
    name="GZ_SIM_RESOURCE_PATH",
    value=[f"{pkg_panda_description}:{gz_models_path}"]
)

    # MODIFIED: Renamed node and changed topic
    panda_spawn_node = Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'panda', 
                            '-topic', '/panda_robot_description' , # Listen on unique topic
                            '-x', '0.438782',
                            '-y', '0.134319',
                            '-z', '0.775935'], 
                            output='screen')

    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    

    sdf_file_path = os.path.join(
        FindPackageShare('panda_description').find('panda_description'),
        'world',
        'Mainworld.sdf'
    )
    ignition_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {sdf_file_path}',
            'use_sim_time': use_sim_time 
        }.items()
    )

    load_position_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    load_eef_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'eef_controller'],
        output='screen'
    )
    
    # --- 2. POLE SENSOR ---
    # NEW: Define the path to your pole sensor xacro
    # (Assuming it's named PoleSensor.xacro and is in the same folder)
    pole_sensor_xacro_path_str = 'urdf/CamToHand.xacro' 
    
    pole_sensor_robot_description_path = PathJoinSubstitution([
        get_package_share_directory('panda_description'),   
        pole_sensor_xacro_path_str
    ])
    
    pole_sensor_robot_description = Command(['xacro ', pole_sensor_robot_description_path])

    # NEW: Add a Robot State Publisher for the pole sensor
    pole_sensor_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='pole_sensor_robot_state_publisher', # Unique name
        output='screen',        
        parameters=[{
            'robot_description': pole_sensor_robot_description,
            'use_sim_time': use_sim_time
        }],
        remappings=[ # Remap to a unique topic
            ('/robot_description', '/pole_sensor_robot_description')
        ]
    )

    # NEW: Add a Spawn node for the pole sensor
    pole_sensor_spawn_node = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'pole_sensor',
                    '-topic', '/pole_sensor_robot_description' , # Listen on its unique topic
                    '-x', '0', # Spawn at 0,0,0 - your XACRO handles the offset
                    '-y', '0',
                    '-z', '0'], 
        output='screen'
    )

    # --- 3. BRIDGES (for Pole Sensor) ---
    # (These are correct as they were, they bridge the pole sensor's topics)
    color_camera_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
            name = 'color_camera_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }],
            arguments = [
                '/color_camera' + '@sensor_msgs/msg/Image' + '[ignition.msgs.Image'
            ],
            remappings = [
                ('/color_camera', '/color_camera')
            ])
    
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    depth_camera_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
            name = 'depth_camera_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }],
            arguments = [
                '/depth_camera' + '@sensor_msgs/msg/Image' + '[ignition.msgs.Image',
                '/depth_camera/points' + '@sensor_msgs/msg/PointCloud2' + '[ignition.msgs.PointCloudPacked'
            ],
            remappings = [
                ('/depth_camera', '/depth_camera'),
                ('/depth_camera/points', '/depth_camera/points')
            ])
    
    
    # --- 4. ADD ACTIONS TO LAUNCH ---

    ld.add_action(use_sim_time_arg)
    ld.add_action(set_prime_offload)
    ld.add_action(set_glx_vendor)
    ld.add_action( gz_resource_path )
    ld.add_action( ignition_gazebo_node )

    # Panda Actions
    ld.add_action( panda_robot_state_publisher_node ) # MODIFIED
    ld.add_action( panda_spawn_node ) # MODIFIED
    ld.add_action( load_joint_state_broadcaster )
    ld.add_action( load_position_controller )  
    ld.add_action( load_eef_controller )  
         
    # Pole Sensor Actions
    ld.add_action( pole_sensor_robot_state_publisher_node ) # NEW
    ld.add_action( pole_sensor_spawn_node ) # NEW

    # Bridge Actions
    ld.add_action( color_camera_bridge )
    ld.add_action( depth_camera_bridge )
    ld.add_action(clock_bridge) 

    return ld