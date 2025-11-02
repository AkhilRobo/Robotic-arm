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

    # ==================================================================
    # === 1. PANDA ROBOT 
    # ==================================================================


    xacro_path = 'urdf/panda.urdf.xacro'
    
    robot_description_path = PathJoinSubstitution([
        get_package_share_directory('panda_description'),   
        xacro_path
    ])
    panda_robot_description_cmd = Command(['xacro ', robot_description_path])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher', 
        output='screen',        
        parameters=[{
            'robot_description': panda_robot_description_cmd,
            'use_sim_time': use_sim_time 
        }]
    )

    workspace_path = os.environ.get('COLCON_PREFIX_PATH') or os.environ.get('AMENT_PREFIX_PATH')
    pkg_panda_description = os.path.join(workspace_path, "panda_description", "share")
    gz_models_path = "/home/akhil/.ignition/models"
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[f"{pkg_panda_description}:{gz_models_path}"]
    )

    spawn_node = Node(package='ros_gz_sim', executable='create',
                 arguments=['-name', 'panda', '-string', panda_robot_description_cmd , 
                            '-x', '0.438782',
                            '-y', '0.134319',
                            '-z', '0.775935'], 
                            output='screen')

    
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

  # ==================================================================
    # === 2. SENSOR POLE (Corrected with Namespace)
    # ==================================================================

    sensor_pole_xacro_file = '/home/akhil/panda_arm/panda_description/urdf/CamToHand.xacro'
    sensor_pole_robot_description = Command(['xacro ', sensor_pole_xacro_file])

    # This node publishes TFs AND the URDF for the SENSOR POLE
    sensor_pole_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='sensor_pole_robot_state_publisher',
        output='screen',
        namespace='sensor_pole',  # <-- PUT IN NAMESPACE
        parameters=[{
            'robot_description': sensor_pole_robot_description,
            'use_sim_time': use_sim_time,
            'publish_robot_description': True  # <-- ADD THIS
        }]
        # This will PUBLISH the URDF to /sensor_pole/robot_description
        # It will SUBSCRIBE to /sensor_pole/joint_states
    )

    # This is the GUI you wanted, to control the TFs
    sensor_pole_joint_state_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='sensor_pole_joint_state_publisher_gui', 
        output='screen',
        namespace='sensor_pole'  # <-- PUT IN NAMESPACE
        # We REMOVED the parameters. It will now subscribe to the
        # /sensor_pole/robot_description topic, which is correct.
        # It will PUBLISH to /sensor_pole/joint_states
    )

    # This spawns the SENSOR POLE
    sensor_pole_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_sensor_pole', 
        output='screen',
        arguments=[
            '-name', 'sensor_pole',
            '-string', sensor_pole_robot_description, # This is fine
            '-x', '0.0',   
            '-y', '-2.0', 
            '-z', '0.0'
        ]
    )

    # ==================================================================
    # === 3. GAZEBO & BRIDGES
    # ==================================================================
    
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
    
    color_camera_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
            name = 'color_camera_bridge',
            
    
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments = ['/color_camera' + '@sensor_msgs/msg/Image' + '[ignition.msgs.Image'],
            remappings = [('/color_camera', '/color_camera')]
    )
    
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
            parameters=[{'use_sim_time': True}],
            arguments = [
                '/depth_camera' + '@sensor_msgs/msg/Image' + '[ignition.msgs.Image',
                '/depth_camera/points' + '@sensor_msgs/msg/PointCloud2' + '[ignition.msgs.PointCloudPacked'
            ],
            remappings = [
                ('/depth_camera', '/depth_camera'),
                ('/depth_camera/points', '/depth_camera/points')
            ]
    )
    
    depth_cam_data2cam_link_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='cam3Tolink',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link', 'panda/link0/d435_depth'])


    # ==================================================================
    # === 4. ADD ACTIONS TO LAUNCH DESCRIPTION
    # ==================================================================
    
    ld.add_action(use_sim_time_arg)
    ld.add_action( gz_resource_path )
    ld.add_action( ignition_gazebo_node )

    ld.add_action( robot_state_publisher_node )
    ld.add_action( spawn_node )
    ld.add_action( load_joint_state_broadcaster )
    ld.add_action( load_position_controller )  
    ld.add_action( load_eef_controller )  

    ld.add_action( sensor_pole_rsp_node )
    ld.add_action( sensor_pole_joint_state_gui )
    ld.add_action( sensor_pole_spawn_node )
         
    ld.add_action( color_camera_bridge )
    ld.add_action( depth_camera_bridge )
    ld.add_action( depth_cam_data2cam_link_tf )
    ld.add_action(clock_bridge) 

    return ld